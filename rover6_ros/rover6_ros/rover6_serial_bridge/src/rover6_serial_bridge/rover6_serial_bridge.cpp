#include <rover6_serial_bridge/rover6_serial_bridge.h>


Rover6SerialBridge::Rover6SerialBridge(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    _roverNamespace = "rover6";
    nh.param<string>("/" + _roverNamespace + "/serial_port", _serialPort, "");
    nh.param<int>("/" + _roverNamespace + "/serial_baud", _serialBaud, 115200);
    nh.param<string>("/" + _roverNamespace + "/imu_frame_id", _imuFrameID, "bno055_imu");
    nh.param<string>("/" + _roverNamespace + "/enc_frame_id", _encFrameID, "encoders");
    nh.param<string>("/" + _roverNamespace + "/motors_topic", _motorsTopicName, "motors");
    nh.param<string>("/" + _roverNamespace + "/servos_topic", _servosTopicName, "servo_cmd");
    int num_servos = 0;
    nh.param<int>("/" + _roverNamespace + "/num_servos", num_servos, 16);
    _numServos = (unsigned int)num_servos;
    nh.param<int>("/" + _roverNamespace + "/front_tilter_servo_num", _frontTilterServoNum, 0);
    nh.param<int>("/" + _roverNamespace + "/back_tilter_servo_num", _backTilterServoNum, 1);
    nh.param<int>("/" + _roverNamespace + "/camera_pan_servo_num", _panServoNum, 2);
    nh.param<int>("/" + _roverNamespace + "/camera_tilt_servo_num", _tiltServoNum, 3);
    nh.param<int>("/" + _roverNamespace + "/pan_servo_num", _panServoNum, 2);
    nh.param<int>("/" + _roverNamespace + "/tilt_servo_num", _tiltServoNum, 3);

    imu_msg.header.frame_id = _imuFrameID;
    enc_msg.header.frame_id = _encFrameID;
    fsr_msg.header.frame_id = "fsr";
    safety_msg.header.frame_id = "safety";

    ina_msg.header.frame_id = "battery";
    ina_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;

    servo_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    servo_msg.layout.dim[0].size = _numServos;
    servo_msg.layout.dim[0].stride = 1;
    servo_msg.layout.dim[0].label = "servos";
    servo_msg.data.resize(_numServos);

    tof_msg.header.frame_id = "tof";

    _serialBuffer = "";
    _serialBufferIndex = 0;
    _currentBufferSegment = "";
    _readPacketNum = 0;
    _writePacketNum = 0;
    _recvCharIndex = 0;
    _recvCharBuffer = new char[0xfff];

    _dateString = new char[16];

    readyState = new StructReadyState;
    readyState->rover_name = "";
    readyState->is_ready = false;
    readyState->time_ms = 0;

    deviceStartTime = ros::Time::now();
    offsetTimeMs = 0;

    imu_pub = nh.advertise<sensor_msgs::Imu>("bno055", 100);
    enc_pub = nh.advertise<rover6_serial_bridge::Rover6Encoder>("encoders", 100);
    fsr_pub = nh.advertise<rover6_serial_bridge::Rover6FSR>("fsrs", 100);
    safety_pub = nh.advertise<rover6_serial_bridge::Rover6Safety>("safety", 100);
    ina_pub = nh.advertise<sensor_msgs::BatteryState>("battery", 10);
    servo_pub = nh.advertise<std_msgs::Int16MultiArray>("servo_pos", 10);
    tof_pub = nh.advertise<rover6_serial_bridge::Rover6TOF>("tof", 10);

    motors_sub = nh.subscribe(_motorsTopicName, 100, &Rover6SerialBridge::motorsCallback, this);
    servos_sub = nh.subscribe(_servosTopicName, 100, &Rover6SerialBridge::servosCallback, this);

    pid_service = nh.advertiseService("rover6_pid", &Rover6SerialBridge::set_pid, this);
    safety_service = nh.advertiseService("rover6_safety", &Rover6SerialBridge::set_safety_thresholds, this);
    menu_service = nh.advertiseService("rover6_menu", &Rover6SerialBridge::send_menu_event, this);

    ROS_INFO("Rover 6 serial bridge init done");
}


void Rover6SerialBridge::configure()
{
    ROS_INFO("Configuring serial device.");
    // attempt to open the serial port
    try
    {
        ROS_DEBUG_STREAM("Selected port: " << _serialPort);
        _serialRef.setPort(_serialPort);
        ROS_DEBUG_STREAM("Selected baud: " << _serialBaud);
        _serialRef.setBaudrate(_serialBaud);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        _serialRef.setTimeout(timeout);
        _serialRef.open();
        ROS_INFO("Serial device configured.");
    }
    catch (serial::IOException e)
    {
        ROS_ERROR_STREAM("Unable to open port: " << _serialPort);
        ROS_ERROR_STREAM("Serial exception: " << e.what());
        cerr << "Serial exception: " << e.what() << endl;
        throw;
    }
}

void Rover6SerialBridge::setStartTime(uint32_t time_ms) {
    deviceStartTime = ros::Time::now();
    offsetTimeMs = time_ms;
}

ros::Time Rover6SerialBridge::getDeviceTime(uint32_t time_ms) {
    return deviceStartTime + ros::Duration((double)(time_ms - offsetTimeMs) / 1000.0);
}

void Rover6SerialBridge::checkReady()
{
    ROS_INFO("Checking if the serial device is ready.");

    ros::Time begin_time = ros::Time::now();
    ros::Time write_time = ros::Time::now();
    ros::Duration general_timeout = ros::Duration(5.0);
    ros::Duration write_timeout = ros::Duration(1.0);

    writeSerial("?", "s", "rover6");

    while (!readyState->is_ready)
    {
        if (!ros::ok()) {
            break;
        }
        if ((ros::Time::now() - begin_time) > general_timeout) {
            throw ReadyTimeoutException;
        }
        if ((ros::Time::now() - write_time) > write_timeout) {
            ROS_INFO("Writing signal again");
            writeSerial("?", "s", "rover6");
            write_time = ros::Time::now();
        }
        if (_serialRef.available() > 2) {
            readSerial();
        }
    }

    if (readyState->is_ready) {
        setStartTime(readyState->time_ms);
        ROS_INFO_STREAM("Serial device is ready. Rover name is " << readyState->rover_name);
    }
    else {
        ROS_ERROR("Failed to receive ready signal!");
    }
}

bool Rover6SerialBridge::waitForPacketStart()
{
    stringstream msg_buffer;
    char c1, c2;
    ros::Time wait_time = ros::Time::now();
    ros::Duration wait_timeout = ros::Duration(0.05);
    while (true) {
        if (ros::Time::now() - wait_time > wait_timeout) {
            return false;
        }
        if (_serialRef.available() < 2) {
            continue;
        }
        c1 = _serialRef.read(1).at(0);
        if (c1 == PACKET_START_0) {
            c2 = _serialRef.read(1).at(0);
            if (c2 == PACKET_START_1) {
                return true;
            }
        }

        else if (c1 == PACKET_STOP || c2 == PACKET_STOP) {
            ROS_INFO_STREAM("Device message: " << msg_buffer.str());
            msg_buffer.str(std::string());
            return false;
        }
        else {
            msg_buffer << c1;
        }
    }
}

bool Rover6SerialBridge::readSerial()
{
    if (!waitForPacketStart()) {
        return false;
    }
    char c;
    _recvCharIndex = 0;
    while (true) {
        if (_serialRef.available()) {
            c = _serialRef.read(1).at(0);
            if (c == PACKET_STOP) {
                break;
            }
            _recvCharBuffer[_recvCharIndex] = c;
            _recvCharIndex++;
        }
    }
    _recvCharBuffer[_recvCharIndex] = '\0';

    // _serialBuffer = _serialRef.readline();
    _serialBuffer = string(_recvCharBuffer);
    // _serialBuffer = _serialBuffer.substr(0, _serialBuffer.length() - 1);  // remove newline character
    ROS_DEBUG_STREAM("Buffer: " << _serialBuffer);
    // at least 1 char for packet num
    // \t + at least 1 category char
    // 2 chars for checksum
    if (_serialBuffer.length() < 5) {
        ROS_ERROR_STREAM("Received packet has an invalid number of characters! " << _serialBuffer);
        _readPacketNum++;
        return false;
    }

    _serialBufferIndex = 0;
    uint8_t calc_checksum = 0;
    // compute checksum using all characters except the checksum itself
    for (size_t index = 0; index < _serialBuffer.length() - 2; index++) {
        calc_checksum += (uint8_t)_serialBuffer.at(index);
    }

    uint16_t recv_checksum = 0;
    try {
        recv_checksum = std::stoul(_serialBuffer.substr(_serialBuffer.length() - 2), nullptr, 16);
    }
    catch (exception& e) {
        ROS_ERROR_STREAM("Failed to parse checksum. Buffer: " << _serialBuffer << ". Exception: " << e.what());
        return false;
    }

    if (calc_checksum != recv_checksum) {
        // checksum failed
        ROS_ERROR("Checksum failed! recv %d != calc %d", calc_checksum, recv_checksum);
        ROS_ERROR_STREAM("Buffer: " << _serialBuffer);
        _readPacketNum++;
        return false;
    }

    // get packet num segment
    if (!getNextSegment()) {
        ROS_ERROR("Failed to find packet number segment! %s", _serialBuffer);
        _readPacketNum++;
        return false;
    }
    uint32_t recv_packet_num = (uint32_t)stoi(_currentBufferSegment);
    if (recv_packet_num != _readPacketNum) {
        ROS_ERROR("Received packet num doesn't match local count. recv %d != local %d", recv_packet_num, _readPacketNum);
        ROS_ERROR_STREAM("Buffer: " << _serialBuffer);
        _readPacketNum = recv_packet_num;
    }

    // find category segment
    if (!getNextSegment()) {
        ROS_ERROR_STREAM("Failed to find category segment! Buffer: " << _serialBuffer);
        _readPacketNum++;
        return false;
    }

    string category = _currentBufferSegment;
    // ROS_INFO_STREAM("category: " << category);

    // remove checksum
    _serialBuffer = _serialBuffer.substr(0, _serialBuffer.length() - 2);

    try {
        processSerialPacket(category);
    }
    catch (exception& e) {
        ROS_ERROR_STREAM("Exception in processSerialPacket: " << e.what());
        return false;
    }

    _readPacketNum++;
    return true;
}

bool Rover6SerialBridge::getNextSegment()
{
    if (_serialBufferIndex >= _serialBuffer.length()) {
        return false;
    }
    size_t separator = _serialBuffer.find('\t', _serialBufferIndex);
    if (separator == std::string::npos) {
        _currentBufferSegment = _serialBuffer.substr(_serialBufferIndex);
        _serialBufferIndex = _serialBuffer.length();
        return true;
    }
    else {
        _currentBufferSegment = _serialBuffer.substr(_serialBufferIndex, separator - _serialBufferIndex);
        _serialBufferIndex = separator + 1;
        return true;
    }
}


void Rover6SerialBridge::processSerialPacket(string category)
{
    if (category.compare("txrx") == 0) {
        CHECK_SEGMENT(0); unsigned long long packet_num = (unsigned long long)stol(_currentBufferSegment);
        CHECK_SEGMENT(1); int error_code = stoi(_currentBufferSegment);
        // CHECK_SEGMENT(2); string message = _currentBufferSegment;

        if (error_code != 0) {
            logPacketErrorCode(error_code, packet_num);
        }
    }
    else if (category.compare("bno") == 0) {
        parseImu();
    }
    else if (category.compare("enc") == 0) {
        parseEncoder();
    }
    else if (category.compare("fsr") == 0) {
        parseFSR();
    }
    else if (category.compare("safe") == 0) {
        parseSafety();
    }
    else if (category.compare("ina") == 0) {
        parseINA();
    }
    else if (category.compare("ir") == 0) {
        parseIR();
    }
    else if (category.compare("servo") == 0) {
        parseServo();
    }
    else if (category.compare("lox") == 0) {
        parseTOF();
    }
    else if (category.compare("ready") == 0) {
        CHECK_SEGMENT(0); readyState->time_ms = (uint32_t)stoi(_currentBufferSegment);
        CHECK_SEGMENT(1); readyState->rover_name = _currentBufferSegment;
        readyState->is_ready = true;
        ROS_INFO_STREAM("Received ready signal! Rover name: " << _currentBufferSegment);
    }
}


void Rover6SerialBridge::writeSerial(string name, const char *formats, ...)
{
    va_list args;
    va_start(args, formats);
    string packet;
    stringstream sstream;
    sstream << PACKET_START_0 << PACKET_START_1 << _writePacketNum << "\t" << name;

    while (*formats != '\0') {
        sstream << "\t";
        if (*formats == 'd') {
            int i = va_arg(args, int32_t);
            sstream << i;
        }
        else if (*formats == 'u') {
            uint32_t u = va_arg(args, uint32_t);
            sstream << u;
        }
        else if (*formats == 's') {
            char *s = va_arg(args, char*);
            sstream << s;
        }
        else if (*formats == 'f') {
            double f = va_arg(args, double);
            sstream << fixed << setprecision(4) << f;
        }
        else {
            ROS_ERROR("Invalid format %c", *formats);
        }
        ++formats;
    }
    va_end(args);

    packet = sstream.str();

    uint8_t calc_checksum = 0;
    for (size_t index = 2; index < packet.length(); index++) {
        calc_checksum += (uint8_t)packet.at(index);
    }
    ROS_DEBUG("calc_checksum: %d", calc_checksum);

    if (calc_checksum < 0x10) {
        sstream << "0";
    }
    // can't pass uint8_t to std::hex, has to be int
    sstream << std::hex << (int)calc_checksum;

    sstream << PACKET_STOP;

    packet = sstream.str();

    // checksum might be inserting null characters. Force the buffer to extend
    // to include packet stop and checksum

    ROS_DEBUG_STREAM("Writing: " << packet);
    _serialRef.write(packet);
    _writePacketNum++;
    ros::Duration(0.0005).sleep();
}

void Rover6SerialBridge::setup()
{
    configure();

    // wait for startup messages from the microcontroller
    checkReady();

    // tell the microcontroller to start
    resetSensors();
    setActive(true);
    setReporting(true);
}


void Rover6SerialBridge::loop()
{
    // if the serial buffer has data, parse it
    if (_serialRef.available() > 2) {
        while (_serialRef.available()) {
            readSerial();
        }
    }

    ROS_INFO_THROTTLE(15, "Read packet num: %llu", _readPacketNum);
}

void Rover6SerialBridge::stop()
{
    // setActive(false);
    // setReporting(false);
    _serialRef.close();
}


int Rover6SerialBridge::run()
{
    setup();

    ros::Rate clock_rate(120);  // run loop at 120 Hz

    int exit_code = 0;
    while (ros::ok())
    {
        // let ROS process any events
        ros::spinOnce();
        clock_rate.sleep();

        try {
            loop();
        }
        catch (exception& e) {
            ROS_ERROR_STREAM("Exception in main loop: " << e.what());
            exit_code = 1;
            break;
        }
    }
    stop();

    return exit_code;
}

void Rover6SerialBridge::motorsCallback(const rover6_serial_bridge::Rover6Motors::ConstPtr& msg) {
    // motor commands in ticks per second
    // ROS_DEBUG("left motor: %f, right motor: %f", msg->left, msg->right);
    // if (motors_msg.left != msg->left || motors_msg.right != msg->right) {
    writeSerial("m", "ff", msg->left, msg->right);
    //     motors_msg.left = msg->left;
    //     motors_msg.right = msg->right;
    // }
}

void Rover6SerialBridge::servosCallback(const rover6_serial_bridge::Rover6Servos::ConstPtr& msg) {
    if (servos_msg.camera_tilt != msg->camera_tilt) {
        writeServo(_tiltServoNum, msg->camera_tilt, msg->mode);
        servos_msg.camera_tilt = msg->camera_tilt;
    }
    if (servos_msg.camera_pan != msg->camera_pan) {
        writeServo(_panServoNum, msg->camera_pan, msg->mode);
        servos_msg.camera_pan = msg->camera_pan;
    }
}

void Rover6SerialBridge::writeServo(unsigned int n, float command, uint8_t mode) {
    if (mode == rover6_serial_bridge::Rover6Servos::POSITION_MODE) {
        int command_int = (int)command;
        if (command_int == -1) {
            writeSerial("sd", "d", n);
        }
        else if (command_int >= 0) {
            writeSerial("s", "dd", n, command_int);
        }
        // if command < -1, skip the servo command
    }
    else if (mode == rover6_serial_bridge::Rover6Servos::VELOCITY_MODE) {
        writeSerial("sv", "df", n, command);
    }
}

bool Rover6SerialBridge::set_pid(rover6_serial_bridge::Rover6PidSrv::Request  &req,
         rover6_serial_bridge::Rover6PidSrv::Response &res)
{
    writeK(req.kp_A, req.ki_A, req.kd_A, req.kp_B, req.ki_B, req.kd_B, req.speed_kA, req.speed_kB);
    ROS_INFO("Setting pid: kp_A=%f, ki_A=%f, kd_A=%f, kp_B=%f, ki_B=%f, kd_B=%f, speed_kA=%f, speed_kB=%f",
        req.kp_A, req.ki_A, req.kd_A, req.kp_B, req.ki_B, req.kd_B, req.speed_kA, req.speed_kB
    );
    res.resp = true;
    return true;
}

bool Rover6SerialBridge::set_safety_thresholds(rover6_serial_bridge::Rover6SafetySrv::Request  &req,
         rover6_serial_bridge::Rover6SafetySrv::Response &res)
{
    writeObstacleThresholds(
        req.back_obstacle_threshold, req.back_ledge_threshold,
        req.front_obstacle_threshold, req.front_ledge_threshold
    );

    writeServo(_frontTilterServoNum, req.front_servo_command, rover6_serial_bridge::Rover6Servos::POSITION_MODE);
    writeServo(_backTilterServoNum, req.back_servo_command, rover6_serial_bridge::Rover6Servos::POSITION_MODE);

    ROS_INFO("Setting safety: back_lower=%d, back_upper=%d, front_lower=%d, front_upper=%d",
        req.back_obstacle_threshold, req.back_ledge_threshold, req.front_obstacle_threshold, req.front_ledge_threshold
    );
    ROS_INFO("Setting servos: front_servo_command=%d, back_servo_command=%d",
        req.front_servo_command, req.back_servo_command
    );
    res.resp = true;
    return true;
}

bool Rover6SerialBridge::send_menu_event(rover6_serial_bridge::Rover6MenuSrv::Request &req, rover6_serial_bridge::Rover6MenuSrv::Response &res)
{
    writeSerial("menu", "s", req.event.c_str());
    ROS_INFO_STREAM("Sending menu event: " << req.event);
    res.resp = true;
    return true;
}


void Rover6SerialBridge::setActive(bool state)
{
    if (state) {
        writeSerial("<>", "d", 1);
    }
    else {
        writeSerial("<>", "d", 0);
    }
}

void Rover6SerialBridge::softRestart() {
    writeSerial("<>", "d", 2);
}

void Rover6SerialBridge::setReporting(bool state)
{
    if (state) {
        writeSerial("[]", "d", 1);
    }
    else {
        writeSerial("[]", "d", 0);
    }
}

void Rover6SerialBridge::resetSensors() {
    writeSerial("[]", "d", 2);
}

// void Rover6SerialBridge::writeCurrentState()
// {
//     time_t curr_time;
// 	tm* curr_tm;
//     time(&curr_time);
// 	curr_tm = localtime(&curr_time);
//     strftime(_dateString, 50, "%I:%M:%S%p", curr_tm);
// }

void Rover6SerialBridge::writeSpeed(float speedA, float speedB) {
    writeSerial("m", "ff", speedA, speedB);
}

void Rover6SerialBridge::writeK(float kp_A, float ki_A, float kd_A, float kp_B, float ki_B, float kd_B, float speed_kA, float speed_kB) {
    // writeSerial("ks", "ffffffff", kp_A, ki_A, kd_A, kp_B, ki_B, kd_B, speed_kA, speed_kB);
    writeSerial("ks", "df", 0, kp_A);
    writeSerial("ks", "df", 1, ki_A);
    writeSerial("ks", "df", 2, kd_A);
    writeSerial("ks", "df", 3, kp_B);
    writeSerial("ks", "df", 4, ki_B);
    writeSerial("ks", "df", 5, kd_B);
    writeSerial("ks", "df", 6, speed_kA);
    writeSerial("ks", "df", 7, speed_kB);
}

void Rover6SerialBridge::writeObstacleThresholds(int back_lower, int back_upper, int front_lower, int front_upper) {
    writeSerial("safe", "dddd", front_upper, back_upper, front_lower, back_lower);
}

void Rover6SerialBridge::parseImu()
{
    double roll, pitch, yaw;
    CHECK_SEGMENT(0); imu_msg.header.stamp = getDeviceTime((uint32_t)stol(_currentBufferSegment));
    CHECK_SEGMENT(1); yaw = stof(_currentBufferSegment);
    CHECK_SEGMENT(2); pitch = stof(_currentBufferSegment);
    CHECK_SEGMENT(3); roll = stof(_currentBufferSegment);
    CHECK_SEGMENT(4); imu_msg.angular_velocity.x = stof(_currentBufferSegment);
    CHECK_SEGMENT(5); imu_msg.angular_velocity.y = stof(_currentBufferSegment);
    CHECK_SEGMENT(6); imu_msg.angular_velocity.z = stof(_currentBufferSegment);
    CHECK_SEGMENT(7); imu_msg.linear_acceleration.x = stof(_currentBufferSegment);
    CHECK_SEGMENT(8); imu_msg.linear_acceleration.y = stof(_currentBufferSegment);
    CHECK_SEGMENT(9); imu_msg.linear_acceleration.z = stof(_currentBufferSegment);
    eulerToQuat(roll, pitch, yaw);

    imu_pub.publish(imu_msg);
}

void Rover6SerialBridge::logPacketErrorCode(int error_code, unsigned long long packet_num)
{
    ROS_WARN("Packet %d returned an error!");
    switch (error_code) {
        case 1: ROS_WARN("c1 != \\x12", packet_num); break;
        case 2: ROS_WARN("c2 != \\x34", packet_num); break;
        case 3: ROS_WARN("packet is too short", packet_num); break;
        case 4: ROS_WARN("checksums don't match", packet_num); break;
        case 5: ROS_WARN("packet count segment not found", packet_num); break;
        case 6: ROS_WARN("packet counts not synchronized", packet_num); break;
        case 7: ROS_WARN("failed to find category segment", packet_num); break;
        case 8: ROS_WARN("invalid format", packet_num); break;
    }
}

void Rover6SerialBridge::eulerToQuat(double roll, double pitch, double yaw)
{
    double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	imu_msg.orientation.w = cy * cr * cp + sy * sr * sp;
	imu_msg.orientation.x = cy * sr * cp - sy * cr * sp;
	imu_msg.orientation.y = cy * cr * sp + sy * sr * cp;
	imu_msg.orientation.z = sy * cr * cp - cy * sr * sp;
}


void Rover6SerialBridge::parseEncoder()
{
    CHECK_SEGMENT(0); enc_msg.header.stamp = getDeviceTime((uint32_t)stol(_currentBufferSegment));
    CHECK_SEGMENT(1); enc_msg.left_ticks = stol(_currentBufferSegment);
    CHECK_SEGMENT(2); enc_msg.right_ticks = stol(_currentBufferSegment);
    CHECK_SEGMENT(3); enc_msg.left_speed_ticks_per_s = stol(_currentBufferSegment);
    CHECK_SEGMENT(4); enc_msg.right_speed_ticks_per_s = stol(_currentBufferSegment);

    enc_pub.publish(enc_msg);
}

void Rover6SerialBridge::parseFSR()
{
  CHECK_SEGMENT(0); fsr_msg.header.stamp = getDeviceTime((uint32_t)stol(_currentBufferSegment));
  CHECK_SEGMENT(1); fsr_msg.left = (uint16_t)stoi(_currentBufferSegment);
  CHECK_SEGMENT(2); fsr_msg.right = (uint16_t)stoi(_currentBufferSegment);

  fsr_pub.publish(fsr_msg);
}

void Rover6SerialBridge::parseSafety()
{
    CHECK_SEGMENT(0); safety_msg.header.stamp = getDeviceTime((uint32_t)stol(_currentBufferSegment));
    CHECK_SEGMENT(1); safety_msg.is_left_bumper_trig = (bool)stoi(_currentBufferSegment);
    CHECK_SEGMENT(2); safety_msg.is_right_bumper_trig = (bool)stoi(_currentBufferSegment);
    CHECK_SEGMENT(3); safety_msg.is_front_tof_trig = (bool)stoi(_currentBufferSegment);
    CHECK_SEGMENT(4); safety_msg.is_back_tof_trig = (bool)stoi(_currentBufferSegment);
    CHECK_SEGMENT(5); safety_msg.is_front_tof_ok = (bool)stoi(_currentBufferSegment);
    CHECK_SEGMENT(6); safety_msg.is_back_tof_ok = (bool)stoi(_currentBufferSegment);
    CHECK_SEGMENT(7); safety_msg.are_servos_active = (bool)stoi(_currentBufferSegment);
    CHECK_SEGMENT(8); safety_msg.are_motors_active = (bool)stoi(_currentBufferSegment);
    CHECK_SEGMENT(9); safety_msg.voltage_ok = (bool)stoi(_currentBufferSegment);
    CHECK_SEGMENT(10); safety_msg.is_active = (bool)stoi(_currentBufferSegment);
    CHECK_SEGMENT(11); safety_msg.is_reporting_enabled = (bool)stoi(_currentBufferSegment);
    CHECK_SEGMENT(12); safety_msg.is_speed_pid_enabled = (bool)stoi(_currentBufferSegment);

    safety_pub.publish(safety_msg);
}

void Rover6SerialBridge::parseINA()
{
    CHECK_SEGMENT(0); ina_msg.header.stamp = getDeviceTime((uint32_t)stol(_currentBufferSegment));
    CHECK_SEGMENT(1); ina_msg.current = stof(_currentBufferSegment);
    CHECK_SEGMENT(2); // ina_msg doesn't have a slot for power
    CHECK_SEGMENT(3); ina_msg.voltage = stof(_currentBufferSegment);

    ina_pub.publish(ina_msg);
}

void Rover6SerialBridge::parseIR()
{
    // CHECK_SEGMENT(0);  // time ms
    // CHECK_SEGMENT(1);  // remote type
    // CHECK_SEGMENT(2);  // received value
}

void Rover6SerialBridge::parseServo()
{
    servo_msg.data.clear();
    CHECK_SEGMENT(0); // time ms
    CHECK_SEGMENT(1); int i = stoi(_currentBufferSegment);
    CHECK_SEGMENT(2); servo_msg.data[i] = stoi(_currentBufferSegment);
    servo_pub.publish(servo_msg);
}

void Rover6SerialBridge::parseTOF()
{
    CHECK_SEGMENT(0); tof_msg.header.stamp = getDeviceTime((uint32_t)stol(_currentBufferSegment));
    CHECK_SEGMENT(1); tof_msg.front_mm = stoi(_currentBufferSegment);
    CHECK_SEGMENT(2); tof_msg.back_mm = stoi(_currentBufferSegment);
    CHECK_SEGMENT(3); tof_msg.front_measure_status = stoi(_currentBufferSegment);
    CHECK_SEGMENT(4); tof_msg.back_measure_status = stoi(_currentBufferSegment);
    CHECK_SEGMENT(5); tof_msg.front_status = stoi(_currentBufferSegment);
    CHECK_SEGMENT(6); tof_msg.back_status = stoi(_currentBufferSegment);

    tof_pub.publish(tof_msg);
}
