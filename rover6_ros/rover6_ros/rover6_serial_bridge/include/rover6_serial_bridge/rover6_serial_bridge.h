#ifndef _ROVER6_SERIAL_BRIDGE_H_

#include <exception>
#include <iostream>
#include <ctime>

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/BatteryState.h"
#include "serial/serial.h"

#include "rover6_serial_bridge/Rover6Encoder.h"
#include "rover6_serial_bridge/Rover6FSR.h"
#include "rover6_serial_bridge/Rover6Safety.h"
#include "rover6_serial_bridge/Rover6TOF.h"
#include "rover6_serial_bridge/Rover6Motors.h"
#include "rover6_serial_bridge/Rover6Servos.h"

#include "rover6_serial_bridge/Rover6PidSrv.h"
#include "rover6_serial_bridge/Rover6SafetySrv.h"


using namespace std;

#define CHECK_SEGMENT(n)  if (!getNextSegment()) {  ROS_ERROR_STREAM("Failed to parse segment #" << n << ". Buffer: " << _serialBuffer);  return;  }

char PACKET_START_0 = '\x12';
char PACKET_START_1 = '\x34';
char PACKET_STOP = '\n';

struct StructReadyState {
    uint32_t time_ms;
    string rover_name;
    bool is_ready;
};

class ReadyTimeoutExceptionClass : public exception {
    virtual const char* what() const throw() { return "Timeout reached. Never got ready signal from serial device"; }
} ReadyTimeoutException;

class Rover6SerialBridge {
private:
    ros::NodeHandle nh;  // ROS node handle

    string _serialPort;
    int _serialBaud;
    string _serialBuffer;
    int _serialBufferIndex;
    string _currentBufferSegment;
    serial::Serial _serialRef;
    unsigned long long _readPacketNum;
    unsigned long long _writePacketNum;
    char* _dateString;
    ros::Time deviceStartTime;
    uint32_t offsetTimeMs;
    string _roverNamespace;

    string _imuFrameID;
    ros::Publisher imu_pub;
    sensor_msgs::Imu imu_msg;

    string _encFrameID;
    double _wheelRadiusCm, _ticksPerRotation, _maxRPM, _cmPerTick, _cpsToCmd;
    ros::Publisher enc_pub;
    rover6_serial_bridge::Rover6Encoder enc_msg;

    ros::Publisher fsr_pub;
    rover6_serial_bridge::Rover6FSR fsr_msg;

    ros::Publisher safety_pub;
    rover6_serial_bridge::Rover6Safety safety_msg;

    ros::Publisher ina_pub;
    sensor_msgs::BatteryState ina_msg;

    unsigned int _numServos;
    string _servosTopicName;
    ros::Publisher servo_pub;
    std_msgs::Int16MultiArray servo_msg;

    ros::Publisher tof_pub;
    rover6_serial_bridge::Rover6TOF tof_msg;

    string _motorsTopicName;
    ros::Subscriber motors_sub;
    rover6_serial_bridge::Rover6Motors motors_msg;
    void motorsCallback(const rover6_serial_bridge::Rover6Motors::ConstPtr& msg);

    ros::Subscriber servos_sub;
    rover6_serial_bridge::Rover6Servos servos_msg;
    void servosCallback(const rover6_serial_bridge::Rover6Servos::ConstPtr& msg);
    void writeServo(unsigned int n, int command);

    ros::ServiceServer pid_service;
    ros::ServiceServer safety_service;

    StructReadyState* readyState;

    int _frontTilterServoNum;
    int _backTilterServoNum;
    int _panServoNum;
    int _tiltServoNum;

    void configure();
    void checkReady();
    void setStartTime(uint32_t time_ms);
    ros::Time getDeviceTime(uint32_t time_ms);
    bool getNextSegment();
    void waitForPacketStart();
    void processSerialPacket(string category);

    bool readSerial();
    void writeSerial(string name, const char *formats, ...);

    void setup();
    void loop();
    void stop();

    bool set_pid(rover6_serial_bridge::Rover6PidSrv::Request  &req, rover6_serial_bridge::Rover6PidSrv::Response &res);
    bool set_safety_thresholds(rover6_serial_bridge::Rover6SafetySrv::Request  &req, rover6_serial_bridge::Rover6SafetySrv::Response &res);

    void setActive(bool state);
    void softRestart();
    void setReporting(bool state);
    void resetSensors();
    // void writeCurrentState();
    void writeSpeed(float speedA, float speedB);
    void writeK(float kp_A, float ki_A, float kd_A, float kp_B, float ki_B, float kd_B, float speed_kA, float speed_kB);
    void writeObstacleThresholds(int back_lower, int back_upper, int front_lower, int front_upper);
    void logPacketErrorCode(int error_code, unsigned long long packet_num);

    void parseImu();
    void eulerToQuat(double roll, double pitch, double yaw);

    void parseEncoder();
    double convertTicksToCm(long ticks);

    void parseFSR();
    void parseSafety();
    void parseINA();
    void parseIR();
    void parseServo();
    void parseTOF();
public:
    Rover6SerialBridge(ros::NodeHandle* nodehandle);
    int run();
};

#endif  // _ROVER6_SERIAL_BRIDGE_H_
