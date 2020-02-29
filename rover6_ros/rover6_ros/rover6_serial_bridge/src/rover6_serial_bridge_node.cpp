
#include "rover6_serial_bridge/rover6_serial_bridge.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rover6_serial_bridge");
    ros::NodeHandle nh("/");

    Rover6SerialBridge broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
