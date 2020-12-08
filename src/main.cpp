#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include <fcntl.h>
#include <string>
#include <map>
#include <vector>
#include <iostream>
#include "oscc.h"
#include <unistd.h>
#include "compile.h"

#include "commander.h"
#include "can_protocols/steering_can_protocol.h"
#include "node.h"
#define SYSTEM(A, ...)                \
    {                                 \
        char m[1000];                 \
        sprintf(m, A, ##__VA_ARGS__); \
        system(m);                    \
    }

using namespace std;
int channel, bitrate = 500000;
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto n = std::make_shared<DrivekitNode>();
    auto log = n->get_logger();
    RCLCPP_INFO(log, "V%d.%d.%d: COMPILED WITH [ ROS2 DRIVEKIT ]", MAJOR, MINOR, RELEASE);

    channel = n->declare_parameter<int>("channel", 0);

    errno = 0;

    oscc_result_t ret = OSCC_OK;

    RCLCPP_INFO(log, "Configuring canbus");
    if (geteuid())
    {
        RCLCPP_WARN(log, "Could not set canbus speed, run as root or set manually and restart the node");
        RCLCPP_WARN(log, "If you already did these step, ignore the warning");
        RCLCPP_WARN(log, "\tsudo ifconfig can%d down", channel);
        RCLCPP_WARN(log, "\tsudo ip link set can%d type can bitrate %d", channel, bitrate);
        RCLCPP_WARN(log, "\tsudo ip link set up can%d", channel);
        RCLCPP_WARN(log, "\tsudo ifconfig can%d up", channel);
        RCLCPP_WARN(log, "If you already did these step, ignore the warning");
    }
    else
    {
        SYSTEM("ifconfig can%d down\n", channel);
        SYSTEM("ip link set can%d type can bitrate %d\n", channel, bitrate);
        SYSTEM("ip link set up can%d\n", channel);
        SYSTEM("ifconfig can%d up\n", channel);
    }

    ret = (oscc_result_t)commander_init(channel);
    if (ret == OSCC_OK)
    {
        RCLCPP_INFO(log, "Status : OK : WAITING FOR ROS TOPICS");
        RCLCPP_INFO(log, "\tCANBUS   TOPIC   :%s", CAN_TOPIC.c_str());
        RCLCPP_INFO(log, "\tFEEDBACK TOPICS  :%s", CAR_ANGLE_FEEDBACK_TOPIC.c_str());
        RCLCPP_INFO(log, "\t                  %s", CAR_SPEED_FEEDBACK_TOPIC.c_str());
        RCLCPP_INFO(log, "\tCOMMAND TOPICS   :%s", CAR_BRAKE_TOPIC.c_str());
        RCLCPP_INFO(log, "\t                  %s", CAR_THROTTLE_TOPIC.c_str());
        RCLCPP_INFO(log, "\t                  %s", CAR_STEERING_TORQUE_TOPIC.c_str());
        RCLCPP_INFO(log, "\t                  %s", CAR_ENABLED_TOPIC.c_str());

        
        rclcpp::spin(n);
        rclcpp::shutdown();

        commander_close(channel);
    }
    return 0;
}