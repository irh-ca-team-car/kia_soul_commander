
#define MAJOR 2
#define MINOR 0
#define RELEASE 1
#ifndef COMPILE_H
#define COMPILE_H

#ifndef COMMANDER
#ifndef ROS
#define COMMANDER true
#define ROS true
#endif
#endif

#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
struct state
{
    double throttle;
    rclcpp::Time throttle_time;
    double steering_torque;
    rclcpp::Time steering_torque_time;
    double brakes;
    rclcpp::Time brakes_time;
    bool enabled;
    rclcpp::Duration max_duration = rclcpp::Duration(1,500);
};

#endif