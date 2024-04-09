#ifndef NODE_H
#define NODE_H
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "can_msgs/msg/frame.hpp"
#include "compile.h"
#include "topics.h"
#include "commander.h"
#define SYSTEM(A, ...)                \
    {                                 \
        char m[1000];                 \
        sprintf(m, A, ##__VA_ARGS__); \
        system(m);                    \
    }
using namespace std::chrono_literals;
using std::placeholders::_1;

class DrivekitNode : public rclcpp::Node
{
public:
    DrivekitNode();
    static void info(std::string msg)
    {
        std::cout << "[INFO]" << msg;
        //RCLCPP_INFO(instance->get_logger(), msg);
    }
    static void warn(std::string msg)
    {
        std::cerr << "[WARN]" << msg;
        //RCLCPP_WARN(instance->get_logger(), msg);
    }
    static void error(std::string msg)
    {
        std::cerr << "[ERROR]" << msg;
        //RCLCPP_ERROR(instance->get_logger(), msg);
    }
    static void publishCan(can_msgs::msg::Frame msg)
    {
        instance->pub_canbs->publish(msg);
    }
    static void publishSpeed(double speed)
    {
        auto msg_torque = std_msgs::msg::Float64();
        msg_torque.data = speed;
        instance->pub_curr_speed->publish(msg_torque);
    }
    static void publishAngle(double angle)
    {
        auto msg_torque = std_msgs::msg::Float64();
        msg_torque.data = angle;
        instance->pub_curr_angle->publish(msg_torque);
    }
    static state car_state;
    int channel, bitrate;
    bool run=true;
    void shutdown();
private:
    static DrivekitNode *instance;
    void steering_callback(const std_msgs::msg::Float64::SharedPtr msg) ;
    void throttle_callback(const std_msgs::msg::Float64::SharedPtr msg) ;
    void brake_callback(const std_msgs::msg::Float64::SharedPtr msg) ;
    void enabled_callback(const std_msgs::msg::Bool::SharedPtr msg) const;
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_canbs;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_curr_speed;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_curr_angle;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_steer;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_throt;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_brake;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enabled;
};

#endif