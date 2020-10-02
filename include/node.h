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
using namespace std::chrono_literals;
using std::placeholders::_1;

class DrivekitNode:public rclcpp::Node{
public:
    DrivekitNode()
    : Node("car_drivekit")
    {
        pub_canbs = this->create_publisher<can_msgs::msg::Frame>(CAN_TOPIC, 10);
        pub_curr_speed = this->create_publisher<std_msgs::msg::Float64>(CAR_SPEED_FEEDBACK_TOPIC, 10);
        pub_curr_angle = this->create_publisher<std_msgs::msg::Float64>(CAR_ANGLE_FEEDBACK_TOPIC, 10);
        
       sub_steer = this->create_subscription<std_msgs::msg::Float64>(
      CAR_STEERING_TORQUE_TOPIC, 10, std::bind(&DrivekitNode::steering_callback, this, _1));
       sub_throt = this->create_subscription<std_msgs::msg::Float64>(
      CAR_THROTTLE_TOPIC, 10, std::bind(&DrivekitNode::throttle_callback, this, _1));
       sub_brake = this->create_subscription<std_msgs::msg::Float64>(
      CAR_BRAKE_TOPIC, 10, std::bind(&DrivekitNode::brake_callback, this, _1));
       sub_enabled = this->create_subscription<std_msgs::msg::Bool>(
      CAR_ENABLED_TOPIC, 10, std::bind(&DrivekitNode::enabled_callback, this, _1));
      instance=this;

      timer_ = this->create_wall_timer(
      1ms, std::bind(&DrivekitNode::timer_callback, this));
    }
    
    static void publishCan(can_msgs::msg::Frame msg)
    {
        instance->pub_canbs->publish(msg);
    }
    static void publishSpeed(double speed){
        auto msg_torque = std_msgs::msg::Float64();
        msg_torque.data = speed;
        instance->pub_curr_speed->publish(msg_torque);
    }
    static void publishAngle(double angle){
        auto msg_torque = std_msgs::msg::Float64();
        msg_torque.data = angle;
        instance->pub_curr_angle->publish(msg_torque);
    }
    static state car_state;
private:
    static DrivekitNode* instance;
    void steering_callback (const std_msgs::msg::Float64::SharedPtr msg) const;
    void throttle_callback (const std_msgs::msg::Float64::SharedPtr msg) const;
    void brake_callback (const std_msgs::msg::Float64::SharedPtr msg) const;
    void enabled_callback (const std_msgs::msg::Bool::SharedPtr msg) const;
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