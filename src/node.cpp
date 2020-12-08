#include "node.h"
#include "commander.h"
std::string CAN_TOPIC;
std::string CAR_ENABLED_TOPIC;
std::string CAR_BRAKE_TOPIC;
std::string CAR_THROTTLE_TOPIC;
std::string CAR_STEERING_TORQUE_TOPIC;
std::string CAR_SPEED_FEEDBACK_TOPIC;
std::string CAR_ANGLE_FEEDBACK_TOPIC;

void DrivekitNode::steering_callback(const std_msgs::msg::Float64::SharedPtr msg) const
{
    double d = msg->data;
    DrivekitNode::car_state.steering_torque = d;
}
void DrivekitNode::throttle_callback(const std_msgs::msg::Float64::SharedPtr msg) const
{
    double d = msg->data;
    DrivekitNode::car_state.throttle = d;
}
void DrivekitNode::brake_callback(const std_msgs::msg::Float64::SharedPtr msg) const
{
    double d = msg->data;
    DrivekitNode::car_state.brakes = d;
}
void DrivekitNode::enabled_callback(const std_msgs::msg::Bool::SharedPtr msg) const
{
    bool d = msg->data;
    std::cout << "ROS:car/enabled::" << d<<std::endl;
    if(DrivekitNode::car_state.enabled!=d)
    {
        std::cout <<  "DRIVEKIT STATE CHANGED TO "<<d<<std::endl;
    }
    DrivekitNode::car_state.enabled = d;
}

void DrivekitNode::timer_callback()
{
    commander_update(car_state);
}
state DrivekitNode::car_state;
DrivekitNode* DrivekitNode::instance;