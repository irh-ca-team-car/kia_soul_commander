#include "node.h"
#include "oscc.h"
#include "commander.h"
#include <unistd.h>
std::string CAN_TOPIC;
std::string CAR_ENABLED_TOPIC;
std::string CAR_BRAKE_TOPIC;
std::string CAR_THROTTLE_TOPIC;
std::string CAR_STEERING_TORQUE_TOPIC;
std::string CAR_SPEED_FEEDBACK_TOPIC;
std::string CAR_ANGLE_FEEDBACK_TOPIC;
DrivekitNode::DrivekitNode() : Node("car_drivekit")
{
    RCLCPP_INFO(get_logger(), "V%d.%d.%d: COMPILED WITH [ ROS2 DRIVEKIT ]", MAJOR, MINOR, RELEASE);
    instance = this;
    
    auto prefix = declare_parameter<std::string>("prefix", "car");
    channel = declare_parameter<int>("channel", 0);
    bitrate = declare_parameter<int>("bitrate", 500000);

    RCLCPP_INFO(get_logger(), "--------------------");
    RCLCPP_INFO(get_logger(), "Parameters");
    RCLCPP_INFO(get_logger(), "--------------------");
    RCLCPP_INFO(get_logger(), "\tprefix=" + prefix);
    RCLCPP_INFO(get_logger(), "\tchannel=can%d", channel);
    RCLCPP_INFO(get_logger(), "\tbitrate=%d", bitrate);
    RCLCPP_INFO(get_logger(), "--------------------");

    CAN_TOPIC = prefix + CAN_TOPIC_DEFAULT;
    CAR_ANGLE_FEEDBACK_TOPIC = prefix + CAR_ANGLE_FEEDBACK_TOPIC_DEFAULT;
    CAR_SPEED_FEEDBACK_TOPIC = prefix + CAR_SPEED_FEEDBACK_TOPIC_DEFAULT;
    CAR_BRAKE_TOPIC = prefix + CAR_BRAKE_TOPIC_DEFAULT;
    CAR_THROTTLE_TOPIC = prefix + CAR_THROTTLE_TOPIC_DEFAULT;
    CAR_STEERING_TORQUE_TOPIC = prefix + CAR_STEERING_TORQUE_TOPIC_DEFAULT;
    CAR_ENABLED_TOPIC = prefix + CAR_ENABLED_TOPIC_DEFAULT;
    RCLCPP_INFO(get_logger(), "Topics");
    RCLCPP_INFO(get_logger(), "--------------------");
    RCLCPP_INFO(get_logger(), "\tCANBUS   :%s", CAN_TOPIC.c_str());
    RCLCPP_INFO(get_logger(), "\tFEEDBACK :%s", CAR_ANGLE_FEEDBACK_TOPIC.c_str());
    RCLCPP_INFO(get_logger(), "\t          %s", CAR_SPEED_FEEDBACK_TOPIC.c_str());
    RCLCPP_INFO(get_logger(), "\tCOMMAND  :%s", CAR_BRAKE_TOPIC.c_str());
    RCLCPP_INFO(get_logger(), "\t          %s", CAR_THROTTLE_TOPIC.c_str());
    RCLCPP_INFO(get_logger(), "\t          %s", CAR_STEERING_TORQUE_TOPIC.c_str());
    RCLCPP_INFO(get_logger(), "\t          %s", CAR_ENABLED_TOPIC.c_str());
    RCLCPP_INFO(get_logger(), "--------------------");

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
    

    timer_ = this->create_wall_timer(
        1ms, std::bind(&DrivekitNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Configuring canbus");
    if (geteuid())
    {
        RCLCPP_WARN(get_logger(), "Could not set canbus speed, run as root or set manually and restart the node");
        RCLCPP_WARN(get_logger(), "If you already did these step, ignore the warning");
        RCLCPP_WARN(get_logger(), "\tsudo ifconfig can%d down", channel);
        RCLCPP_WARN(get_logger(), "\tsudo ip link set can%d type can bitrate %d", channel, bitrate);
        RCLCPP_WARN(get_logger(), "\tsudo ip link set up can%d", channel);
        RCLCPP_WARN(get_logger(), "\tsudo ifconfig can%d up", channel);
        RCLCPP_WARN(get_logger(), "If you already did these step, ignore the warning");
    }
    else
    {
        SYSTEM("ifconfig can%d down\n", channel);
        SYSTEM("ip link set can%d type can bitrate %d\n", channel, bitrate);
        SYSTEM("ip link set up can%d\n", channel);
        SYSTEM("ifconfig can%d up\n", channel);
    }
    auto ret = (oscc_result_t)commander_init(channel);
    if (ret != OSCC_OK)
    {
        RCLCPP_ERROR(get_logger(), "CANNOT CONTACT DRIVEKIT");
        RCLCPP_INFO(get_logger(), "Status : Shutting down ");
        run=false;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Status : Running ");
    }
}
void DrivekitNode::shutdown()
{
    RCLCPP_INFO(get_logger(), "Status : Shutting down ");
    commander_close(channel);
}
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
    std::cout << "ROS:car/enabled::" << d << std::endl;
    if (DrivekitNode::car_state.enabled != d)
    {
        std::cout << "DRIVEKIT STATE CHANGED TO " << d << std::endl;
    }
    DrivekitNode::car_state.enabled = d;
}

void DrivekitNode::timer_callback()
{
    commander_update(car_state);
}
state DrivekitNode::car_state;
DrivekitNode *DrivekitNode::instance;