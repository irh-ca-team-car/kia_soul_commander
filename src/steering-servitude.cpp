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
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "can_msgs.h"

double steering = 0, speed = 0, steering_actual = 0, speed_actual = 0;
ros::Publisher pub_torque;
ros::Publisher pub_acceleration;
ros::Publisher pub_brake;

void steering_callback(const std_msgs::Float64::ConstPtr &msg)
{
    steering= msg->data;
    
}
void speed_callback(const std_msgs::Float64::ConstPtr &msg)
{
    speed = msg->data;
}
void steering_actual_callback(const std_msgs::Float64::ConstPtr &msg)
{
   steering_actual = msg->data;
}
void speed_actual_callback(const std_msgs::Float64::ConstPtr &msg)
{
    speed_actual = msg->data;
}
bool rosAllowedChar(char c)
{
    if (c >= 'a' && c <= 'z')
        return true;
    if (c >= 'A' && c <= 'Z')
        return true;
    if (c >= '0' && c <= '9')
        return true;
    return false;
}

int main(int argc, char *argv[])
{

    char hostnamePtr[80];
    char *ptr = hostnamePtr;
    gethostname(hostnamePtr, 79);
    while (*ptr != 0)
    {
        if (!rosAllowedChar(*ptr))
            *ptr = '_';
        ptr++;
    }
    ros::init(argc, argv, std::string(hostnamePtr) + "_servitude");
    ros::NodeHandle n;
    ros::Subscriber sub_steer = n.subscribe("car/steering/angle", 1, steering_callback);
    ros::Subscriber sub_speed = n.subscribe("car/speed", 1, speed_callback);
    ros::Subscriber sub_steer2 = n.subscribe("car/steering/angle/actual", 1, steering_actual_callback);
    ros::Subscriber sub_speed2 = n.subscribe("car/speed/actual", 1, speed_actual_callback);
    pub_torque = n.advertise<std_msgs::Float64>("car/steering/torque", 1);
    pub_acceleration = n.advertise<std_msgs::Float64>("car/throttle", 1);
    pub_brake = n.advertise<std_msgs::Float64>("car/brake", 1);
    ros::Rate rate(30);

    while(ros::ok())
    {
        rate.sleep();
        //replace with servitude
        double torque = steering;

        auto msg_torque = std_msgs::Float64();
        msg_torque.data = torque;
        pub_torque.publish(msg_torque);
        //replace with servitude
        double acceleration = speed;
        double brake = 0;

        auto msg_accel = std_msgs::Float64();
        msg_accel.data = acceleration;
        pub_acceleration.publish(msg_accel);
        auto msg_brake = std_msgs::Float64();
        msg_brake.data = brake;
        pub_torque.publish(msg_brake);
    }
    
    return 0;
}