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
const double KP = 1850, KI = 8.3, KD = 1900, DIV = 10000;

double steering = 0, speed = 0, steering_actual = 0, speed_actual = 0;
ros::Publisher pub_torque;
ros::Publisher pub_acceleration;
ros::Publisher pub_brake;

void steering_callback(const std_msgs::Float64::ConstPtr &msg)
{
    steering= msg->data;
    //printf("%s",__func__);
    
}
void speed_callback(const std_msgs::Float64::ConstPtr &msg)
{
    speed = msg->data;
    //printf("%s",__func__);
}
void steering_actual_callback(const std_msgs::Float64::ConstPtr &msg)
{
   steering_actual = msg->data;
   //printf("%s",__func__);
}
void speed_actual_callback(const std_msgs::Float64::ConstPtr &msg)
{
    speed_actual = msg->data;
    //printf("%s",__func__);
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
    ros::Subscriber sub_steer = n.subscribe("/car/steering/angle/desired", 1, steering_callback);
    ros::Subscriber sub_speed = n.subscribe("/car/speed/desired", 1, speed_callback);
    ros::Subscriber sub_steer2 = n.subscribe("/car/steering/angle/actual", 1, steering_actual_callback);
    ros::Subscriber sub_speed2 = n.subscribe("/car/speed/actual", 1, speed_actual_callback);
    pub_torque = n.advertise<std_msgs::Float64>("/car/steering/torque", 1);
    pub_acceleration = n.advertise<std_msgs::Float64>("/car/throttle", 1);
    pub_brake = n.advertise<std_msgs::Float64>("/car/brake", 1);
    ros::Rate rate(2);
    double sum_error = 0;
    double previous_error = 0;

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        //replace with servitude
        double torque = steering;

        auto msg_torque = std_msgs::Float64();
        msg_torque.data = torque;
        pub_torque.publish(msg_torque);
    
        double error = speed_actual - speed;
        sum_error += error;
        double delta_error = error-previous_error;
        previous_error = error;
        double cmd = (KP*error + KI*sum_error + KD*delta_error) / DIV;
        if (cmd > 1) cmd = 1;
        if (cmd <-1) cmd = -1;
        double acceleration = (cmd<0)?-cmd:0;
        double brake = (cmd>0)?cmd:0;
        printf("%lf, %lf, %lf, %lf \r\n",speed_actual ,speed , acceleration, brake);

        auto msg_accel = std_msgs::Float64();
        msg_accel.data = acceleration;
        pub_acceleration.publish(msg_accel);
        auto msg_brake = std_msgs::Float64();
        msg_brake.data = brake;
        pub_brake.publish(msg_brake);
    }
    
    return 0;
}