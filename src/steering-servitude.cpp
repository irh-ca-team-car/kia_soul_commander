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
const double KIA_SOUL_ACCELERATION = 4.34; // m/s2
const double MAX_ACCELARATION = 1.0; // m/s2
const double MAX_DECELARATION = 1.0; // m/s2
const double KIA_SOUL_DECELERATION = 8; // m/s2
const double SPEED_KP = 1850, SPEED_KI = 8.3, SPEED_KD = 1900, SPEED_DIV = (10000 * KIA_SOUL_ACCELERATION);
const double STEERING_KP = 0.125, STEERING_KI = 0, STEERING_KD = 0, STEERING_DIV = 2;
const int N=20;

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
    double sum_error_speed = 0;
    double previous_error_speed = 0;
    double sum_error_steering = 0;
    double previous_error_steering = 0;
    double last_accelerations[N];
    double last_steerings[N];
    int index = 0, count_cmd = N;
    for (int i = 0; i < N; i++)
    {
        last_accelerations[i] = 0;
        last_steerings[i] = 0;
    }
    

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        ++count_cmd;
        if (count_cmd > N) count_cmd = N;

        double steering_error = steering_actual - steering;
        sum_error_steering += steering_error;
        double steering_delta_error = steering_error-previous_error_steering;
        previous_error_steering = steering_error;
        double steering_cmd = -(STEERING_KP*steering_error + STEERING_KI*sum_error_steering + STEERING_KD*steering_delta_error) / STEERING_DIV;
        if(steering_cmd < -1) steering_cmd= -1;
        if(steering_cmd > 1) steering_cmd = 1;
        printf("%lf, %lf, %lf \r\n",steering_actual ,steering , steering_cmd);
        last_steerings[index] = steering_cmd;

        double torque = 0;
        for (int i = 0; i < count_cmd; i++)
        {
            double cmd = last_steerings[i];
            if(steering_cmd<0 && cmd < steering_cmd)cmd=steering_cmd;
            if(steering_cmd>0 && cmd > steering_cmd)cmd=steering_cmd;
            torque += cmd;
        }
        torque /= count_cmd;

        auto msg_torque = std_msgs::Float64();
        msg_torque.data = torque;
        pub_torque.publish(msg_torque);
    
        double speed_error = speed_actual - speed;
        sum_error_speed += speed_error;
        double speed_delta_error = speed_error-previous_error_speed;
        previous_error_speed = speed_error;
        double speed_cmd = (SPEED_KP*speed_error + SPEED_KI*sum_error_speed + SPEED_KD*speed_delta_error) / SPEED_DIV;
        speed_cmd = -speed_cmd;
        if (speed_cmd < -MAX_DECELARATION / KIA_SOUL_DECELERATION) speed_cmd = -MAX_DECELARATION / KIA_SOUL_DECELERATION;
        if (speed_cmd >MAX_ACCELARATION / KIA_SOUL_ACCELERATION) speed_cmd = MAX_ACCELARATION / KIA_SOUL_ACCELERATION;
        double acceleration_cmd = (speed_cmd>0)?speed_cmd:0;
        double brake = (speed_cmd<0)?-speed_cmd:0;

         last_accelerations[index] = acceleration_cmd;

        double acceleration = 0;
        for (int i = 0; i < count_cmd; i++)
        {
          double cmd = last_accelerations[i];
            if(acceleration_cmd<0 && cmd < acceleration_cmd)cmd=acceleration_cmd;
            if(acceleration_cmd>0 && cmd > acceleration_cmd)cmd=acceleration_cmd;
            acceleration += cmd;
        }
        acceleration /= count_cmd;

        auto msg_accel = std_msgs::Float64();
        msg_accel.data = acceleration;
        pub_acceleration.publish(msg_accel);
        auto msg_brake = std_msgs::Float64();
        msg_brake.data = brake;
        pub_brake.publish(msg_brake);
        ++index;
        if (index == N) index = 0;
    }
    
    return 0;
}