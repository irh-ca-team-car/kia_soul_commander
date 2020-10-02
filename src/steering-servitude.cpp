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
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

const double KIA_SOUL_ACCELERATION = 4.34; // m/s2
const double MAX_ACCELARATION = 1.0; // m/s2
const double MAX_DECELARATION = 1.0; // m/s2
const double KIA_SOUL_DECELERATION = 8; // m/s2
const double SPEED_KP = 1850, SPEED_KI = 8.3, SPEED_KD = 1900, SPEED_DIV = (10000 * KIA_SOUL_ACCELERATION);
const double STEERING_KP = 0.125, STEERING_KI = 0, STEERING_KD = 0, STEERING_DIV = 2;
const int N=20;

double steering = 0, speed = 0, steering_actual = 0, speed_actual = 0;

class Serviture : public rclcpp::Node{
    public:
    Serviture()
    : Node("car_servitude")
    {
        pub_torque = this->create_publisher<std_msgs::msg::Float64>("car/steering/torque", 10);
        pub_acceleration = this->create_publisher<std_msgs::msg::Float64>("car/throttle", 10);
        pub_brake = this->create_publisher<std_msgs::msg::Float64>("car/brake", 10);
        timer_ = this->create_wall_timer(
            50ms, std::bind(&Serviture::timer_callback, this));
        for (int i = 0; i < N; i++)
        {
            last_accelerations[i] = 0;
            last_steerings[i] = 0;
        }
       subscription1_ = this->create_subscription<std_msgs::msg::Float64>(
      "car/steering/angle/desired", 10, std::bind(&Serviture::steering_callback, this, _1));
       subscription2_ = this->create_subscription<std_msgs::msg::Float64>(
      "car/speed/desired", 10, std::bind(&Serviture::speed_callback, this, _1));
       subscription3_ = this->create_subscription<std_msgs::msg::Float64>(
      "car/steering/angle/actual", 10, std::bind(&Serviture::steering_actual_callback, this, _1));
       subscription4_ = this->create_subscription<std_msgs::msg::Float64>(
      "car/speed/actual", 10, std::bind(&Serviture::speed_actual_callback, this, _1));
    }
    private:
    void steering_callback(const std_msgs::msg::Float64::SharedPtr msg) const
    {
        steering= msg->data;
    }
    void speed_callback(const std_msgs::msg::Float64::SharedPtr msg) const
    {
        speed = msg->data;
    }
    void steering_actual_callback(const std_msgs::msg::Float64::SharedPtr msg) const
    {
        steering_actual = msg->data;
    }
    void speed_actual_callback(const std_msgs::msg::Float64::SharedPtr msg) const
    {
        speed_actual = msg->data;
    }
       void timer_callback()
    {
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

        auto msg_torque = std_msgs::msg::Float64();
        msg_torque.data = torque;
        pub_torque->publish(msg_torque);
    
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

        auto msg_accel = std_msgs::msg::Float64();
        msg_accel.data = acceleration;
        pub_acceleration->publish(msg_accel);
        auto msg_brake = std_msgs::msg::Float64();
        msg_brake.data = brake;
        pub_brake->publish(msg_brake);
        ++index;
        if (index == N) index = 0;
    }
    
    double sum_error_speed = 0;
    double previous_error_speed = 0;
    double sum_error_steering = 0;
    double previous_error_steering = 0;
    double last_accelerations[N];
    double last_steerings[N];
    int index = 0, count_cmd = N;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_torque;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_acceleration;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_brake;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription1_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription2_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription3_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription4_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Serviture>());
    rclcpp::shutdown();
    return 0;
}