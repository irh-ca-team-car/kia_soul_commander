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

#ifdef COMMANDER
#include "commander.h"
#include "can_protocols/steering_can_protocol.h"
#endif
#ifdef ROS
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "can_msgs.h"
#endif
#ifdef JOYSTICK
#include "joystick.h"
#endif
#define STEERING_RANGE_PERCENTAGE (0.36)
void smooth(double *v, int e);

#define COMMANDER_UPDATE_INTERVAL_MICRO (5000)
#define SLEEP_TICK_INTERVAL_MICRO (1000)
state car_state;

static int error_thrown = OSCC_OK;

static unsigned long long get_timestamp_micro()
{
    struct timeval time;

    gettimeofday(&time, NULL);

    return (time.tv_usec);
}

static unsigned long long get_elapsed_time(unsigned long long timestamp)
{
    unsigned long long now = get_timestamp_micro();
    unsigned long long elapsed_time = now - timestamp;

    return elapsed_time;
}

void signal_handler(int signal_number)
{
    if (signal_number == SIGINT)
    {
        error_thrown = OSCC_ERROR;
    }
}

#if ROS
void steering_callback(const std_msgs::Float64::ConstPtr &msg)
{
    double d = msg->data;
    car_state.steering_torque = d;
}
void throttle_callback(const std_msgs::Float64::ConstPtr &msg)
{
    double d = msg->data;
    car_state.throttle = d;
}
void brake_callback(const std_msgs::Float64::ConstPtr &msg)
{
    double d = msg->data;
    car_state.brakes = d;
}
void enabled_callback(const std_msgs::Bool::ConstPtr &msg)
{
    bool d = msg->data;
    car_state.enabled = d;
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
ros::Publisher *p_steer;
ros::Publisher *p_throt;
ros::Publisher *p_brake;
ros::Publisher *p_enabl;
ros::Publisher *p_canbs;
ros::Publisher *p_curr_speed;
ros::Publisher *p_curr_angle;
std::string brake_topic;
std::string throt_topic;
std::string steer_topic;
std::string enabl_topic;
std::string can_topic;
std::string current_steer_angle_topic;
std::string current_speed_topic;
void ros_send(state car_state)
{
    auto msgs = std_msgs::Float64();
    msgs.data = car_state.steering_torque;
    p_steer->publish(msgs);
    auto msgst = std_msgs::Float64();
    msgst.data = car_state.throttle;
    p_throt->publish(msgst);
    auto msgsb = std_msgs::Float64();
    msgsb.data = car_state.brakes;
    p_brake->publish(msgsb);
    auto bmsg = std_msgs::Bool();
    bmsg.data = car_state.enabled;
    p_enabl->publish(bmsg);
}
#endif
#if COMMANDER
int channel;
#endif
void decodeParameters(int argc, char *argv[]);
int main(int argc, char *argv[])
{
    printf("V%d.%d.%d: COMPILED WITH [", MAJOR, MINOR, RELEASE);
#if ROS
    printf(" ROS ");
#endif
#if JOYSTICK
    printf(" JOYSTICK ");
#endif
#if COMMANDER
    printf(" COMMANDER ");
#endif
    printf("]\r\n");

    errno = 0;

    decodeParameters(argc, argv);
    oscc_result_t ret = OSCC_OK;

#if COMMANDER
    char m[150];
    const char *str = "ip link set can%d type can bitrate 500000\n";
    sprintf(m, str, channel);
    system(m);

    sprintf(m, "ip link set up can%d\n", channel);
    system(m);
    unsigned long long update_timestamp = get_timestamp_micro();
    unsigned long long elapsed_time = 0;
#endif
    struct sigaction sig;
    sig.sa_handler = signal_handler;
//
    sigaction(SIGINT, &sig, NULL);
#if ROS
    char hostnamePtr[80];
    char *ptr = hostnamePtr;
    gethostname(hostnamePtr, 79);
    while (*ptr != 0)
    {
        if (!rosAllowedChar(*ptr))
            *ptr = '_';
        ptr++;
    }
#if COMMANDER
    ros::init(argc, argv, "car_drivekit");
    ros::NodeHandle n;
    ros::Subscriber sub_steer = n.subscribe("car/steering/torque", 1, steering_callback);
    ros::Subscriber sub_throt = n.subscribe("car/throttle", 1, throttle_callback);
    ros::Subscriber sub_brake = n.subscribe("car/brake", 1, brake_callback);
    ros::Subscriber sub_enabled = n.subscribe("car/enabled", 1, enabled_callback);
    ros::Publisher pub_canbs = n.advertise<can_msgs::Frame>("car/can0",1);
    ros::Publisher pub_curr_speed= n.advertise<std_msgs::Float64>("car/speed/actual",1);
    ros::Publisher pub_curr_angle= n.advertise<std_msgs::Float64>("car/steering/angle/actual",1);
    p_canbs = &pub_canbs;
    p_curr_speed =&pub_curr_speed;
    p_curr_angle=&pub_curr_angle;
#endif
#if JOYSTICK
    ros::init(argc, argv, "joystick_ros");
    ros::NodeHandle n;
    ros::Publisher pub_steep = n.advertise<std_msgs::Float64>(steer_topic, 1);
    ros::Publisher pub_throt = n.advertise<std_msgs::Float64>(throt_topic, 1);
    ros::Publisher pub_brake = n.advertise<std_msgs::Float64>(brake_topic, 1);
    ros::Publisher pub_enabl = n.advertise<std_msgs::Bool>(enabl_topic, 1);
    
    p_steer = &pub_steep;
    p_brake = &pub_brake;
    p_enabl = &pub_enabl;
    p_throt = &pub_throt;

#endif
#endif

#if COMMANDER
    ret = (oscc_result_t)commander_init(channel);
    if (ret == OSCC_OK)
    {
#endif

#if JOYSTICK
        printf("\nControl Ready:\n");
        printf("    START - Enable controls\n");
        printf("    BACK - Disable controls\n");
        printf("    LEFT TRIGGER - Brake\n");
        printf("    RIGHT TRIGGER - Throttle\n");
        printf("    LEFT STICK - Steering\n");
        joystick_init();
#endif

        while (ret == OSCC_OK && error_thrown == OSCC_OK
#if ROS
               && ros::ok()
#endif
        )
        {
#if COMMANDER
#if ROS
            ros::spinOnce();
#endif
            elapsed_time = get_elapsed_time(update_timestamp);

            if (elapsed_time > COMMANDER_UPDATE_INTERVAL_MICRO)
            {
                update_timestamp =- get_timestamp_micro();
#endif

#if JOYSTICK
                ret = (oscc_result_t)check_for_controller_update(car_state);
#endif
#if ROS && JOYSTICK
                ros_send(car_state);
#endif
#if COMMANDER
                commander_update(car_state);
            }
#endif
            // Delay 1 ms to avoid loading the CPU
            (void)usleep(SLEEP_TICK_INTERVAL_MICRO);
        }
#if COMMANDER
        commander_close(channel);
    }
#endif
#if JOYSTICK
    joystick_close();
#endif
    return 0;
}
bool isInteger(std::string line)
{
    if (line.empty())
        return false;
    char *p;
    strtol(line.c_str(), &p, 10);
    return *p == 0;
}
bool isDouble(std::string line)
{
    if (line.empty())
        return false;
    char *p;
    strtod(line.c_str(), &p);
    return *p == 0;
}
std::string getValueForParameter(std::string name, std::vector<std::string> unnamed, std::map<std::string, std::string> named, int &idx)
{
    if (!named[name].empty())
        return named.at(name);
    if (idx < unnamed.size())
    {
        auto ret = unnamed[idx];
        idx++;
        return ret;
    }
    return "\b";
}
bool isParameterPresent(std::string name, std::map<std::string, std::string> named)
{
    return (!named[name].empty());
}
int asInt(std::string name, std::string val, int def, bool mandatory)
{
    int ret;
    if (!isInteger(val.c_str()))
    {
        if (mandatory)
        {
            std::cout << "argument " + name + " is missing or invalid";
            exit(1);
        }
        else
        {
            ret = def;
        }
    }
    else
    {
        ret = atoi(val.c_str());
    }
}
double asDouble(std::string name, std::string val, double def, bool mandatory)
{
    double ret;
    if (!isDouble(val.c_str()))
    {
        if (mandatory)
        {
            std::cout << "argument " + name + " is missing or invalid";
            exit(1);
        }
        else
        {
            ret = def;
        }
    }
    else
    {
        ret = atof(val.c_str());
    }
}
void printHelp(std::string str)
{
    std::cout << "usage : "
#ifdef COMMANDER
              << "sudo "
#endif
              << str
#ifdef COMMANDER
              << " channel "
#endif
#ifdef ROS
              << " [brake-topic=/car/brake] [throttle-topic=/car/throttle] [steering-topic=/car/steering/torque] [enabled-topic=/car/enabled]"
#endif
#if ROS && COMMANDER
              << " [can-topic=/car/can0]"
#endif
              << std::endl
#ifdef JOYSTICK
              << "Possible smoothing types:" << std::endl
              << "    0\t:y=1-(sqrt(a-x^2)) * sign(x)" << std::endl
              << "    1\t:y=x" << std::endl
              << "    2\t:y=x^2 *sign(x)" << std::endl
              << "    3\t:y=x^3 " << std::endl
              << "  e>3\t:y=x^e (*sign(x) if e is even) " << std::endl
#endif
              << std::endl;
    exit(0);
}
int getValueForParameterInt(std::string name, std::vector<std::string> unnamed, std::map<std::string, std::string> named, int &idx, int def, bool mandatory)
{
    return asInt(name, getValueForParameter(name, unnamed, named, idx), def, mandatory);
}
double getValueForParameterDouble(std::string name, std::vector<std::string> unnamed, std::map<std::string, std::string> named, int &idx, double def, bool mandatory)
{
    return asDouble(name, getValueForParameter(name, unnamed, named, idx), def, mandatory);
}
std::string getValueForParameterString(std::string name, std::vector<std::string> unnamed, std::map<std::string, std::string> named, int &idx, std::string def, bool mandatory)
{
    std::string tmp = getValueForParameter(name, unnamed, named, idx);
    if (tmp.empty())
        return def;
}
void decodeParameters(int argc, char *argv[])
{
    //check parameters validity
    bool foundNamed = false, previousIsName = false;
    std::string previous;
    std::map<std::string, std::string> parameters;
    std::vector<std::string> unnamed;
    for (auto i = 1; i < argc; i++)
    {
        auto isNumber = isDouble(argv[i]);
        auto isNamed = !isNumber && argv[i][0] == '-';
        if (isNamed)
        {
            if (previousIsName)
                parameters[previous] = "<exists>";
            foundNamed = true;
            previousIsName = true;
            previous = argv[i];
            if (previous == "-help")
                printHelp(argv[0]);
            if (i == argc - 1)
                parameters[previous] = "<exists>";
        }
        else //not named
        {
            if (foundNamed && previousIsName)
            {
                parameters[previous] = argv[i];
            }
            else
            {
                unnamed.push_back(argv[i]);
            }

            previousIsName = false;
        }
    }
    for (auto it = parameters.begin(), end = parameters.end(); it != end; it++)
    {
        std::cout << "GOT PARAMETER " << it->first << " = " << it->second << std::endl;
    }

    for (auto it = unnamed.begin(), end = unnamed.end(); it != end; it++)
    {
        std::cout << "GOT UNNAMED PARAMETER " << *it << std::endl;
    }

    int idx = 0;
    std::vector<std::string> empty;
    int i_empty = 0;
#if COMMANDER
    channel = getValueForParameterInt("-c", unnamed, parameters, idx, 0, false);
    channel = getValueForParameterInt("-channel", unnamed, parameters, idx, channel, false);
#endif
#if JOYSTICK && ROS
    brake_topic = getValueForParameterString("-brake-topic", empty, parameters, i_empty, "/car/brake", false);
    throt_topic = getValueForParameterString("-throttle-topic", empty, parameters, i_empty, "/car/speed/desired", false);
    steer_topic = getValueForParameterString("-steering-topic", empty, parameters, i_empty, "/car/steering/angle/desired", false);
    enabl_topic = getValueForParameterString("-enabled-topic", empty, parameters, i_empty, "/car/enabled", false);
    can_topic = getValueForParameterString("-can-topic", empty, parameters, i_empty, "/car/can0", false);
#endif
#if COMMANDER && ROS
    current_steer_angle_topic = getValueForParameterString("-current-steer-angle-topic", empty, parameters, i_empty, "/car/steering/angle/actual", false);
    current_speed_topic = getValueForParameterString("-current-speed-topic", empty, parameters, i_empty, "/car/speed/actual", false);
#endif
}
