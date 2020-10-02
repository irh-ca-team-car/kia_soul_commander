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

#include "commander.h"
#include "can_protocols/steering_can_protocol.h"
#include "node.h"

int channel;
void decodeParameters(int argc, char *argv[]);
int main(int argc, char *argv[])
{
    printf("V%d.%d.%d: COMPILED WITH [", MAJOR, MINOR, RELEASE);
    printf(" ROS2 ");
    printf(" DRIVEKIT ");
    printf("]\r\n");

    errno = 0;

    decodeParameters(argc, argv);
    oscc_result_t ret = OSCC_OK;

    char m[150];
    sprintf(m, "ifconfig can%d down\n", channel);
    system(m);

    sprintf(m, "ip link set can%d type can bitrate 500000\n", channel);
    system(m);

    sprintf(m, "ip link set up can%d\n", channel);
    system(m);

    sprintf(m, "ifconfig can%d up\n", channel);
    system(m);

    ret = (oscc_result_t)commander_init(channel);
    if (ret == OSCC_OK)
    {
        printf("\nStatus : OK : WAITING FOR ROS TOPICS\n");
        printf("\tCANBUS   TOPIC   :" CAN_TOPIC"\n");
        printf("\tFEEDBACK TOPICS  :" CAR_ANGLE_FEEDBACK_TOPIC"\n");
        printf("\t                  " CAR_SPEED_FEEDBACK_TOPIC"\n");
        printf("\tCOMMAND TOPICS   :" CAR_BRAKE_TOPIC "\n");
        printf("\t                  " CAR_THROTTLE_TOPIC"\n");
        printf("\t                  " CAR_STEERING_TORQUE_TOPIC"\n");
        printf("\t                  " CAR_ENABLED_TOPIC "\n");

        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<DrivekitNode>());
        rclcpp::shutdown();
           
        commander_close(channel);
    }
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
    if (idx < (int)unnamed.size())
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
    return ret;
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
    return ret;
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
    else if(mandatory)
        throw "ERROR";
    return "";
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
  
    int idx = 0;
    std::vector<std::string> empty;
    channel = getValueForParameterInt("-c", unnamed, parameters, idx, 0, false);
    channel = getValueForParameterInt("-channel", unnamed, parameters, idx, channel, false);
}
