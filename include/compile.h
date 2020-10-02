
#define MAJOR 2
#define MINOR 0
#define RELEASE 0
#ifndef COMPILE_H
#define COMPILE_H

#ifndef COMMANDER
#ifndef ROS
#define COMMANDER true
#define ROS true
#endif
#endif

struct state
{
    double throttle;
    double steering_torque;
    double brakes;
    bool enabled;
};

#endif