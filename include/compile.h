#define MAJOR 1
#define MINOR 1
#define RELEASE 0
#ifndef COMMANDER
#ifndef ROS
#ifndef JOYSTICK
#define COMMANDER true
#define ROS true
#define JOYSTICK true
#endif
#endif
#endif

struct state
{
    double throttle;
    double steering_torque;
    double brakes;
    bool enabled;
};
