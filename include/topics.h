#pragma once
extern std::string CAN_TOPIC;
extern std::string CAR_ENABLED_TOPIC;
extern std::string CAR_BRAKE_TOPIC;
extern std::string CAR_THROTTLE_TOPIC;
extern std::string CAR_STEERING_TORQUE_TOPIC;
extern std::string CAR_SPEED_FEEDBACK_TOPIC;
extern std::string CAR_ANGLE_FEEDBACK_TOPIC;

#define CAN_TOPIC_DEFAULT "/can0"
#define CAR_ENABLED_TOPIC_DEFAULT "/enabled"
#define CAR_BRAKE_TOPIC_DEFAULT "/brake"
#define CAR_THROTTLE_TOPIC_DEFAULT "/throttle"
#define CAR_STEERING_TORQUE_TOPIC_DEFAULT "/steering/torque"
#define CAR_SPEED_FEEDBACK_TOPIC_DEFAULT "/speed/actual"
#define CAR_ANGLE_FEEDBACK_TOPIC_DEFAULT "/steering/angle/actual"
