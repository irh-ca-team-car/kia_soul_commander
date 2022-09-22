/**
 * @file commander.h
 * @brief Commander Interface.
 *
 */
#include "rclcpp/time.hpp"

#ifndef COMMANDER_H
#define COMMANDER_H

int commander_init( int channel );

void commander_close( int channel );

void commander_update(state &car_state, rclcpp::Time time);

#endif /* COMMANDER_H */
