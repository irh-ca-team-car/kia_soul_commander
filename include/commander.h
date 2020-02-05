/**
 * @file commander.h
 * @brief Commander Interface.
 *
 */


#ifndef COMMANDER_H
#define COMMANDER_H

int commander_init( int channel );

void commander_close( int channel );

void commander_update(state &car_state);

#endif /* COMMANDER_H */
