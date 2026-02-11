#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include <Motoron.h>

void motor_setup();
void set_motor_speed(int16_t speed);
void set_motor_speeds(int16_t front_left, int16_t front_right, int16_t back_left, int16_t back_right);

extern MotoronI2C motor_driver_front(17);
extern MotoronI2C motor_driver_back(16);

#endif
