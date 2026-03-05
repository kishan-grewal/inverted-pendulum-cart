#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <Arduino.h>

#define MOTOR_KP 2000.0
#define MOTOR_KI 1000.0
#define MOTOR_KD 10.0 // 20.0
#define INTEGRAL_LIMIT 500.0

int16_t compute_pid_front_left(float desired_speed, float actual_speed, float dt);
int16_t compute_pid_front_right(float desired_speed, float actual_speed, float dt);
int16_t compute_pid_back_left(float desired_speed, float actual_speed, float dt);
int16_t compute_pid_back_right(float desired_speed, float actual_speed, float dt);

#endif
