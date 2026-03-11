#ifndef PENDULUM_ENCODER_H
#define PENDULUM_ENCODER_H

#include <Arduino.h>

#define encoderA 9 
#define encoderB 11
#define encoderI 8

extern volatile long pendulum_encoder_pulse_count; 

void pendulum_encoder_setup();
void handle_A_rising();
void handle_B_rising();
void handle_I_pulse();

#endif