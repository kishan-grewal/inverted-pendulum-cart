#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#define encoderA 2
#define encoderB 3
#define encoderI 4

extern volatile long pulse_count; 

void encoder_setup();
void handle_A_rising();
// void handle_B_rising();
void handle_I_pulse();

#endif