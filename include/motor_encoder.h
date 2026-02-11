#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <Arduino.h>

#define encoder_front_left_encoder_A 2
#define encoder_front_left_encoder_B 3

#define encoder_front_right_encoder_A 5
#define encoder_front_right_encoder_B 6

#define encoder_back_left_encoder_A 8
#define encoder_back_left_encoder_B 9

#define encoder_back_right_encoder_A 11
#define encoder_back_right_encoder_B 12

extern volatile long encoder_front_left_pulse_count;
extern volatile long encoder_front_right_pulse_count;
extern volatile long encoder_back_left_pulse_count;
extern volatile long encoder_back_right_pulse_count;

void motor_encoder_setup();
void handle_encoder(int encoder_index);

/**
 * Read the current speed of a motor wheel
 * @param encoder_index: 0=front_left, 1=front_right, 2=back_left, 3=back_right
 * @return Speed in m/s
 */
float read_encoder_speed(int encoder_index);

/**
 * Get total distance traveled by a wheel
 * @param encoder_index: 0=front_left, 1=front_right, 2=back_left, 3=back_right
 * @return Distance in meters
 */
float get_wheel_distance(int encoder_index);

/**
 * Reset encoder count for a single motor
 * @param encoder_index: 0=front_left, 1=front_right, 2=back_left, 3=back_right
 */
void reset_encoder(int encoder_index);

/**
 * Reset all encoder counts
 */
void reset_all_encoders();

#endif