#include "motor_pid.h"

static float integral_front_left = 0;
static float integral_front_right = 0;
static float integral_back_left = 0;
static float integral_back_right = 0;

static float last_error_front_left = 0;
static float last_error_front_right = 0;
static float last_error_back_left = 0;
static float last_error_back_right = 0;

int16_t compute_pid_front_left(float desired_speed, float actual_speed, float dt) {
    float error = desired_speed - actual_speed;
    integral_front_left += error * dt;
    integral_front_left = constrain(integral_front_left, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float derivative = (error - last_error_front_left) / dt;
    last_error_front_left = error;
    float output = MOTOR_KP * error + MOTOR_KI * integral_front_left + MOTOR_KD * derivative;
    return (int16_t)constrain(output, -800, 800);
}

int16_t compute_pid_front_right(float desired_speed, float actual_speed, float dt) {
    float error = desired_speed - actual_speed;
    integral_front_right += error * dt;
    integral_front_right = constrain(integral_front_right, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float derivative = (error - last_error_front_right) / dt;
    last_error_front_right = error;
    float output = MOTOR_KP * error + MOTOR_KI * integral_front_right + MOTOR_KD * derivative;
    return (int16_t)constrain(output, -800, 800);
}

int16_t compute_pid_back_left(float desired_speed, float actual_speed, float dt) {
    float error = desired_speed - actual_speed;
    integral_back_left += error * dt;
    integral_back_left = constrain(integral_back_left, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float derivative = (error - last_error_back_left) / dt;
    last_error_back_left = error;
    float output = MOTOR_KP * error + MOTOR_KI * integral_back_left + MOTOR_KD * derivative;
    return (int16_t)constrain(output, -800, 800);
}

int16_t compute_pid_back_right(float desired_speed, float actual_speed, float dt) {
    float error = desired_speed - actual_speed;
    integral_back_right += error * dt;
    integral_back_right = constrain(integral_back_right, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float derivative = (error - last_error_back_right) / dt;
    last_error_back_right = error;
    float output = MOTOR_KP * error + MOTOR_KI * integral_back_right + MOTOR_KD * derivative;
    return (int16_t)constrain(output, -800, 800);
}
