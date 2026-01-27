#include "motor.h"
#include "config.h"
#include "hardware.h"

static float velocity_setpoint = 0.0;
static float motor1_integral = 0.0;
static float motor2_integral = 0.0;
static float motor3_integral = 0.0;
static float motor4_integral = 0.0;

static int32_t motor1_prev_count = 0;
static int32_t motor2_prev_count = 0;
static int32_t motor3_prev_count = 0;
static int32_t motor4_prev_count = 0;

void motor_init() {
    velocity_setpoint = 0.0;
    motor1_integral = 0.0;
    motor2_integral = 0.0;
    motor3_integral = 0.0;
    motor4_integral = 0.0;
    motor1_prev_count = 0;
    motor2_prev_count = 0;
    motor3_prev_count = 0;
    motor4_prev_count = 0;
}

void motor_reset() {
    motor1_integral = 0.0;
    motor2_integral = 0.0;
    motor3_integral = 0.0;
    motor4_integral = 0.0;
}

void motor_set_velocity_command(float velocity_cmd) {
    velocity_setpoint = velocity_cmd;
}

static float compute_motor_velocity(int32_t current_count, int32_t prev_count, float dt) {
    int32_t delta = current_count - prev_count;
    float revolutions = (float)delta / (float)ENCODER_CPR_MOTOR;
    float distance = revolutions * 2.0 * 3.14159265 * WHEEL_RADIUS;
    float velocity = distance / dt;
    return velocity;
}

static float motor_pid_compute(float error, float *integral) {
    float P = MOTOR_PID_KP * error;
    
    *integral += error;
    if (*integral > MOTOR_PID_INTEGRAL_LIMIT) *integral = MOTOR_PID_INTEGRAL_LIMIT;
    if (*integral < -MOTOR_PID_INTEGRAL_LIMIT) *integral = -MOTOR_PID_INTEGRAL_LIMIT;
    float I = MOTOR_PID_KI * (*integral);
    
    float D = 0.0;
    
    float output = P + I + D;
    return output;
}

void motor_update(float dt) {
    int32_t motor1_count = hardware_read_encoder_motor1();
    int32_t motor2_count = hardware_read_encoder_motor2();
    int32_t motor3_count = hardware_read_encoder_motor3();
    int32_t motor4_count = hardware_read_encoder_motor4();
    
    float motor1_velocity = compute_motor_velocity(motor1_count, motor1_prev_count, dt);
    float motor2_velocity = compute_motor_velocity(motor2_count, motor2_prev_count, dt);
    float motor3_velocity = compute_motor_velocity(motor3_count, motor3_prev_count, dt);
    float motor4_velocity = compute_motor_velocity(motor4_count, motor4_prev_count, dt);
    
    motor1_prev_count = motor1_count;
    motor2_prev_count = motor2_count;
    motor3_prev_count = motor3_count;
    motor4_prev_count = motor4_count;
    
    float motor1_error = velocity_setpoint - motor1_velocity;
    float motor2_error = velocity_setpoint - motor2_velocity;
    float motor3_error = velocity_setpoint - motor3_velocity;
    float motor4_error = velocity_setpoint - motor4_velocity;
    
    float motor1_pwm = motor_pid_compute(motor1_error, &motor1_integral);
    float motor2_pwm = motor_pid_compute(motor2_error, &motor2_integral);
    float motor3_pwm = motor_pid_compute(motor3_error, &motor3_integral);
    float motor4_pwm = motor_pid_compute(motor4_error, &motor4_integral);
    
    hardware_set_motor1_pwm(motor1_pwm);
    hardware_set_motor2_pwm(motor2_pwm);
    hardware_set_motor3_pwm(motor3_pwm);
    hardware_set_motor4_pwm(motor4_pwm);
}