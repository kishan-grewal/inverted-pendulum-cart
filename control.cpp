#include "control.h"
#include "config.h"

static ControllerType active_controller = CTRL_LQR;
static float pid_integral = 0.0;

void control_init() {
    active_controller = CTRL_LQR;
    pid_integral = 0.0;
}

void control_set_controller(ControllerType type) {
    active_controller = type;
    control_reset();
}

void control_reset() {
    pid_integral = 0.0;
}

static float lqr_compute(float state[4]) {
    float K[4] = {LQR_K0, LQR_K1, LQR_K2, LQR_K3};
    float x_dot_cmd = -(K[0]*state[0] + K[1]*state[1] + K[2]*state[2] + K[3]*state[3]);
    return x_dot_cmd;
}

static float pid_compute(float state[4], float dt) {
    float theta = state[2];
    float theta_dot = state[3];
    
    float P = PID_KP * theta;
    
    pid_integral += theta * dt;
    if (pid_integral > PID_INTEGRAL_LIMIT) pid_integral = PID_INTEGRAL_LIMIT;
    if (pid_integral < -PID_INTEGRAL_LIMIT) pid_integral = -PID_INTEGRAL_LIMIT;
    float I = PID_KI * pid_integral;
    
    float D = PID_KD * theta_dot;
    
    float x_dot_cmd = P + I + D;
    return x_dot_cmd;
}

static float pole_compute(float state[4]) {
    float K[4] = {1.0, 2.0, 30.0, 6.0};
    float x_dot_cmd = -(K[0]*state[0] + K[1]*state[1] + K[2]*state[2] + K[3]*state[3]);
    return x_dot_cmd;
}

float control_compute(float state[4], float dt) {
    float x_dot_cmd = 0.0;
    
    if (active_controller == CTRL_LQR) {
        x_dot_cmd = lqr_compute(state);
    } else if (active_controller == CTRL_PID) {
        x_dot_cmd = pid_compute(state, dt);
    } else if (active_controller == CTRL_POLE) {
        x_dot_cmd = pole_compute(state);
    }
    
    return x_dot_cmd;
}