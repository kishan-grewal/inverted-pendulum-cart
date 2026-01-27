#ifndef CONTROL_H
#define CONTROL_H

enum ControllerType {
    CTRL_LQR,
    CTRL_PID,
    CTRL_POLE
};

void control_init();
void control_set_controller(ControllerType type);
float control_compute(float state[4], float dt);
void control_reset();

#endif