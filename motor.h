#ifndef MOTOR_H
#define MOTOR_H

void motor_init();
void motor_reset();
void motor_set_velocity_command(float velocity_cmd);
void motor_update(float dt);

#endif