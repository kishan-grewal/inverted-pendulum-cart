#ifndef OBSERVER_H
#define OBSERVER_H

void observer_init();
void observer_reset();
void observer_update(float y_measured[2], float u_velocity, float dt, float state_out[4]);

#endif