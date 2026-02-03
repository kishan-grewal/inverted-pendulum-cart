#include <Motoron.h>
#include <Wire.h>
#include "drive.h"

MotoronI2C bottom_shield(16);
MotoronI2C top_shield(17);

void motor_setup() {
    Wire1.begin();

    // Configure shields to use Wire1 (SCL1/SDA1)
    bottom_shield.setBus(&Wire1);
    //top_shield.setBus(&Wire1);

    // Initialize shields
    bottom_shield.reinitialize();
    bottom_shield.clearResetFlag();
    //top_shield.reinitialize();
    //top_shield.clearResetFlag();

    // Bottom shield max accels
    bottom_shield.setMaxAcceleration(1, 140);
    bottom_shield.setMaxDeceleration(1, 300);
    //bottom_shield.setMaxAcceleration(2, 140);
    //bottom_shield.setMaxDeceleration(2, 300);

    // Top shield max accels
    //top_shield.setMaxAcceleration(1, 140);
    //top_shield.setMaxDeceleration(1, 300);
    //top_shield.setMaxAcceleration(2, 140);
    //top_shield.setMaxDeceleration(2, 300);
}

void set_motor_speed(int16_t speed) {
	// Limit speed
    if (speed > 3200) speed = 3200;
    if (speed < -3200) speed = -3200;

    if (speed >= 0) {
        // Forward
        bottom_shield.setSpeed(1, speed);
    } else {
        // Reverse
        bottom_shield.setSpeed(1, speed);
    }
}