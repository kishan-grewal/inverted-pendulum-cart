#include <Motoron.h>
#include <Wire.h>
#include "drive.h"

MotoronI2C motoron1(0x10); // I2C address 0x10


void motor_setup() {
    Wire.begin();
    motoron1.reinitialize();
    motoron1.disableCrc();

    motoron1.clearResetFlag();

    motoron1.setMaxAcceleration(1, 70);
    motoron1.setMaxDeceleration(1, 150);

    motoron1.clearMotorFaultUnconditional();

    motoron1.setSpeed(1, 0);
}

void set_motor_speed(int16_t speed) {
	// Limit speed
    if (speed > 3200) speed = 3200;
    if (speed < -3200) speed = -3200;

    if (speed >= 0) {
        // Forward
        motoron1.setSpeed(1, speed);
    } else {
        // Reverse
        motoron1.setSpeed(1, speed);
    }

}