#include <Motoron.h>
#include <Wire.h>
#include "drive.h"

MotoronI2C motoron1; // I2C address 0x10


void motor_setup() {
    // Initialize I2C first, then set it for Motoron
    Wire.begin();
    motoron1.setBus(&Wire);
    
    // Reinitialize the Motoron controller
    motoron1.reinitialize();
    motoron1.disableCrc();

    // Clear any reset flags
    motoron1.clearResetFlag();

    // Set acceleration limits for motor 1
    motoron1.setMaxAcceleration(1, 70);
    motoron1.setMaxDeceleration(1, 150);
    
    // Set acceleration limits for motor 2 (if using both motors)
    motoron1.setMaxAcceleration(2, 70);
    motoron1.setMaxDeceleration(2, 150);

    // Clear any motor faults
    motoron1.clearMotorFaultUnconditional();

    // Initialize both motors to zero speed
    motoron1.setSpeed(1, 0);
    motoron1.setSpeed(2, 0);
}

void set_motor_speed(int16_t speed) {
	// Limit speed
    if (speed > 3200) speed = 3200;
    if (speed < -3200) speed = -3200;

    motoron1.setSpeed(1, speed);
    motoron1.setSpeed(2, speed);

}