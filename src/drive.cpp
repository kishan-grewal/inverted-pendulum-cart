#include <Motoron.h>

Motoron motoron1(0x20);

void motor_setup() {
    motoron1.begin();
    motoron1.setSpeed(0);
}

void set_motor_speed(int16_t speed) {
	// Limit speed
    if (speed > 3200) speed = 3200;
    if (speed < -3200) speed = -3200;

    if (speed >= 0) {
        // Forward
        HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_SET);
    } else {
        // Reverse
        HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_RESET);
    }

    // Use abs() to make pulse_width positive
    uint32_t pulse_width = abs(speed);

    // Update the Timer Capture Compare Register
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_width);
}