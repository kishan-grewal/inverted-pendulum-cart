#include "encoder.h"

volatile long pulse_count = 0;


void encoder_setup() {
    pinMode(encoderA, INPUT_PULLUP);
    pinMode(encoderB, INPUT_PULLUP);
    // pinMode(encoderI, INPUT_PULLUP);

    // Attach separate handlers for A and B
    attachInterrupt(digitalPinToInterrupt(encoderA), handle_A_rising, RISING);

    // Index pulse for absolute zero
    attachInterrupt(digitalPinToInterrupt(encoderI), handle_I_pulse, RISING);
}

void handle_A_rising() {
    /* When A rises: 
       If B is LOW, A is leading B (Clockwise)
       If B is HIGH, B is leading A (Counter-Clockwise) */
    if (digitalRead(encoderB) == LOW) {
        pulse_count++; // CW
    } else {
        pulse_count--; //CCW
    }
}

// void handle_B_rising() {
//     /* When B rises: 
//        If A is HIGH, A transitioned first (Clockwise)
//        If A is LOW, B transitioned first (Counter-Clockwise) */
//     if (digitalRead(encoderA) == HIGH) {
//         pulse_count++;
//     } else {
//         pulse_count--;
//     }
// }

void handle_I_pulse() {
    // Reset pulse count to 0 once per full 360-degree rotation
    pulse_count = 0;
}