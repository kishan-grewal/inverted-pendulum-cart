#include "motor_encoder.h"

volatile long motor_encoder_pulse_counts[4] = {0, 0, 0, 0};
long last_pulse_counts[4] = {0, 0, 0, 0};

int last_state_A[4] = {0, 0, 0, 0};
int last_state_B[4] = {0, 0, 0, 0};

// FIX 1: per-encoder timestamp array instead of a single shared value
unsigned long last_speed_update_time[4] = {0, 0, 0, 0};

const int encoder_pin_A[4] = {encoder_front_left_encoder_A, encoder_front_right_encoder_A, encoder_back_left_encoder_A, encoder_back_right_encoder_A};
const int encoder_pin_B[4] = {encoder_front_left_encoder_B, encoder_front_right_encoder_B, encoder_back_left_encoder_B, encoder_back_right_encoder_B};

const float PULSES_PER_REVOLUTION = 1000.0;
const float WHEEL_DIAMETER_MM = 70.0;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_MM * PI / 1000.0;
const float GEAR_RATIO = 9.7;

const float SPEED_FILTER_ALPHA = 0.8;
float filtered_speed[4] = {0, 0, 0, 0};

// FIX 4: named ISR wrappers instead of capturing lambdas
void isr_encoder_0() { handle_encoder(0); }
void isr_encoder_1() { handle_encoder(1); }
void isr_encoder_2() { handle_encoder(2); }
void isr_encoder_3() { handle_encoder(3); }

void motor_encoder_setup() {
    void (*isr_table[4])() = {isr_encoder_0, isr_encoder_1, isr_encoder_2, isr_encoder_3};

    for (int i = 0; i < 4; i++) {
        pinMode(encoder_pin_A[i], INPUT);
        pinMode(encoder_pin_B[i], INPUT);

        last_state_A[i] = digitalRead(encoder_pin_A[i]);
        last_state_B[i] = digitalRead(encoder_pin_B[i]);

        attachInterrupt(digitalPinToInterrupt(encoder_pin_A[i]), isr_table[i], CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoder_pin_B[i]), isr_table[i], CHANGE);

        // FIX 1: initialise each encoder's timestamp individually
        last_speed_update_time[i] = millis();
    }
}

void handle_encoder(int encoder_index) {
    int state_A = digitalRead(encoder_pin_A[encoder_index]);
    int state_B = digitalRead(encoder_pin_B[encoder_index]);

    if (state_A != last_state_A[encoder_index]) {
        if (last_state_A[encoder_index] != last_state_B[encoder_index]) {
            motor_encoder_pulse_counts[encoder_index]++;
        } else {
            motor_encoder_pulse_counts[encoder_index]--;
        }
    }
    else if (state_B != last_state_B[encoder_index]) {
        if (state_A == state_B) {
            motor_encoder_pulse_counts[encoder_index]++;
        } else {
            motor_encoder_pulse_counts[encoder_index]--;
        }
    }

    last_state_A[encoder_index] = state_A;
    last_state_B[encoder_index] = state_B;
}

float read_encoder_speed(int encoder_index) {
    if (encoder_index < 0 || encoder_index > 3) {
        return 0.0;
    }

    unsigned long current_time = millis();
    // FIX 1: use per-encoder timestamp
    unsigned long time_elapsed = current_time - last_speed_update_time[encoder_index];

    if (time_elapsed == 0) {
        return filtered_speed[encoder_index];
    }

    noInterrupts();
    long current_pulse_count = motor_encoder_pulse_counts[encoder_index];
    interrupts();

    long pulse_delta = current_pulse_count - last_pulse_counts[encoder_index];

    last_pulse_counts[encoder_index] = current_pulse_count;
    // FIX 1: update per-encoder timestamp
    last_speed_update_time[encoder_index] = current_time;

    float revolutions = pulse_delta / (PULSES_PER_REVOLUTION * GEAR_RATIO);
    float distance_m = revolutions * WHEEL_CIRCUMFERENCE;
    float time_s = time_elapsed / 1000.0;
    float raw_speed = distance_m / time_s;

    filtered_speed[encoder_index] = (SPEED_FILTER_ALPHA * raw_speed) + ((1.0 - SPEED_FILTER_ALPHA) * filtered_speed[encoder_index]);

    return filtered_speed[encoder_index];
}

void reset_encoder(int encoder_index) {
    if (encoder_index < 0 || encoder_index > 3) {
        return;
    }

    noInterrupts();
    motor_encoder_pulse_counts[encoder_index] = 0;
    interrupts();

    last_pulse_counts[encoder_index] = 0;
    filtered_speed[encoder_index] = 0.0;
}

void reset_all_encoders() {
    for (int i = 0; i < 4; i++) {
        reset_encoder(i);
    }
}

float get_wheel_distance(int encoder_index) {
    if (encoder_index < 0 || encoder_index > 3) {
        return 0.0;
    }

    noInterrupts();
    long current_pulse_count = motor_encoder_pulse_counts[encoder_index];
    interrupts();

    float revolutions = current_pulse_count / (PULSES_PER_REVOLUTION * GEAR_RATIO);
    return revolutions * WHEEL_CIRCUMFERENCE;
}