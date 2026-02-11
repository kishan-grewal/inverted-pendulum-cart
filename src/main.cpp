#include <Arduino.h>
#include "drive.h"
#include "encoder.h"
#include "motor_pid.h"
#include "lqr.h"
#include "localisation_kalman.h"

LocalisationKalman kalman;
LQRController lqr(2.0, 1.0, 0.5, 0.1); // Randon gains, need to be computed

float pendulum_encoder_angle = 0.0;
const float pulses_per_revolution = 1000;

float error = 0.0;
float previous_error = 0.0;
float derivative = 0.0;
float integral = 0.0;
float control_signal = 0.0;

unsigned long t0 = 0.0;
unsigned long t1 = 0.0;
float dt = 0.0;

const float desired_speed = 1.0; // m/s
float actual_speed_front_left = 0.0;
float actual_speed_front_right = 0.0;
float actual_speed_back_left = 0.0;
float actual_speed_back_right = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    // Wait for Serial port to connect (up to 3 seconds)
  }
  
  Serial.println("Initialising Motoron controller...");
  motor_setup();
  Serial.println("Motoron initialisation complete.");

  Serial.println("Initialising Encoder...");
  encoder_setup();
  Serial.println("Encoder initialisation complete.");
}

void loop() {
  t0 = millis();

  noInterrupts();
  long current_count = pulse_count;
  interrupts();

  pendulum_encoder_angle = (current_count / pulses_per_revolution) * 360.0;

  if (pendulum_encoder_angle > 360.0) {
    pendulum_encoder_angle = fmod(pendulum_encoder_angle, 360.0);

  } else if (pendulum_encoder_angle < 0.0) {
    pendulum_encoder_angle += 360.0;
  }

  error = 180.0 - pendulum_encoder_angle;
  derivative = error - previous_error;

  t1 = millis();

  dt = (t1 - t0) / 1000.0; // Convert milliseconds to seconds
  
  integral += error * dt;

  control_signal = (MOTOR_KP * error) + (MOTOR_KI * integral) + (MOTOR_KD * derivative);

  previous_error = error;

  // Compute PID output for each motor
  int16_t pid_front_left = compute_pid_front_left(desired_speed, actual_speed_front_left, dt);
  int16_t pid_front_right = compute_pid_front_right(desired_speed, actual_speed_front_right, dt);
  int16_t pid_back_left = compute_pid_back_left(desired_speed, actual_speed_back_left, dt);
  int16_t pid_back_right = compute_pid_back_right(desired_speed, actual_speed_back_right, dt);

  // Apply PID outputs to motors
  set_motor_speeds(pid_front_left, pid_front_right, pid_back_left, pid_back_right);

  Serial.println("Encoder Angle (Degrees): " + String(pendulum_encoder_angle, 2));
  Serial.println("Control Signal: " + String(control_signal, 2));
  Serial.println("Motor PID [FL, FR, BL, BR]: " + String(pid_front_left) + ", " + String(pid_front_right) + ", " + String(pid_back_left) + ", " + String(pid_back_right));

}