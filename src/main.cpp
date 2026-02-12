#include <Arduino.h>
#include "drive.h"
#include "pendulum_encoder.h"
#include "motor_encoder.h"
#include "motor_pid.h"
#include "lqr.h"
#include "localisation_kalman.h"

LocalisationKalman kalman;
LQRController lqr(2.0, 1.0, 0.5, 0.1); // Random gains, need to be computed

float pendulum_encoder_angle = 0.0;
const float pendulum_pulses_per_revolution = 1000;

unsigned long t0 = 0.0;
unsigned long t1 = 0.0;
float dt = 0.0;

unsigned long last_loop_time = 0;

const float desired_speed = 1.0; // m/s

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    // Wait for Serial port to connect (up to 3 seconds)
  }
  
  Serial.println("Initialising Motoron controller...");
  motor_setup();
  Serial.println("Motoron initialisation complete.");

  Serial.println("Initialising Pendulum Encoder...");
  pendulum_encoder_setup();
  Serial.println("Pendulum Encoder initialisation complete.");

  Serial.println("Initialising Motor Encoders...");
  motor_encoder_setup();
  Serial.println("Motor Encoder initialisation complete.");
}

void loop() {
  t0 = millis();

  // Read pendulum angle
  noInterrupts();
  long pendulum_current_count = pendulum_encoder_pulse_count;
  interrupts();

  pendulum_encoder_angle = (pendulum_current_count / pendulum_pulses_per_revolution) * 360.0;

  if (pendulum_encoder_angle > 360.0) {
    pendulum_encoder_angle = fmod(pendulum_encoder_angle, 360.0);
  } else if (pendulum_encoder_angle < 0.0) {
    pendulum_encoder_angle += 360.0;
  }

  // Calculate time delta
  unsigned long current_time = millis();
  dt = (current_time - last_loop_time) / 1000.0;  // Convert to seconds
  last_loop_time = current_time;

  // Protect against first iteration
  if (dt > 1.0) {
    dt = 0.01;  // Default to 10ms if something is wrong
  }

  // Read actual speeds from encoders
  float actual_speed_front_left = read_encoder_speed(0);
  float actual_speed_front_right = read_encoder_speed(1);
  float actual_speed_back_left = read_encoder_speed(2);
  float actual_speed_back_right = read_encoder_speed(3);

  // Kalman filter to estimate cart position and velocity
  // float z[4] = {actual_speed_front_left, actual_speed_front_right, actual_speed_back_left, actual_speed_back_right};
  // kalman.update(z, 4, dt);
  // float estimated_position = kalman.getPosition();
  // float estimated_velocity = kalman.getVelocity();

  // ==================== NEED TO ADD LQR HERE ====================
  // use estimated position, velocity, pendulum angle, pendulum angular velocity to compute control output

  // Compute PID output for each motor
  int16_t pid_front_left = compute_pid_front_left(desired_speed, actual_speed_front_left, dt);
  int16_t pid_front_right = compute_pid_front_right(desired_speed, actual_speed_front_right, dt);
  int16_t pid_back_left = compute_pid_back_left(desired_speed, actual_speed_back_left, dt);
  int16_t pid_back_right = compute_pid_back_right(desired_speed, actual_speed_back_right, dt);

  // Apply PID outputs to motors
  set_motor_speeds(pid_front_left, pid_front_right, pid_back_left, pid_back_right);

  // Debug output
  // Serial.println("Pendulum Encoder Angle (Degrees): " + String(pendulum_encoder_angle, 2));
  // Serial.println("Motor Speeds (m/s) [FL, FR, BL, BR]: " + String(actual_speed_front_left, 3) + ", " + String(actual_speed_front_right, 3) + ", " + String(actual_speed_back_left, 3) + ", " + String(actual_speed_back_right, 3));
  Serial.println("Motor PID [FL, FR, BL, BR]: " + String(pid_front_left) + ", " + String(pid_front_right) + ", " + String(pid_back_left) + ", " + String(pid_back_right));
  Serial.println("---");
}