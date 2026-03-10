// #include <Arduino.h>
#include <math.h>
#include "drive.h"
#include "pendulum_encoder.h"
#include "motor_encoder.h"
#include "motor_pid.h"
#include "lqr.h"
#include "localisation_kalman.h"

LocalisationKalman kalman;
// LQR gains from python/lqr_gains.py (force output F = -K*x; accel = F/M_TOTAL)
LQRController lqr(-99.081f, -103.15984f, -496.930798f, -97.132137f);

float pendulum_encoder_angle = 0.0;
const float pendulum_pulses_per_revolution = 1000;
#define CALIBRATION_OFFSET_DEG (-78.84f)
#define CALIBRATION_OFFSET_DEG (0.0f)

unsigned long t0 = 0;
unsigned long t1 = 0;
float dt = 0.0;

unsigned long last_loop_time = 0;

// LQR: integrate acceleration to get velocity setpoint; clamp to this range
const float LQR_VELOCITY_MAX = 0.5f;   // m/s
const float LQR_VELOCITY_MIN = -0.5f;  // m/s

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("Setup starting...");
  Serial.flush();

  Serial.println("Initialising Motoron controller...");
  motor_setup();
  Serial.println("Motoron initialisation complete.");

  Serial.println("Initialising Pendulum Encoder...");
  pendulum_encoder_setup();
  Serial.println("Pendulum Encoder initialisation complete.");

  Serial.println("Initialising Motor Encoders...");
  motorEncoders_init();
  Serial.println("Motor Encoder initialisation complete.");

  // LQR output is force [N]; limit to avoid excessive acceleration
  // lqr.setOutputLimits(-10.0f, 10.0f);
  lqr.setOutputLimits(-15.0f, 15.0f);  // F_max = M_t * g = 1.515 * 9.81 = 14.86 N

  Serial.println("Setup complete. Entering loop.");
  Serial.flush();
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
  pendulum_encoder_angle += CALIBRATION_OFFSET_DEG;

  // Wrap to [-180, 180]
  if (pendulum_encoder_angle > 180.0f) {
    pendulum_encoder_angle -= 360.0f;
  } else if (pendulum_encoder_angle < -180.0f) {
    pendulum_encoder_angle += 360.0f;
  }

  // Calculate time delta MILLIS / MICROS
  unsigned long current_time = millis();
  dt = (current_time - last_loop_time) / 1000.0;  // Milli to seconds
  // unsigned long current_time = micros();
  // dt = (current_time - last_loop_time) / 1000000.0f;  // Micro to seconds
  last_loop_time = current_time;

  // Protect against first iteration
  if (dt > 1.0) {
    dt = 0.01;  // Default to 10ms if something is wrong
  } else if (dt <= 0.0) { dt = 0.01; } // Prevent zero or negative dt

  // Read encoder distances
  float d1, d2, d3, d4;
  encoders_getDistance(&d1, &d2, &d3, &d4);

  // Differentiate to get speed
  static float prev_d[4] = {0, 0, 0, 0};
  float dists[4] = {d1, d2, d3, d4};
  float speeds[4];

  #define AVG_SIZE 10
  static float speed_buf[4][AVG_SIZE] = {};
  static int buf_idx = 0;

  for (int i = 0; i < 4; i++) {
    float raw = 0.0f;
    if (dt > 0.001f) {
      raw = (dists[i] - prev_d[i]) / dt;
    }
    prev_d[i] = dists[i];
    speed_buf[i][buf_idx] = raw;

    float sum = 0.0f;
    for (int j = 0; j < AVG_SIZE; j++) sum += speed_buf[i][j];
    speeds[i] = sum / AVG_SIZE;
  }
  buf_idx = (buf_idx + 1) % AVG_SIZE;

  // Throttle serial to avoid buffer overflow / watchdog; print every ~100 ms
  static unsigned long last_print = 0;
  bool do_print = (current_time - last_print >= 100);
  // if (do_print) {
  //   last_print = current_time;
  //   Serial.println("Encoder Speed (m/s) [FL, FR, BL, BR]: " + String(speeds[0], 3) + ", " + String(speeds[1], 3) + ", " + String(speeds[2], 3) + ", " + String(speeds[3], 3));
  // }

  // Kalman filter: estimate state [x, v, theta, theta_dot]; velocity update reduces bias when cart at rest
  float z_cart[4] = {d1, d2, d3, d4};
  float theta_rad = pendulum_encoder_angle * (3.14159265f / 180.0f);
  float z_velocity = (speeds[0] + speeds[1] + speeds[2] + speeds[3]) / 4.0f;  // avg encoder speed [m/s]
  static float u_prev = 0.0f;  // previous LQR force for Kalman prediction
  kalman.update(u_prev, z_cart, 4, theta_rad, dt, z_velocity);

  float estimated_position = kalman.getPosition();
  float estimated_velocity = kalman.getVelocity();

  // LQR state: [x, x_dot, theta, theta_dot] from Kalman (theta from encoder fused in Kalman)
  float state[4] = {
    estimated_position,
    estimated_velocity,
    kalman.getTheta(),
    kalman.getThetaVelocity()
  };
  float target[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // upright at rest

  // LQR outputs force [N]; convert to acceleration and integrate for velocity setpoint
  const float M_TOTAL = 1.515f;
  float lqr_force = lqr.compute(state, target);
  float lqr_accel = lqr_force / M_TOTAL;
  static float v_target = 0.0f;
  v_target += lqr_accel * dt;
  if (v_target > LQR_VELOCITY_MAX) v_target = LQR_VELOCITY_MAX;
  if (v_target < LQR_VELOCITY_MIN) v_target = LQR_VELOCITY_MIN;

  float desired_speed = v_target;

  // Force for next Kalman prediction [N]
  u_prev = lqr_force;

  // Compute PID output for each motor
  int16_t pid_front_left = compute_pid_front_left(desired_speed, speeds[0], dt);
  int16_t pid_front_right = compute_pid_front_right(desired_speed, speeds[1], dt);
  int16_t pid_back_left = compute_pid_back_left(desired_speed, speeds[2], dt);
  int16_t pid_back_right = compute_pid_back_right(desired_speed, speeds[3], dt);

  if (pid_front_left > 5) {
    pid_front_left = pid_front_left + 40;
  } else if (pid_front_left < -5) {
    pid_front_left = pid_front_left - 40;
  }

  if (pid_front_right > 5) {
    pid_front_right = pid_front_right + 40;
  } else if (pid_front_right < -5) {
    pid_front_right = pid_front_right - 40;
  }

  if (pid_back_left > 5) {
    pid_back_left = pid_back_left + 40;
  } else if (pid_back_left < -5) {
    pid_back_left = pid_back_left - 40;
  }

  if (pid_back_right > 5) {
    pid_back_right = pid_back_right + 40;
  } else if (pid_back_right < -5) {
    pid_back_right = pid_back_right - 40;
  }

  // Apply PID outputs to motors
  set_motor_speeds(pid_front_left, pid_front_right, pid_back_left, pid_back_right);
  // set_motor_speeds(pid_front_left, 0, 0, 0);  // TEST: Only control front left motor for now

  // Debug output (throttled, same 100 ms as above)
  // if (do_print) {
  //   Serial.println("Pendulum Angle: " + String(pendulum_encoder_angle));
  //   Serial.println("Motor PID [FL, FR, BL, BR]: " + String(pid_front_left) + ", " + String(pid_front_right) + ", " + String(pid_back_left) + ", " + String(pid_back_right));
  //   Serial.println("---");
  // }

  // dt average finder
  static float dt_sum = 0.0f;
  static long dt_n = 0;
  dt_sum += dt * 1000.0f;
  dt_n++;
  float dt_cumavg = dt_sum / dt_n;

  // for plotting (kalman_graph, lqr_graph)
  if (do_print) {
    last_print = current_time;
    Serial.println(">desired:" + String(desired_speed, 3));
    Serial.println(">actual:" + String(speeds[0], 3));
    Serial.println(">estimated_velocity:" + String(estimated_velocity, 3));
    Serial.println(">pendulum_angle:" + String(pendulum_encoder_angle, 3));
    Serial.println(">pwm:" + String((float)pid_front_left / 1000.0, 3));

    Serial.println(">dt_cumavg_ms:" + String(dt_cumavg, 3));
  }
}