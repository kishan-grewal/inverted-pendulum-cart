// #include <Arduino.h>
#include <math.h>
#include "drive.h"
#include "pendulum_encoder.h"
#include "motor_encoder.h"
#include "motor_pid.h"
#include "lqr.h"
#include "localisation_kalman.h"

#define START_BUTTON_PIN  10
#define CONTROL_SELECT_BUTTON_PIN  12

LocalisationKalman kalman;
LQRController lqr(-99.081f, -103.15984f, -496.930798f, -97.132137f);

float pendulum_encoder_angle = 0.0f;
const float pendulum_pulses_per_revolution = 1000.0f;
#define CALIBRATION_OFFSET_DEG (0.0f)

float dt = 0.0f;
unsigned long last_loop_time = 0;

const float LQR_VELOCITY_MAX =  2.0f;
const float LQR_VELOCITY_MIN = -2.0f;

static float v_target = 0.0f;
static float u_prev   = 0.0f;

void setup() {
  Serial.begin(115200);

  Serial.flush();
  Serial.println("Setup starting...");

  motor_setup();
  Serial.println("Motoron initialisation complete.");

  pendulum_encoder_setup();
  Serial.println("Pendulum Encoder initialisation complete.");

  motorEncoders_init();
  Serial.println("Motor Encoder initialisation complete.");

  lqr.setOutputLimits(-15.0f, 15.0f);

  last_loop_time = millis();  // prevent large first dt
  Serial.println("Setup complete. Entering loop.");
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CONTROL_SELECT_BUTTON_PIN, INPUT_PULLUP);

  Serial.println("Setup complete. Waiting for start button to be pressed...");

  while (digitalRead(START_BUTTON_PIN) == HIGH) {
    unsigned long start_time = micros();

    while (digitalRead(CONTROL_SELECT_BUTTON_PIN) == LOW) {

      // wait for 300ms button press before changing control mode
      if (micros() - start_time > 300000) {
        control_mode = control_mode + 1;

        if (control_mode > 1) {
          control_mode = 0;
        }

        Serial.print("Control mode: ");

        switch (control_mode) {
          case 0:
            Serial.println("LQR");
            break;

          case 1:
            Serial.println("PID");
            break;

          default:
            Serial.println("Unknown");
            break;
        }

        // exit inner loop to reset start time
        break;
      }
    }
  }
}

void loop() {
  // --- PENDULUM ANGLE ---
  noInterrupts();
  long pendulum_current_count = pendulum_encoder_pulse_count;
  interrupts();

  pendulum_encoder_angle = (pendulum_current_count / pendulum_pulses_per_revolution) * 360.0f;

  if      (pendulum_encoder_angle > 360.0f) pendulum_encoder_angle = fmod(pendulum_encoder_angle, 360.0f);
  else if (pendulum_encoder_angle < 0.0f)   pendulum_encoder_angle += 360.0f;

  pendulum_encoder_angle += CALIBRATION_OFFSET_DEG;

  if      (pendulum_encoder_angle > 180.0f)  pendulum_encoder_angle -= 360.0f;
  else if (pendulum_encoder_angle < -180.0f) pendulum_encoder_angle += 360.0f;

  // --- TIMING ---
  unsigned long current_time = millis();
  dt = (current_time - last_loop_time) / 1000.0f;
  last_loop_time = current_time;
  if (dt > 0.1f || dt <= 0.0f) dt = 0.01f;  // clamp: allow up to 100ms, reject zero

  // --- ENCODER DISTANCES & SPEEDS ---
  float d1, d2, d3, d4;
  encoders_getDistance(&d1, &d2, &d3, &d4);

  static float prev_d[4]         = {0, 0, 0, 0};
  float dists[4]                  = {d1, d2, d3, d4};
  float speeds[4];

  #define AVG_SIZE 10
  static float speed_buf[4][AVG_SIZE] = {};
  static int buf_idx = 0;

  for (int i = 0; i < 4; i++) {
    float raw = (dt > 0.001f) ? (dists[i] - prev_d[i]) / dt : 0.0f;
    prev_d[i] = dists[i];
    speed_buf[i][buf_idx] = raw;
    float sum = 0.0f;
    for (int j = 0; j < AVG_SIZE; j++) sum += speed_buf[i][j];
    speeds[i] = sum / AVG_SIZE;
  }
  buf_idx = (buf_idx + 1) % AVG_SIZE;

  // --- ANGLE GUARD: only run controller if pendulum is near upright ---
  if (fabsf(pendulum_encoder_angle) > 30.0f) {
    set_motor_speeds(0, 0, 0, 0);
    v_target = 0.0f;
    u_prev   = 0.0f;
    reset_motor_pids();
    return;
  }

  // --- KALMAN ---
  float z_cart[4] = {d1, d2, d3, d4};
  float theta_rad  = pendulum_encoder_angle * (3.14159265f / 180.0f);
  float z_velocity = (speeds[0] + speeds[1] + speeds[2] + speeds[3]) / 4.0f;
  kalman.update(u_prev, z_cart, 4, theta_rad, dt, z_velocity);

  float estimated_position = kalman.getPosition();
  float estimated_velocity = kalman.getVelocity();

  // --- LQR ---
  float state[4]  = { estimated_position, estimated_velocity, kalman.getTheta(), kalman.getThetaVelocity() };
  float target[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

  const float M_TOTAL = 1.515f;
  float lqr_force = lqr.compute(state, target);
  v_target += (lqr_force / M_TOTAL) * dt;
  v_target  = constrain(v_target, LQR_VELOCITY_MIN, LQR_VELOCITY_MAX);
  u_prev    = lqr_force;

  float desired_speed = v_target;

  // --- MOTOR PIDs ---
  int16_t pid_fl = compute_pid_front_left (desired_speed, speeds[0], dt);
  int16_t pid_fr = compute_pid_front_right(desired_speed, speeds[1], dt);
  int16_t pid_bl = compute_pid_back_left  (desired_speed, speeds[2], dt);
  int16_t pid_br = compute_pid_back_right (desired_speed, speeds[3], dt);

  // deadband compensation
  if (pid_fl > 5)  pid_fl += 40; else if (pid_fl < -5)  pid_fl -= 40;
  if (pid_fr > 5)  pid_fr += 40; else if (pid_fr < -5)  pid_fr -= 40;
  if (pid_bl > 5)  pid_bl += 40; else if (pid_bl < -5)  pid_bl -= 40;
  if (pid_br > 5)  pid_br += 40; else if (pid_br < -5)  pid_br -= 40;

  set_motor_speeds(pid_fl, pid_fr, pid_bl, pid_br);

  // --- SERIAL (teleplot, throttled 100ms) ---
  static unsigned long last_print = 0;
  static float dt_sum = 0.0f;
  static long  dt_n   = 0;
  dt_sum += dt * 1000.0f;
  dt_n++;

  if (current_time - last_print >= 100) {
    last_print = current_time;
    float dt_cumavg = dt_sum / dt_n;
    Serial.println(">desired:"            + String(desired_speed,          3));
    Serial.println(">actual:"             + String(speeds[0],              3));
    Serial.println(">estimated_velocity:" + String(estimated_velocity,     3));
    Serial.println(">pendulum_angle:"     + String(pendulum_encoder_angle, 3));
    Serial.println(">lqr_force:"          + String(lqr_force,              3));
    Serial.println(">kalman_theta:"       + String(kalman.getTheta(),      3));
    Serial.println(">kalman_v:"           + String(estimated_velocity,     3));
    Serial.println(">kalman_x:"           + String(estimated_position,     3));
    Serial.println(">pwm:"               + String((float)pid_fl / 1000.0f, 3));
    Serial.println(">dt_cumavg_ms:"       + String(dt_cumavg,              3));
  }
}