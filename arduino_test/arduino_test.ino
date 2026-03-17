// #include <Arduino.h>
#include <math.h>
#include "drive.h"
#include "pendulum_encoder.h"
#include "motor_encoder.h"
#include "motor_pid.h"
#include "lqr.h"
#include "localisation_kalman.h"
#include "cascaded_pid.h"

#define START_BUTTON_PIN  10
#define CONTROL_SELECT_BUTTON_PIN  12
#define TASK_SELECT_BUTTON_PIN  13

LocalisationKalman kalman;
LQRController lqr(-99.081f, -128.098f, -757.579f, -116.691f);
LQRController pole(-99.081f, -128.098f, -757.579f, -116.691f);

float pendulum_encoder_angle = 0.0f;  // degrees, for Serial/display
#define CALIBRATION_OFFSET_DEG (0.0f)

#define AVG_SIZE 10

float dt = 0.0f;
unsigned long last_loop_time = 0;

const float LQR_VELOCITY_MAX =  1.25f;
const float LQR_VELOCITY_MIN = -1.25f;

static float v_target = 0.0f;
static float u_prev   = 0.0f;

int control_mode = 1; // 0 = LQR, 1 = POLE
int task_mode = 0; //0 = stabilisation, 1 = sprint

float start_time = 0.0f;
float x_final = 0.0f;
float x_target = 0.0f;

// Sprint trajectory parameters (1 m move; sign = direction)
const float SPRINT_DISTANCE_M = 1.0f;
const float SPRINT_A_MAX      = 0.1f;   // [m/s^2] conservative acceleration
const float SPRINT_V_MAX      = 0.175f;  // [m/s]   conservative cruise speed

// Derived sprint timing (computed in setup)
float sprint_t_accel  = 0.0f;  // duration of accel phase  [s]
float sprint_t_cruise = 0.0f;  // duration of cruise phase [s]
float sprint_t_total  = 0.0f;  // total sprint duration    [s]

// Sprint state
bool  sprint_active   = false;
float sprint_x_start  = 0.0f;

const float PENDULUM_KP = 1.0f;
const float PENDULUM_KI = 0.00001f;
const float PENDULUM_KD = 0.001f;
const float PENDULUM_X_KP = 10.0f;
const float PENDULUM_X_MAX = 2.0f;
const float PENDULUM_X_INTEGRAL_LIMIT = 0.2f;

CascadedPID cascaded_pid(PENDULUM_KP, PENDULUM_KI, PENDULUM_KD, PENDULUM_X_KP, PENDULUM_X_MAX, PENDULUM_X_INTEGRAL_LIMIT);

void setup() {
  Serial.begin(250000);

  Serial.flush();
  Serial.println("Setup starting...");

  motor_setup();
  Serial.println("Motoron initialisation complete.");

  pendulumEncoder_init();
  Serial.println("Pendulum Encoder initialisation complete.");

  motorEncoders_init();
  Serial.println("Motor Encoder initialisation complete.");

  lqr.setOutputLimits(-15.0f, 15.0f);
  pole.setOutputLimits(-15.0f, 15.0f);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CONTROL_SELECT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TASK_SELECT_BUTTON_PIN, INPUT_PULLUP);

  Serial.println("Setup complete. Waiting for start button to be pressed...");

  while (digitalRead(START_BUTTON_PIN) == HIGH) {
    static bool control_mode_changed = false;
    static bool task_mode_changed = false;

    // Reset flag when control select button is released
    if (digitalRead(CONTROL_SELECT_BUTTON_PIN) == HIGH) {
      control_mode_changed = false;
    }

    if (digitalRead(TASK_SELECT_BUTTON_PIN) == HIGH) {
      task_mode_changed = false;
    }

    start_time = micros();

    while (digitalRead(CONTROL_SELECT_BUTTON_PIN) == LOW) {

      // wait for 200ms button press before changing control mode
      if ((micros() - start_time > 200000) && !control_mode_changed) {
        control_mode = control_mode + 1;
        control_mode_changed = true;

        if (control_mode > 1) {
          control_mode = 0;
        }

        Serial.print("Control mode: ");

        switch (control_mode) {
          case 0:
            Serial.println("LQR");
            break;

          case 1:
            Serial.println("POLE");
            break;

          default:
            Serial.println("Unknown");
            break;
        }

        // exit inner loop to reset start time
        break;
      }
    }

    while (digitalRead(TASK_SELECT_BUTTON_PIN) == LOW) {

      // wait for 200ms button press before changing task mode
      if ((micros() - start_time > 200000) && !task_mode_changed) {
        task_mode = task_mode + 1;
        task_mode_changed = true;

        if (task_mode > 1) {
          task_mode = 0;
        }

        Serial.print("Task mode: ");

        switch (task_mode) {
          case 0:
            Serial.println("Stabilisation");
            break;

          case 1:
            Serial.println("Sprint");
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

  delay(1000);

  if (task_mode == 1) {
    x_final = 2.0f;
  }

  // Pre-compute sprint timings for trapezoidal/triangular profile
  {
    const float D_signed = SPRINT_DISTANCE_M;
    const float D        = fabsf(D_signed);   // use magnitude for timing
    const float a        = SPRINT_A_MAX;
    const float vm       = SPRINT_V_MAX;

    float t_accel = vm / a;
    float d_accel = 0.5f * a * t_accel * t_accel;

    if (2.0f * d_accel >= D) {
      // Triangular profile: no cruise, reduce peak speed
      t_accel = sqrtf(D / a);
      sprint_t_accel  = t_accel;
      sprint_t_cruise = 0.0f;
      sprint_t_total  = 2.0f * t_accel;
    } else {
      // Trapezoidal profile with cruise
      float d_cruise = D - 2.0f * d_accel;
      float t_cruise = d_cruise / vm;
      sprint_t_accel  = t_accel;
      sprint_t_cruise = t_cruise;
      sprint_t_total  = 2.0f * t_accel + t_cruise;
    }
  }

  start_time = micros();
  last_loop_time = micros();  // prevent large first dt
}

void loop() {


  // Pause: 20ms press stops loop. Resume: 200ms hold to continue (then reset time/PIDs to avoid windup).
  if (digitalRead(START_BUTTON_PIN) == LOW) {
    unsigned long pause_start = micros();
    while (digitalRead(START_BUTTON_PIN) == LOW) {
      if ((micros() - pause_start) >= 20000) {
        // 20ms held: we're paused. Wait for release, then 200ms hold to continue.
        Serial.println("Paused");
        while (digitalRead(START_BUTTON_PIN) == HIGH) {
          delay(5);
        }
        unsigned long hold_start = micros();
        while (true) {
          if (digitalRead(START_BUTTON_PIN) == HIGH) {
            hold_start = micros();
          } else if ((micros() - hold_start) >= 200000) {
            break;  // 200ms hold to continue
          }
          delay(5);
        }
        // Reset time and integrators so no windup on first frame after resume
        reset_motor_pids();
        last_loop_time = micros();
        start_time = last_loop_time;
        cascaded_pid.reset();
        v_target = 0.0f;
        u_prev   = 0.0f;
        Serial.println("Resumed");
        // Wait for release so next loop() iteration doesn't re-enter pause
        while (digitalRead(START_BUTTON_PIN) == LOW) {
          delay(5);
        }
        break;
      }
    }
  }

  
  // Unwrapped radians from encoder; apply calibration
  float angle_rad = get_pendulum_angle_rad() + (CALIBRATION_OFFSET_DEG * (3.14159265f / 180.0f));
  pendulum_encoder_angle = angle_rad * (180.0f / 3.14159265f);  // degrees for Serial/display

  // Timing
  unsigned long current_time = micros();
  dt = (current_time - last_loop_time) / 1000000.0f;
  last_loop_time = current_time;
  if (dt > 0.1f || dt <= 0.0f) dt = 0.001f;  // clamp: allow up to 100ms, reject zero

  // Encoder distances & speeds
  float d1, d2, d3, d4;
  encoders_getDistance(&d1, &d2, &d3, &d4);

  static float prev_d[4]         = {0, 0, 0, 0};
  float dists[4]                  = {d1, d2, d3, d4};
  float speeds[4];

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

  // Angle guard: only run controller if pendulum is near upright
  // if (fabsf(pendulum_encoder_angle) > 30.0f) {
  //   set_motor_speeds(0, 0, 0, 0);
  //   v_target = 0.0f;
  //   u_prev   = 0.0f;
  //   reset_motor_pids();
  //   sprint_active = false;
  //   return;
  // }

  // Kalman filter
  float z_cart[4] = {d1, d2, d3, d4};
  float z_velocity = (speeds[0] + speeds[1] + speeds[2] + speeds[3]) / 4.0f;
  kalman.update(u_prev, z_cart, 4, angle_rad, dt, z_velocity);

  float estimated_position = kalman.getPosition();
  float estimated_velocity = kalman.getVelocity();

  float desired_speed = 0.0f;

  float lqr_force = 0.0f;

  current_time = micros();

  // Compute sprint reference trajectory (position & velocity) and pendulum lean
  float x_ref        = 0.0f;
  float xdot_ref     = 0.0f;
  float theta_ref    = 0.0f;
  float theta_dot_ref = 0.0f;

  if (task_mode == 1) {
    if (!sprint_active) {
      // Lazily initialise sprint start on first loop in sprint mode
      sprint_active  = true;
      sprint_x_start = estimated_position;
      start_time     = current_time;
    }

    float t = (current_time - start_time) / 1000000.0f;  // [s] since sprint start

    if (t >= sprint_t_total) {
      // Sprint finished: stabilise about hard-coded origin (x = 0)
      x_ref         = 0.0f;
      xdot_ref      = 0.0f;
      theta_ref     = 0.0f;
      theta_dot_ref = 0.0f;
      sprint_active = false;
    } else {
      const float t1 = sprint_t_accel;
      const float t2 = sprint_t_accel + sprint_t_cruise;
      const float a  = SPRINT_A_MAX;
      const float vm = SPRINT_V_MAX;
      const float dir = (SPRINT_DISTANCE_M >= 0.0f) ? 1.0f : -1.0f;

      if (t < t1) {
        // Acceleration phase
        xdot_ref = dir * (a * t);
        x_ref    = sprint_x_start + dir * (0.5f * a * t * t);
        // Small forward lean during acceleration
        const float theta_fwd_rad = 7.0f * (3.14159265f / 180.0f);  // 5 degrees
        theta_ref     = theta_fwd_rad * (t / t1);
        theta_dot_ref = theta_fwd_rad / t1;
      } else if (t < t2) {
        // Cruise phase (if any)
        xdot_ref = dir * vm;
        float d_accel = 0.5f * a * t1 * t1;
        x_ref    = sprint_x_start + dir * (d_accel + vm * (t - t1));
        theta_ref     = 0.0f;
        theta_dot_ref = 0.0f;
      } else {
        // Deceleration phase
        float t_dec       = t - t2;
        float t_dec_total = sprint_t_total - t2;

        // Mirror of acceleration, coming to rest at D
        float d_accel      = 0.5f * a * t1 * t1;
        float d_cruise     = vm * sprint_t_cruise;
        float d_before_dec = d_accel + d_cruise;

        xdot_ref = dir * (vm - a * t_dec);
        x_ref    = sprint_x_start + dir * (d_before_dec + vm * t_dec - 0.5f * a * t_dec * t_dec);

        // Smoothly return to upright during braking
        const float theta_fwd_rad = 5.0f * (3.14159265f / 180.0f);
        float alpha = (t_dec_total > 0.0f) ? (1.0f - t_dec / t_dec_total) : 0.0f;
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;
        theta_ref     = theta_fwd_rad * alpha;
        theta_dot_ref = (theta_fwd_rad * (alpha - 1.0f)) / (t_dec_total > 0.0f ? t_dec_total : 1.0f);
      }
    }
  } else {
    // Stabilisation task: stabilise about hard-coded origin (x = 0)
    x_ref         = 0.0f;
    xdot_ref      = 0.0f;
    theta_ref     = 0.0f;
    theta_dot_ref = 0.0f;
    sprint_active = false;
  }

  x_target = x_ref;

  // Set desired speed based on control mode
  // LQR
  float state[4]  = { estimated_position, estimated_velocity, kalman.getTheta(), kalman.getThetaVelocity() };
  float target[4] = { x_ref, xdot_ref, theta_ref, theta_dot_ref };

  const float M_TOTAL = 1.515f;

  if (control_mode == 0) {
    lqr_force = lqr.compute(state, target);
  }
  else {
    lqr_force = pole.compute(state, target);
  }

  v_target += (lqr_force / M_TOTAL) * dt;
  v_target  = constrain(v_target, LQR_VELOCITY_MIN, LQR_VELOCITY_MAX);
  u_prev    = lqr_force;

  desired_speed = v_target;

  // Motor PIDs
  int16_t pid_fl = compute_pid_front_left (desired_speed, speeds[0], dt);
  int16_t pid_fr = compute_pid_front_right(desired_speed, speeds[1], dt);
  int16_t pid_bl = compute_pid_back_left  (desired_speed, speeds[2], dt);
  int16_t pid_br = compute_pid_back_right (desired_speed, speeds[3], dt);

  // Deadband compensation
  if (pid_fl > 5) pid_fl += 40; else if (pid_fl < -5) pid_fl -= 40;
  if (pid_fr > 5) pid_fr += 40; else if (pid_fr < -5) pid_fr -= 40;
  if (pid_bl > 5) pid_bl += 40; else if (pid_bl < -5) pid_bl -= 40;
  if (pid_br > 5) pid_br += 40; else if (pid_br < -5) pid_br -= 40;

  set_motor_speeds(pid_fl, pid_fr, pid_bl, pid_br);

  // Serial (teleplot, throttled 100ms)
  static unsigned long last_print = 0;
  static float dt_sum = 0.0f;
  static long  dt_n   = 0;
  dt_sum += dt * 1000.0f;
  dt_n++;

  if (current_time - last_print >= 100000) {
    last_print = current_time;
    float dt_cumavg = dt_sum / dt_n;
    Serial.println(">motor_encoders(FL, FR, BL, BR):"     + String(d1,                 3) + "," + String(d2,                 3) + "," + String(d3,                 3) + "," + String(d4,                 3));
    Serial.println(">desired:"            + String(desired_speed,      3));
    Serial.println(">actual:"             + String(speeds[0],          3));
    Serial.println(">estimated_velocity:" + String(estimated_velocity, 3));
    Serial.println(">pendulum_angle:"     + String(pendulum_encoder_angle, 3));
    Serial.println(">lqr_force:"          + String(lqr_force,          3));
    Serial.println(">kalman_theta:"       + String(kalman.getTheta(),  3));
    Serial.println(">kalman_v:"           + String(estimated_velocity, 3));
    Serial.println(">kalman_x:"           + String(estimated_position, 3));
    Serial.println(">x_ref:"              + String(x_ref,              3));
    Serial.println(">theta_ref:"          + String(theta_ref * (180.0f / 3.14159265f), 3));
    Serial.println(">sprint_active:"      + String(sprint_active ? 1 : 0));
    Serial.println(">pwm:"                + String((float)pid_fl / 1000.0f, 3));
    Serial.println(">dt_cumavg_ms:"       + String(dt_cumavg,          3));
    Serial.println("---");
  }
}