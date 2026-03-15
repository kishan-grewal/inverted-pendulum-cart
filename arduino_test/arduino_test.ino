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
LQRController lqr(-99.081f, -145.698f, -1020.569f, -128.843f);

float pendulum_encoder_angle = 0.0f;  // degrees, for Serial/display
#define CALIBRATION_OFFSET_DEG (0.0f)

#define AVG_SIZE 10

float dt = 0.0f;
unsigned long last_loop_time = 0;

const float LQR_VELOCITY_MAX =  1.0f;
const float LQR_VELOCITY_MIN = -1.0f;

static float v_target = 0.0f;
static float u_prev   = 0.0f;

int control_mode = 0; // 0 = LQR, 1 = PID
int task_mode = 0; //0 = stabilisation, 1 = sprint

float x_target = 0.0f;

// Sprint waypoint
#define SETTLE_WINDOW 500
const float WAYPOINT_STEP          = 0.1f;
const float WAYPOINT_STD_THRESHOLD = 0.05f;
const float SPRINT_TARGET          = 2.0f;
static float settle_buf[SETTLE_WINDOW] = {};
static int   settle_idx = 0;
static int   settle_n   = 0;

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

    unsigned long start_time = micros();

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

  last_loop_time = micros();
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
        cascaded_pid.reset();
        v_target = 0.0f;
        u_prev   = 0.0f;

        // reset sprint stuff
        x_target   = 0.0f;
        settle_n   = 0;
        settle_idx = 0;
        memset(settle_buf, 0, sizeof(settle_buf));

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

  // // Angle guard: only run controller if pendulum is near upright
  // if (fabsf(pendulum_encoder_angle) > 30.0f) {
  //   set_motor_speeds(0, 0, 0, 0);
  //   v_target = 0.0f;
  //   u_prev   = 0.0f;
  //   reset_motor_pids();
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

  // Set desired speed based on control mode
  if (control_mode == 0) {

    if (task_mode == 1 && x_target < SPRINT_TARGET) {
      float err = estimated_position - x_target;
      settle_buf[settle_idx] = err;
      settle_idx = (settle_idx + 1) % SETTLE_WINDOW;
      if (settle_n < SETTLE_WINDOW) settle_n++;

      if (settle_n == SETTLE_WINDOW) {
        float mean = 0.0f;
        for (int i = 0; i < SETTLE_WINDOW; i++) mean += settle_buf[i];
        mean /= SETTLE_WINDOW;

        float var = 0.0f;
        for (int i = 0; i < SETTLE_WINDOW; i++)
          var += (settle_buf[i] - mean) * (settle_buf[i] - mean);
        float std = sqrtf(var / SETTLE_WINDOW);

        if (std < WAYPOINT_STD_THRESHOLD) {
          x_target   = min(x_target + WAYPOINT_STEP, SPRINT_TARGET);
          settle_n   = 0;
          settle_idx = 0;
          memset(settle_buf, 0, sizeof(settle_buf));
        }
      }
    }

    float state[4]  = { estimated_position, estimated_velocity, kalman.getTheta(), kalman.getThetaVelocity() };
    float target[4] = { x_target, 0.0f, 0.0f, 0.0f };

    const float M_TOTAL = 1.515f;
    lqr_force = lqr.compute(state, target);
    v_target += (lqr_force / M_TOTAL) * dt;
    v_target  = constrain(v_target, LQR_VELOCITY_MIN, LQR_VELOCITY_MAX);
    u_prev    = lqr_force;

    desired_speed = v_target;

  } else if (control_mode == 1) {
    float pid_x_target = (task_mode == 1) ? SPRINT_TARGET : 0.0f;
    desired_speed = cascaded_pid.compute(kalman.getTheta(), kalman.getThetaVelocity(), estimated_position, pid_x_target, dt);
  }

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
    Serial.println(">motor_encoders(FL, FR, BL, BR):" + String(d1, 3) + "," + String(d2, 3) + "," + String(d3, 3) + "," + String(d4, 3));
    Serial.println(">desired:"            + String(desired_speed,      3));
    Serial.println(">actual:"             + String(speeds[0],          3));
    Serial.println(">estimated_velocity:" + String(estimated_velocity, 3));
    Serial.println(">pendulum_angle:"     + String(pendulum_encoder_angle, 3));
    Serial.println(">lqr_force:"          + String(lqr_force,          3));
    Serial.println(">kalman_theta:"       + String(kalman.getTheta(),  3));
    Serial.println(">kalman_v:"           + String(estimated_velocity, 3));
    Serial.println(">kalman_x:"           + String(estimated_position, 3));
    Serial.println(">pwm:"                + String((float)pid_fl / 1000.0f, 3));
    Serial.println(">dt_cumavg_ms:"       + String(dt_cumavg,          3));
    Serial.println(">x_target:" + String(x_target, 3));
    Serial.println("---");
  }
}