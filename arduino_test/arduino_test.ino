// #include <Arduino.h>
#include <math.h>
#include "drive.h"
#include "pendulum_encoder.h"
#include "motor_encoder.h"
#include "motor_pid.h"
#include "lqr.h"
#include "localisation_kalman.h"

#define START_BUTTON_PIN 10
#define CONTROL_SELECT_BUTTON_PIN 12
#define TASK_SELECT_BUTTON_PIN 13

const float DEG_TO_RAD = 3.14159265f / 180.0f;
const float RAD_TO_DEG = 180.0f / 3.14159265f;

// ============================================================
//  ANGLED-START JERK TUNING PARAMETERS
//  Task mode 3: manually tilt pendulum, press start.
//  Cart fires a single open-loop pulse to kick it upright,
//  then LQR stabilise takes over automatically.
// ============================================================
const float        JERK_K                 = 400.0f;   // motor_cmd = K * init_angle_rad  [tune first]
const unsigned long JERK_DURATION_MS      = 400UL;    // open-loop pulse width [ms]       [tune second]
const float        JERK_CAPTURE_ANGLE_DEG = 3.0f;     // LQR takes over inside +/- this   [tune last]
const float        JERK_CAPTURE_ANGLE_RAD = JERK_CAPTURE_ANGLE_DEG * DEG_TO_RAD;
// NOTE: if cart moves the WRONG direction on pulse, negate JERK_K.
// ============================================================

LocalisationKalman kalman;
LQRController lqr_stabilise(-104.182f, -153.199f, -1073.178f, -135.522f);
LQRController pole_stabilise(-105.135f, -154.459f, -1074.500f, -136.213f);
LQRController lqr_recovery(-104.182f, -153.199f, -1073.178f, -135.522f);
LQRController pole_recovery(-105.135f, -154.459f, -1074.500f, -136.213f);

LQRController lqr_sprint(-104.182f, -133.052f, -791.064f, -120.821f);
LQRController pole_sprint(-111.100f, -140.426f, -800.810f, -124.891f);

const float LQR_FORCE_LIMIT          = 15.627f;
const float LQR_FORCE_LIMIT_RECOVERY = 15.627f;

const float RECOVERY_SWITCH_ANGLE_DEG = 3.0f;
const float RECOVERY_SWITCH_ANGLE_RAD = RECOVERY_SWITCH_ANGLE_DEG * DEG_TO_RAD;

const bool     ENABLE_OPEN_LOOP_RECOVERY_TEST = true;
const int16_t  RECOVERY_TEST_MOTOR_SPEED      = 800;
const unsigned long RECOVERY_TEST_DURATION_MS = 120UL;
const bool     RECOVERY_TEST_INVERT_DIRECTION = false;

float pendulum_encoder_angle = 0.0f;
#define CALIBRATION_OFFSET_DEG (0.0f)

#define AVG_SIZE 10

const float DEADBAND_LIMIT        = 1.0f;
const float DEADBAND_COMPENSATION = 30.0f;

float dt = 0.0f;
unsigned long last_loop_time = 0;

const float LQR_VELOCITY_MAX =  1.25f;
const float LQR_VELOCITY_MIN = -1.25f;

static float v_target = 0.0f;
static float u_prev   = 0.0f;

int control_mode = 0; // 0 = LQR, 1 = POLE
int task_mode    = 0; // 0 = stabilisation, 1 = sprint, 2 = recovery, 3 = angled start

unsigned long start_time_us = 0;
float x_final  = 0.0f;
float x_target = 0.0f;

// Sprint parameters
const float SPRINT_DISTANCE_M  = 2.01f;
const float SPRINT_A_MAX       = 0.09f;
const float SPRINT_V_MAX       = 1.5f;
const float SPRINT_LINEAR_V_SCALE = 0.5f;

float sprint_t_accel  = 0.0f;
float sprint_t_cruise = 0.0f;
float sprint_t_total  = 0.0f;

bool  sprint_active    = false;
float sprint_x_start   = 0.0f;
bool  sprint_completed = false;

// Recovery test state
bool          recovery_test_pulse_active   = false;
bool          recovery_test_pulse_done     = false;
unsigned long recovery_test_pulse_start_us = 0;
int16_t       recovery_test_motor_command  = 0;

void reset_recovery_test_state() {
  recovery_test_pulse_active   = false;
  recovery_test_pulse_done     = false;
  recovery_test_pulse_start_us = 0;
  recovery_test_motor_command  = 0;
}

int16_t get_recovery_test_motor_command(float theta_rad) {
  int16_t cmd = (theta_rad >= 0.0f) ? RECOVERY_TEST_MOTOR_SPEED : -RECOVERY_TEST_MOTOR_SPEED;
  if (RECOVERY_TEST_INVERT_DIRECTION) cmd = -cmd;
  return cmd;
}

// ============================================================
//  ANGLED-START JERK STATE MACHINE
// ============================================================
enum JerkState { JERK_IDLE, JERK_PULSE, JERK_WAITING, JERK_DONE };
JerkState     jerk_state            = JERK_IDLE;
float         jerk_init_angle_rad   = 0.0f;
int16_t       jerk_motor_cmd        = 0;
unsigned long jerk_pulse_start_us   = 0;

void reset_jerk_state() {
  jerk_state          = JERK_IDLE;
  jerk_init_angle_rad = 0.0f;
  jerk_motor_cmd      = 0;
  jerk_pulse_start_us = 0;
}
// ============================================================

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

  lqr_stabilise.setOutputLimits(-LQR_FORCE_LIMIT,          LQR_FORCE_LIMIT);
  pole_stabilise.setOutputLimits(-LQR_FORCE_LIMIT,          LQR_FORCE_LIMIT);
  lqr_recovery.setOutputLimits( -LQR_FORCE_LIMIT_RECOVERY,  LQR_FORCE_LIMIT_RECOVERY);
  pole_recovery.setOutputLimits( -LQR_FORCE_LIMIT_RECOVERY,  LQR_FORCE_LIMIT_RECOVERY);
  lqr_sprint.setOutputLimits(   -LQR_FORCE_LIMIT,          LQR_FORCE_LIMIT);
  pole_sprint.setOutputLimits(  -LQR_FORCE_LIMIT,          LQR_FORCE_LIMIT);

  pinMode(START_BUTTON_PIN,          INPUT_PULLUP);
  pinMode(CONTROL_SELECT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(TASK_SELECT_BUTTON_PIN,    INPUT_PULLUP);

  Serial.println("Setup complete. Waiting for start button...");

  while (digitalRead(START_BUTTON_PIN) == HIGH) {
    static bool control_mode_changed = false;
    static bool task_mode_changed    = false;

    if (digitalRead(CONTROL_SELECT_BUTTON_PIN) == HIGH) control_mode_changed = false;
    if (digitalRead(TASK_SELECT_BUTTON_PIN)    == HIGH) task_mode_changed    = false;

    start_time_us = micros();

    while (digitalRead(CONTROL_SELECT_BUTTON_PIN) == LOW) {
      if ((micros() - start_time_us > 200000) && !control_mode_changed) {
        control_mode = (control_mode + 1) % 2;
        control_mode_changed = true;
        Serial.print("Control mode: ");
        Serial.println(control_mode == 0 ? "LQR" : "POLE");
        break;
      }
    }

    while (digitalRead(TASK_SELECT_BUTTON_PIN) == LOW) {
      if ((micros() - start_time_us > 200000) && !task_mode_changed) {
        task_mode = (task_mode + 1) % 4;   // 0-3
        task_mode_changed = true;
        Serial.print("Task mode: ");
        switch (task_mode) {
          case 0: Serial.println("Stabilisation");  break;
          case 1: Serial.println("Sprint");         break;
          case 2: Serial.println("Recovery");       break;
          case 3: Serial.println("Angled Start");   break;
          default: Serial.println("Unknown");       break;
        }
        break;
      }
    }
  }

  delay(1000);

  if (task_mode == 1) x_final = 2.0f;

  // Pre-compute sprint timings
  {
    const float D   = fabsf(SPRINT_DISTANCE_M);
    const float a   = SPRINT_A_MAX;
    const float vm  = SPRINT_V_MAX;
    float t_accel   = vm / a;
    float d_accel   = 0.5f * a * t_accel * t_accel;

    if (2.0f * d_accel >= D) {
      t_accel        = sqrtf(D / a);
      sprint_t_accel = t_accel;
      sprint_t_cruise= 0.0f;
      sprint_t_total = 2.0f * t_accel;
    } else {
      float d_cruise  = D - 2.0f * d_accel;
      float t_cruise  = d_cruise / vm;
      sprint_t_accel  = t_accel;
      sprint_t_cruise = t_cruise;
      sprint_t_total  = 2.0f * t_accel + t_cruise;
    }
  }

  start_time_us  = micros();
  last_loop_time = micros();
}

void loop() {

  // Pause / resume
  if (digitalRead(START_BUTTON_PIN) == LOW) {
    unsigned long pause_start = micros();

    while (digitalRead(START_BUTTON_PIN) == LOW) {
      if ((micros() - pause_start) >= 20000) {
        Serial.println("Paused");

        while (digitalRead(START_BUTTON_PIN) == HIGH) delay(5);

        unsigned long hold_start = micros();
        while (true) {
          if (digitalRead(START_BUTTON_PIN) == HIGH) hold_start = micros();
          else if ((micros() - hold_start) >= 200000) break;
          delay(5);
        }

        reset_motor_pids();
        last_loop_time = micros();
        start_time_us  = last_loop_time;
        v_target       = 0.0f;
        u_prev         = 0.0f;
        sprint_active    = false;
        sprint_completed = false;
        reset_recovery_test_state();
        reset_jerk_state();   // re-arm jerk on resume

        Serial.println("Resumed");

        while (digitalRead(START_BUTTON_PIN) == LOW) delay(5);
        break;
      }
    }
  }

  float angle_rad = get_pendulum_angle_rad() + (CALIBRATION_OFFSET_DEG * DEG_TO_RAD);
  pendulum_encoder_angle = angle_rad * RAD_TO_DEG;

  unsigned long current_time = micros();
  dt = (current_time - last_loop_time) / 1000000.0f;
  last_loop_time = current_time;
  if (dt > 0.1f || dt <= 0.0f) dt = 0.001f;

  float d1, d2, d3, d4;
  encoders_getDistance(&d1, &d2, &d3, &d4);

  static float prev_d[4] = {0, 0, 0, 0};
  float dists[4]  = {d1, d2, d3, d4};
  float speeds[4];
  float raw_speeds[4];

  static float speed_buf[4][AVG_SIZE] = {};
  static int   buf_idx = 0;

  for (int i = 0; i < 4; i++) {
    float raw       = (dt > 0.001f) ? (dists[i] - prev_d[i]) / dt : 0.0f;
    raw_speeds[i]   = raw;
    prev_d[i]       = dists[i];
    speed_buf[i][buf_idx] = raw;

    float sum = 0.0f;
    for (int j = 0; j < AVG_SIZE; j++) sum += speed_buf[i][j];
    speeds[i] = sum / AVG_SIZE;
  }
  buf_idx = (buf_idx + 1) % AVG_SIZE;

  // Angle guard — kill everything if fallen
  if (fabsf(pendulum_encoder_angle) > 50.0f) {
    set_motor_speeds(0, 0, 0, 0);
    v_target = 0.0f;
    u_prev   = 0.0f;
    reset_motor_pids();
    sprint_active    = false;
    sprint_completed = false;
    reset_recovery_test_state();
    reset_jerk_state();
    return;
  }

  float z_cart[4]    = {d1, d2, d3, d4};
  float z_velocity   = (speeds[0] + speeds[1] + speeds[2] + speeds[3]) / 4.0f;
  kalman.update(u_prev, z_cart, 4, angle_rad, dt, z_velocity);

  float estimated_position = kalman.getPosition();
  float estimated_velocity = kalman.getVelocity();

  // ============================================================
  //  TASK MODE 3 — ANGLED-START JERK
  //  Fires once on entry; hands to LQR stabilise when captured.
  // ============================================================
  if (task_mode == 3) {

    if (jerk_state == JERK_IDLE) {
      // Capture resting angle and compute motor command
      jerk_init_angle_rad = kalman.getTheta();
      jerk_motor_cmd      = (int16_t)(JERK_K * jerk_init_angle_rad);
      jerk_pulse_start_us = current_time;
      jerk_state          = JERK_PULSE;

      Serial.print("Jerk pulse fired.  init_angle_deg=");
      Serial.print(jerk_init_angle_rad * RAD_TO_DEG, 2);
      Serial.print("  motor_cmd=");
      Serial.println(jerk_motor_cmd);
    }

    if (jerk_state == JERK_PULSE) {
      unsigned long elapsed = current_time - jerk_pulse_start_us;

      if (elapsed < JERK_DURATION_MS * 1000UL) {
        // Open-loop pulse running — bypass Kalman/PID
        v_target = 0.0f;
        u_prev   = 0.0f;
        reset_motor_pids();
        set_motor_speed(jerk_motor_cmd);
        return;
      }

      // Pulse finished
      set_motor_speeds(0, 0, 0, 0);
      jerk_state = JERK_WAITING;
      Serial.println("Jerk pulse done. Waiting for capture window...");
    }

    if (jerk_state == JERK_WAITING) {
      if (fabsf(kalman.getTheta()) <= JERK_CAPTURE_ANGLE_RAD) {
        // Pendulum inside capture window — hand to LQR
        // Re-seed v_target from current encoder average so first LQR frame isn't blind
        v_target = (speeds[0] + speeds[1] + speeds[2] + speeds[3]) / 4.0f;
        u_prev   = 0.0f;
        reset_motor_pids();
        jerk_state = JERK_DONE;
        Serial.println("Captured! LQR stabilise active.");
      } else {
        // Still waiting — hold motors off
        set_motor_speeds(0, 0, 0, 0);
        v_target = 0.0f;
        u_prev   = 0.0f;
        return;
      }
    }

    // JERK_DONE: fall through to LQR stabilise below
  }
  // ============================================================

  float desired_speed = 0.0f;
  float lqr_force     = 0.0f;

  float x_ref       = 0.0f;
  float xdot_ref    = 0.0f;
  float theta_ref   = 0.0f;
  float theta_dot_ref = 0.0f;

  if (task_mode == 1) {
    if (!sprint_active && !sprint_completed) {
      sprint_active  = true;
      sprint_x_start = estimated_position;
      start_time_us  = current_time;
    }

    float t = (current_time - start_time_us) / 1000000.0f;
    const float dir   = (SPRINT_DISTANCE_M >= 0.0f) ? 1.0f : -1.0f;
    const float D_abs = fabsf(SPRINT_DISTANCE_M);
    float v_scale     = (SPRINT_LINEAR_V_SCALE > 0.0f) ? SPRINT_LINEAR_V_SCALE : 1.0f;
    float t_total_linear = (sprint_t_total > 0.0f) ? (sprint_t_total / v_scale) : 0.0f;

    if (t_total_linear <= 0.0f || t >= t_total_linear) {
      x_ref          = sprint_x_start + dir * D_abs;
      xdot_ref       = 0.0f;
      sprint_active  = false;
      sprint_completed = true;
    } else {
      float s = constrain(t / t_total_linear, 0.0f, 1.0f);
      x_ref    = sprint_x_start + dir * D_abs * s;
      xdot_ref = (dir * D_abs) / t_total_linear;
    }
  } else {
    // All other modes (including angled start after capture): stabilise at origin
    x_ref          = 0.0f;
    xdot_ref       = 0.0f;
    sprint_active  = false;
    sprint_completed = false;
  }

  x_target = x_ref;

  float state[4]  = { estimated_position, estimated_velocity, kalman.getTheta(), kalman.getThetaVelocity() };
  float target[4] = { x_ref, xdot_ref, theta_ref, theta_dot_ref };

  const float M_TOTAL = 1.593f;

  // Recovery test open-loop (task mode 2 only)
  bool    recovery_test_bypassing_closed_loop = false;
  bool    recovery_test_waiting_for_capture   = false;
  int16_t open_loop_motor_command             = 0;

  if (task_mode == 2 && ENABLE_OPEN_LOOP_RECOVERY_TEST) {
    const float theta_abs_rad = fabsf(kalman.getTheta());

    if (theta_abs_rad > RECOVERY_SWITCH_ANGLE_RAD) {
      if (!recovery_test_pulse_done) {
        if (!recovery_test_pulse_active) {
          recovery_test_pulse_active   = true;
          recovery_test_pulse_start_us = current_time;
          recovery_test_motor_command  = get_recovery_test_motor_command(kalman.getTheta());
          Serial.print("Recovery pulse started at motor speed ");
          Serial.println(recovery_test_motor_command);
        }

        unsigned long elapsed_us  = current_time - recovery_test_pulse_start_us;
        unsigned long duration_us = RECOVERY_TEST_DURATION_MS * 1000UL;

        if (elapsed_us < duration_us) {
          recovery_test_bypassing_closed_loop = true;
          open_loop_motor_command = recovery_test_motor_command;
        } else {
          recovery_test_pulse_active  = false;
          recovery_test_pulse_done    = true;
          recovery_test_motor_command = 0;
          Serial.println("Recovery pulse finished. Waiting to re-enter stabilise window.");
        }
      }

      if (recovery_test_pulse_done) {
        recovery_test_bypassing_closed_loop = true;
        recovery_test_waiting_for_capture   = true;
        open_loop_motor_command = 0;
      }

    } else if (recovery_test_pulse_active || recovery_test_pulse_done) {
      Serial.println("Recovery angle back inside capture window. Switching to stabilise.");
      reset_recovery_test_state();
    }
  } else if (task_mode != 2) {
    reset_recovery_test_state();
  }

  int16_t pid_fl = 0, pid_fr = 0, pid_bl = 0, pid_br = 0;

  if (!recovery_test_bypassing_closed_loop) {
    // Select LQR gains
    if (task_mode == 0 || task_mode == 3) {
      lqr_force = (control_mode == 0)
        ? lqr_stabilise.compute(state, target)
        : pole_stabilise.compute(state, target);
    }
    else if (task_mode == 1) {
      lqr_force = (control_mode == 0)
        ? lqr_sprint.compute(state, target)
        : pole_sprint.compute(state, target);
    }
    else if (task_mode == 2) {
      if (!ENABLE_OPEN_LOOP_RECOVERY_TEST && fabsf(kalman.getTheta()) > RECOVERY_SWITCH_ANGLE_RAD) {
        lqr_force = (control_mode == 0)
          ? lqr_recovery.compute(state, target)
          : pole_recovery.compute(state, target);
      } else {
        lqr_force = (control_mode == 0)
          ? lqr_stabilise.compute(state, target)
          : pole_stabilise.compute(state, target);
      }
    }

    v_target += (lqr_force / M_TOTAL) * dt;
    v_target  = constrain(v_target, LQR_VELOCITY_MIN, LQR_VELOCITY_MAX);
    u_prev    = lqr_force;

    desired_speed = v_target;

    pid_fl = compute_pid_front_left (desired_speed, speeds[0], dt);
    pid_fr = compute_pid_front_right(desired_speed, speeds[1], dt);
    pid_bl = compute_pid_back_left  (desired_speed, speeds[2], dt);
    pid_br = compute_pid_back_right (desired_speed, speeds[3], dt);

    if (pid_fl >  DEADBAND_LIMIT) pid_fl += DEADBAND_COMPENSATION;
    else if (pid_fl < -DEADBAND_LIMIT) pid_fl -= DEADBAND_COMPENSATION;
    if (pid_fr >  DEADBAND_LIMIT) pid_fr += DEADBAND_COMPENSATION;
    else if (pid_fr < -DEADBAND_LIMIT) pid_fr -= DEADBAND_COMPENSATION;
    if (pid_bl >  DEADBAND_LIMIT) pid_bl += DEADBAND_COMPENSATION;
    else if (pid_bl < -DEADBAND_LIMIT) pid_bl -= DEADBAND_COMPENSATION;
    if (pid_br >  DEADBAND_LIMIT) pid_br += DEADBAND_COMPENSATION;
    else if (pid_br < -DEADBAND_LIMIT) pid_br -= DEADBAND_COMPENSATION;

    set_motor_speeds(pid_fl, pid_fr, pid_bl, pid_br);

  } else {
    desired_speed = 0.0f;
    lqr_force     = 0.0f;
    v_target      = 0.0f;
    u_prev        = 0.0f;
    reset_motor_pids();
    set_motor_speed(open_loop_motor_command);
  }

  // Serial telemetry (Teleplot, throttled 100ms)
  static unsigned long last_print = 0;
  static float dt_sum = 0.0f;
  static long  dt_n   = 0;
  dt_sum += dt * 1000.0f;
  dt_n++;

  if (current_time - last_print >= 100000) {
    last_print  = current_time;
    float dt_cumavg = dt_sum / dt_n;

    Serial.println(">motor_encoders(FL,FR,BL,BR):"    + String(d1,3)+","+String(d2,3)+","+String(d3,3)+","+String(d4,3));
    Serial.println(">motor_speeds(FL,FR,BL,BR):"      + String(speeds[0],3)+","+String(speeds[1],3)+","+String(speeds[2],3)+","+String(speeds[3],3));
    Serial.println(">pid_outputs(FL,FR,BL,BR):"       + String(pid_fl)+","+String(pid_fr)+","+String(pid_bl)+","+String(pid_br));
    Serial.println(">desired:"            + String(desired_speed,      3));
    Serial.println(">actual:"             + String(speeds[0],          3));
    Serial.println(">raw_v1:"             + String(raw_speeds[0],      3));
    Serial.println(">estimated_velocity:" + String(estimated_velocity, 3));
    Serial.println(">kalman_v_pred:"      + String(kalman.getVelocityPred(), 3));
    Serial.println(">pendulum_angle:"     + String(pendulum_encoder_angle, 3));
    Serial.println(">lqr_force:"          + String(lqr_force,          3));
    Serial.println(">kalman_theta:"       + String(kalman.getTheta(),  3));
    Serial.println(">kalman_v:"           + String(estimated_velocity, 3));
    Serial.println(">kalman_x:"           + String(estimated_position, 3));
    Serial.println(">x_ref:"              + String(x_ref,              3));
    Serial.println(">theta_ref:"          + String(theta_ref * RAD_TO_DEG, 3));
    Serial.println(">sprint_active:"      + String(sprint_active ? 1 : 0));
    Serial.println(">recovery_test_active:"  + String(recovery_test_pulse_active ? 1 : 0));
    Serial.println(">recovery_test_waiting:" + String(recovery_test_waiting_for_capture ? 1 : 0));
    Serial.println(">recovery_test_cmd:"     + String(open_loop_motor_command));
    // Jerk telemetry
    Serial.println(">jerk_state:"         + String((int)jerk_state));
    Serial.println(">jerk_init_angle_deg:"+ String(jerk_init_angle_rad * RAD_TO_DEG, 2));
    Serial.println(">jerk_motor_cmd:"     + String(jerk_motor_cmd));
    Serial.println(">pwm:"                + String((float)pid_fl / 1000.0f, 3));
    Serial.println(">dt_cumavg_ms:"       + String(dt_cumavg, 3));
    Serial.println("---");
  }
}
