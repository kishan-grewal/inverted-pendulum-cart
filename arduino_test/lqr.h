#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <Arduino.h>

class LQRController {
  private:
    // Gain matrix K: [k1, k2, k3, k4]
    // Get these values from your Python script output: print(self.K)
    float K[4];
    float output_min;
    float output_max;
    bool limits_enabled;

  public:
    LQRController(float k1, float k2, float k3, float k4) {
      K[0] = k1;
      K[1] = k2;
      K[2] = k3;
      K[3] = k4;
      limits_enabled = false;
    }

    void setOutputLimits(float min_val, float max_val) {
      output_min = min_val;
      output_max = max_val;
      limits_enabled = true;
    }

    /**
     * @param state Current [x, x_dot, theta, theta_dot]
     * @param target Target [x, x_dot, theta, theta_dot]
     * @return Force/Control signal
     */
    float compute(float state[4], float target[4]) {
      float control_signal = 0;

      // Matrix multiplication: F = -K * (state - target)
      for (int i = 0; i < 4; i++) {
        control_signal -= K[i] * (state[i] - target[i]);
      }

      // Apply output limits (saturation)
      if (limits_enabled) {
        if (control_signal > output_max) control_signal = output_max;
        if (control_signal < output_min) control_signal = output_min;
      }

      return control_signal;
    }
};


uint32_t jerk(float angle){

  float K = 40000;  // 10deg becomes 0.4s = 400ms = 400,000us
  uint32_t jerk_time = K * fabs(angle);  // in micros()

  return jerk_time;
}

// NEW JERK CODE:
// One-shot angled-start helper.
// - Uses jerk(angle) to get pulse time (microseconds)
// - Applies fixed motor speed magnitude 700 for that one pulse
// - Blocks normal LQR until pendulum reaches +/-3 deg, then hands over
// Return true  -> angled-start handled this loop cycle (skip normal control)
// Return false -> continue normal LQR control
static inline bool runAngledStartOnce(float angle_deg, float &v_target, float &u_prev, bool reset_state = false) {
  static bool done = false;
  static bool pulse_started = false;
  static bool waiting_for_takeover = false;
  static unsigned long pulse_t0_us = 0;
  static uint32_t pulse_duration_us = 0;
  static int pulse_cmd = 0;

  const float TAKEOVER_DEG = 3.0f;
  const int FIXED_JERK_SPEED = 700;

  if (reset_state) {
    done = false;
    pulse_started = false;
    waiting_for_takeover = false;
    pulse_t0_us = 0;
    pulse_duration_us = 0;
    pulse_cmd = 0;
    return false;
  }

  if (done) return false;

  // LQR takeover as soon as angle is inside +/-3 degrees.
  if (fabsf(angle_deg) <= TAKEOVER_DEG) {
    done = true;
    set_motor_speeds(0, 0, 0, 0);
    reset_motor_pids();
    v_target = 0.0f;
    u_prev = 0.0f;
    return false;
  }

  // Start exactly one jerk pulse.
  if (!pulse_started) {
    pulse_started = true;
    pulse_t0_us = micros();
    pulse_duration_us = jerk(angle_deg);  // your jerk-time mapping
    pulse_cmd = (angle_deg >= 0.0f) ? FIXED_JERK_SPEED : -FIXED_JERK_SPEED;
  }

  // Apply fixed speed while pulse is active.
  if (!waiting_for_takeover) {
    if ((uint32_t)(micros() - pulse_t0_us) < pulse_duration_us) {
      set_motor_speeds(pulse_cmd, pulse_cmd, pulse_cmd, pulse_cmd);
      return true;
    }

    waiting_for_takeover = true;
    set_motor_speeds(0, 0, 0, 0);
  }

  // After pulse ends, wait until +/-3deg then hand over to LQR.
  return true;
}



#endif
