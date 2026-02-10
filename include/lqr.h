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

#endif