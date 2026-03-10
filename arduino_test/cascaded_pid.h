#ifndef CASCADED_PID_H
#define CASCADED_PID_H

#include <Arduino.h>

class CascadedPID {
private:
    // Outer loop: angle → desired velocity
    float Kp_theta;
    float Ki_theta;
    float Kd_theta;

    // Position correction: position error → desired velocity (slow, for sprint/drift)
    float Kp_x;

    // Outer loop state
    float integral_theta = 0.0f;
    float last_error_theta = 0.0f;

    // Limits
    float v_max;
    float integral_limit;

public:
    CascadedPID(float kp_t, float ki_t, float kd_t, float kp_x, float v_max, float integral_limit)
        : Kp_theta(kp_t), Ki_theta(ki_t), Kd_theta(kd_t), Kp_x(kp_x),
          v_max(v_max), integral_limit(integral_limit) {}

    /**
     * Compute desired cart velocity from pendulum angle and cart position.
     * Pass the result into your existing motor PID as desired_speed.
     *
     * @param theta       Pendulum angle [rad], 0 = upright
     * @param theta_dot   Pendulum angular velocity [rad/s]
     * @param x           Cart position [m]
     * @param x_target    Desired cart position [m] (0 for balance, 2.0 for sprint)
     * @param dt          Time step [s]
     * @return            Desired cart velocity [m/s]
     */
    float compute(float theta, float theta_dot, float x, float x_target, float dt) {
        // --- Outer loop: angle PID ---
        float error_theta = 0.0f - theta;  // target is upright (theta = 0)

        integral_theta += error_theta * dt;
        integral_theta = constrain(integral_theta, -integral_limit, integral_limit);

        // Use theta_dot directly as derivative (cleaner than differencing)
        float derivative_theta = -theta_dot;

        float v_desired = Kp_theta * error_theta
                        + Ki_theta * integral_theta
                        + Kd_theta * derivative_theta;

        // --- Position correction (slow outer-outer loop) ---
        float error_x = x_target - x;
        v_desired += Kp_x * error_x;

        // Clamp output
        v_desired = constrain(v_desired, -v_max, v_max);

        return v_desired;
    }

    void reset() {
        integral_theta = 0.0f;
        last_error_theta = 0.0f;
    }
};

#endif