#ifndef MOTOR_KALMAN_H
#define MOTOR_KALMAN_H

class MotorKalman {
  private:
    // State: [position, velocity]
    float x = 0.0f; 
    float v = 0.0f;

    // Covariance matrix P
    float P00 = 1.0f, P01 = 0.0f;
    float P10 = 0.0f, P11 = 1.0f;

    // Process Noise (Q) - How much we trust our "physics" model
    // Increase these if the motor accelerates very fast
    float Q_pos = 0.01f; 
    float Q_vel = 0.1f;

    // Measurement Noise (R) - How much we trust the encoder
    // Encoders are precise, so this is usually small
    float R_pos = 0.05f; 

  public:
    /**
     * @param z Measured position from encoder (in your preferred units, e.g. degrees or radians)
     * @param dt Time elapsed since last update in seconds
     */
    void update(float z, float dt) {
      // 1. PREDICT step
      // x = x + v * dt
      x = x + (v * dt);
      // P = A*P*A' + Q
      P00 = P00 + dt * (P10 + P01) + dt * dt * P11 + Q_pos;
      P01 = P01 + dt * P11;
      P10 = P10 + dt * P11;
      P11 = P11 + Q_vel;

      // 2. UPDATE step
      // Innovation (Error)
      float y = z - x;
      // Innovation Covariance
      float S = P00 + R_pos;
      // Kalman Gain
      float K0 = P00 / S;
      float K1 = P10 / S;

      // Update State
      x += K0 * y;
      v += K1 * y;

      // Update Covariance
      P00 = (1 - K0) * P00;
      P01 = (1 - K0) * P01;
      P10 = -K1 * P00 + P10;
      P11 = -K1 * P01 + P11;
    }

    float getPosition() { return x; }
    float getVelocity() { return v; }
};

#endif