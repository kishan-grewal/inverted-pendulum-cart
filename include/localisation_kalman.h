#ifndef LOCALISATION_KALMAN_H
#define LOCALISATION_KALMAN_H

class LocalisationKalman {
  private:
    float x = 0.0f; 
    float v = 0.0f;

    float P00 = 1.0f, P01 = 0.0f;
    float P10 = 0.0f, P11 = 1.0f;

    float Q_pos = 0.01f; 
    float Q_vel = 0.1f;
    
    // Measurement noise for each individual encoder
    float R_pos = 0.05f; 

  public:
    void update(float z[], int num_sensors, float dt) {
      // 1. PREDICT step (Physics stays the same)
      x = x + (v * dt);
      P00 = P00 + dt * (P10 + P01) + dt * dt * P11 + Q_pos;
      P01 = P01 + dt * P11;
      P10 = P10 + dt * P11;
      P11 = P11 + Q_vel;

      // 2. FUSION UPDATE step
      // We calculate an average measurement (z_avg) or iterate the update.
      // For 4 identical encoders, the simplest robust way is to fuse the average:
      float z_combined = 0;
      for(int i = 0; i < num_sensors; i++) {
          z_combined += z[i];
      }
      float z_avg = z_combined / num_sensors;

      // Effective R decreases because we have more sensors!
      // R_eff = R / n
      float R_eff = R_pos / num_sensors; 

      // Standard Update with fused values
      float y = z_avg - x;
      float S = P00 + R_eff;
      float K0 = P00 / S;
      float K1 = P10 / S;

      x += K0 * y;
      v += K1 * y;

      P00 = (1 - K0) * P00;
      P01 = (1 - K0) * P01;
      P10 = -K1 * P00 + P10;
      P11 = -K1 * P01 + P11;
    }

    float getPosition() { return x; }
    float getVelocity() { return v; }
};

#endif