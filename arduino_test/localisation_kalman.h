#ifndef LOCALISATION_KALMAN_H
#define LOCALISATION_KALMAN_H

class LocalisationKalman {
  private:
    // State: [x, v, theta, theta_v]
    float x       = 0.0f;
    float v       = 0.0f;
    float theta   = 0.0f;
    float theta_v = 0.0f;

    // Predicted (a priori) state snapshot from last update() call
    float v_pred_last = 0.0f;

    float P[4][4] = {
      {1.0f, 0.0f, 0.0f, 0.0f},
      {0.0f, 1.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 1.0f, 0.0f},
      {0.0f, 0.0f, 0.0f, 1.0f}
    };

    // Physical parameters (from dynamics.py)
    static constexpr float M_cart_total  = 1.593f;
    static constexpr float m_rod   = 0.043f;
    static constexpr float m_tip   = 0.050f;
    static constexpr float M_cart  = M_cart_total - m_rod - m_tip; //0.828
    static constexpr float m_pend  = m_rod + m_tip;
    static constexpr float L_rod   = 0.6f;
    static constexpr float l_com   = (m_rod * L_rod / 2.0f + m_tip * L_rod) / m_pend;
    static constexpr float I_pivot = (1.0f/12.0f)*m_rod*L_rod*L_rod
                                   + m_rod*(L_rod/2.0f)*(L_rod/2.0f)
                                   + m_tip*L_rod*L_rod;
    static constexpr float b_x     = 0.1f;
    static constexpr float b_theta = 0.004320f;
    static constexpr float g       = 9.81f;
    static constexpr float M_t     = M_cart + m_pend;
    static constexpr float ml      = m_pend * l_com;
    static constexpr float D_denom = M_t * I_pivot - ml * ml;

    // Non-zero off-diagonal elements of linearised A matrix
    static constexpr float A21 = -(I_pivot * b_x) / D_denom;
    static constexpr float A23 = -(ml * ml * g)   / D_denom;
    static constexpr float A24 =  (ml * b_theta)  / D_denom;
    static constexpr float A42 =  (ml * b_x)      / D_denom;
    static constexpr float A43 =  (M_t * ml * g)  / D_denom;
    static constexpr float A44 = -(M_t * b_theta)  / D_denom;

    // Input matrix B elements
    static constexpr float B2  =  I_pivot / D_denom;
    static constexpr float B4  = -ml      / D_denom;

    // Process noise (diagonal): [x, v, theta, theta_v]
    // Increase Q[1] so velocity estimate tracks actual better (less sluggish, less attenuation)
    float Q[4] = {0.001f, 1.0f, 0.0001f, 0.001f};

    // Measurement noise
    float R_cart     = 0.05f;   // per wheel encoder [m]
    float R_pendulum = 0.005f;  // pendulum encoder [rad]
    float R_velocity = 0.15f;   // encoder-derived velocity [m/s] (reduces bias when cart at rest)

    // Sequential scalar Kalman update for state at index h_idx
    void scalarUpdate(int h_idx, float innov, float R_eff) {
      float S = P[h_idx][h_idx] + R_eff;
      float K[4];
      for (int i = 0; i < 4; i++) K[i] = P[i][h_idx] / S;

      x       += K[0] * innov;
      v       += K[1] * innov;
      theta   += K[2] * innov;
      theta_v += K[3] * innov;

      for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
          P[i][j] -= K[i] * P[h_idx][j];
    }

  public:
    // u:        control force on cart [N]
    // z_cart:   position from each wheel encoder [m] (integrate velocity before passing in)
    // num_cart: number of wheel encoders (4)
    // z_theta:  pendulum angle [rad], 0 = upright
    // dt:       time step [s]
    // z_velocity: encoder-derived cart velocity [m/s] (e.g. average of wheel speeds) – reduces velocity bias when cart at rest
    void update(float u, float z_cart[], int num_cart, float z_theta, float dt, float z_velocity) {

      // 1. PREDICT: x_new = x + A*x*dt + B*u*dt  (Euler, linearised about upright)
      float dx       =  v * dt;
      float dv       = (A21*v + A23*theta + A24*theta_v) * dt + B2*u*dt;
      float dtheta   =  theta_v * dt;
      float dtheta_v = (A42*v + A43*theta + A44*theta_v) * dt + B4*u*dt;

      x       += dx;
      v       += dv;
      theta   += dtheta;
      theta_v += dtheta_v;

      // Snapshot predicted velocity before measurement updates
      v_pred_last = v;

      // Propagate covariance: P = F*P*F' + Q, where F = I + A*dt
      float FP[4][4];
      for (int j = 0; j < 4; j++) {
        FP[0][j] = P[0][j] + dt * P[1][j];
        FP[1][j] = P[1][j] + dt * (A21*P[1][j] + A23*P[2][j] + A24*P[3][j]);
        FP[2][j] = P[2][j] + dt * P[3][j];
        FP[3][j] = P[3][j] + dt * (A42*P[1][j] + A43*P[2][j] + A44*P[3][j]);
      }
      for (int i = 0; i < 4; i++) {
        P[i][0] = FP[i][0] + dt * FP[i][1];
        P[i][1] = FP[i][1] + dt * (A21*FP[i][1] + A23*FP[i][2] + A24*FP[i][3]);
        P[i][2] = FP[i][2] + dt * FP[i][3];
        P[i][3] = FP[i][3] + dt * (A42*FP[i][1] + A43*FP[i][2] + A44*FP[i][3]);
      }
      for (int i = 0; i < 4; i++) P[i][i] += Q[i];

      // 2. UPDATE: cart position (H = [1,0,0,0])
      float z_avg = 0.0f;
      for (int i = 0; i < num_cart; i++) z_avg += z_cart[i];
      z_avg /= num_cart;
      scalarUpdate(0, z_avg - x, R_cart / num_cart);

      // 3. UPDATE: pendulum angle (H = [0,0,1,0])
      scalarUpdate(2, z_theta - theta, R_pendulum);

      // 4. UPDATE: velocity (H = [0,1,0,0]) – pulls estimate toward encoder speed, removes bias when cart at rest
      scalarUpdate(1, z_velocity - v, R_velocity);
    }

    float getPosition()      { return x; }
    float getVelocity()      { return v; }
    float getVelocityPred()  { return v_pred_last; }
    float getTheta()         { return theta; }
    float getThetaVelocity() { return theta_v; }
};

#endif
