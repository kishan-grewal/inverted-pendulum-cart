import numpy as np
import control

def calculate_lqr_gains():
    # Physical parameters — MUST match localisation_kalman.h exactly
    M_cart_total = 1.593
    m_rod   = 0.043
    m_tip   = 0.050
    m_pend  = m_rod + m_tip
    M_cart  = M_cart_total - m_rod - m_tip
    L_rod   = 0.6
    l_com   = (m_rod * L_rod / 2.0 + m_tip * L_rod) / m_pend
    I_pivot = (1.0/12.0)*m_rod*L_rod**2 + m_rod*(L_rod/2.0)**2 + m_tip*L_rod**2
    b_x     = 0.1
    b_theta = 0.004320
    g       = 9.81
    M_t     = M_cart_total
    ml      = m_pend * l_com
    D_denom = M_t * I_pivot - ml**2

    # Linearised A matrix (same as Kalman filter)
    A21 = -(I_pivot * b_x) / D_denom
    A23 = -(ml**2 * g) / D_denom
    A24 =  (ml * b_theta) / D_denom
    A42 =  (ml * b_x) / D_denom
    A43 =  (M_t * ml * g) / D_denom
    A44 = -(M_t * b_theta) / D_denom

    A = np.array([
        [0,   1,    0,    0   ],
        [0,   A21,  A23,  A24 ],
        [0,   0,    0,    1   ],
        [0,   A42,  A43,  A44 ]
    ])

    # Input matrix B — u is force [N]
    B2 =  I_pivot / D_denom
    B4 = -ml / D_denom

    B = np.array([
        [0  ],
        [B2 ],
        [0  ],
        [B4 ]
    ])

    # Bryson's rule
    x_max         = 0.15              # [m]
    x_dot_max     = 1.5               # [m/s]
    theta_max     = np.radians(1.0)   # [rad]
    theta_dot_max = np.radians(30.0)  # [rad/s]
    
    # Pole Placement real-value
    pole_real = -0.8

    F_max         = M_t * g           # [N] reference force
    R_multipler = 1.0  # Reduce R to increase control effort and speed up response (at the cost of more overshoot)

    Q = np.diag([
        1.0 / x_max**2,
        1.0 / x_dot_max**2,
        1.0 / theta_max**2,
        1.0 / theta_dot_max**2
    ])

    R = np.array([[1.0 / F_max**2]]) * R_multipler

    K, S, E = control.lqr(A, B, Q, R)

    from scipy.signal import place_poles
    desired_poles = [E[0], E[1], pole_real + E[2].imag * 1.0j, pole_real - E[2].imag * 1.0j] # Keep the original LQR poles for x and x_dot, but place the theta poles further left for faster response
    K_pp = place_poles(A, B, desired_poles).gain_matrix

    print(f"LQR closed loop poles: {E[0].real:.3f}, {E[1].real:.3f}, {E[2]:.3f}, {E[3]:.3f}")
    print(f"LQR:  {K[0,0]:.3f}f, {K[0,1]:.3f}f, {K[0,2]:.3f}f, {K[0,3]:.3f}f\n")
    print(f"POLE: {K_pp[0,0]:.3f}f, {K_pp[0,1]:.3f}f, {K_pp[0,2]:.3f}f, {K_pp[0,3]:.3f}f\n")
    print(f"LIMITS: x: {x_max}, xdot: {x_dot_max}, theta: {np.degrees(theta_max):.1f}deg, theta_dot: {np.degrees(theta_dot_max):.1f}deg, F: {F_max:.3f}N")
    
    # print("-" * 55)
    # print(" LQR GAINS (force output, matches Kalman filter dynamics)")
    # print("-" * 55)
    # print(f"  M_cart:   {M_cart:.4f} kg")
    # print(f"  m_pend:   {m_pend:.4f} kg")
    # print(f"  M_t:      {M_t:.4f} kg")
    # print(f"  l_com:    {l_com:.4f} m")
    # print(f"  I_pivot:  {I_pivot:.6f} kg.m^2")
    # print(f"  D_denom:  {D_denom:.6f}")
    # print(f"  b_x:      {b_x}")
    # print(f"  b_theta:  {b_theta}")
    # print("-" * 55)
    # print("A matrix:")
    # print(A)
    # print("B matrix:")
    # print(B.T)
    # print("-" * 55)
    # print("LQR gains K (F = -K * [x, x_dot, theta, theta_dot]):")
    # print(f"  k1 (x):         {K[0,0]:>12.6f}")
    # print(f"  k2 (x_dot):     {K[0,1]:>12.6f}")
    # print(f"  k3 (theta):     {K[0,2]:>12.6f}")
    # print(f"  k4 (theta_dot): {K[0,3]:>12.6f}")
    # print("-" * 55)
    # print("Closed-loop poles:")
    # for p in E:
    #     print(f"  {p}")
    # print("-" * 55)
    # print("ARDUINO USAGE:")
    # print(f"  LQRController lqr({K[0,0]:.6f}, {K[0,1]:.6f}, {K[0,2]:.6f}, {K[0,3]:.6f});")
    # print("  float force = lqr.compute(state, target);  // Newtons")
    # print(f"  float accel = force / {M_t:.4f};             // m/s^2")
    # print("  desired_speed += accel * dt;")
    # print("  desired_speed = constrain(desired_speed, -MAX_SPEED, MAX_SPEED);")

if __name__ == "__main__":
    calculate_lqr_gains()