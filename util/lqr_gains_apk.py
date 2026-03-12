import numpy as np
import control

def calculate_lqr_gains():
    # Physical parameters — MUST match localisation_kalman.h exactly
    M_cart_total = 1.515
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
    x_dot_max     = 0.5               # [m/s]
    theta_max     = np.radians(1.0)   # [rad]
    theta_dot_max = np.radians(30.0)  # [rad/s]
    F_max         = M_t * g           # [N] reference force
    R_multipler = 1.0  # Reduce R to increase control effort and speed up response

    Q = np.diag([
        1.0 / x_max**2,
        1.0 / x_dot_max**2,
        1.0 / theta_max**2,
        1.0 / theta_dot_max**2
    ])

    R = np.array([[1.0 / F_max**2]]) * R_multipler

    # 1. ORIGINAL CONTINUOUS TIME LQR
    K, S, E = control.lqr(A, B, Q, R)

    # ---------------------------------------------------------
    # 2. NEW DISCRETE TIME LQR
    # ---------------------------------------------------------
    dt = 0.002  # <--- Change this to match your Arduino loop time (e.g., 0.01 for 100Hz) # 500hz
    
    # Create continuous state-space system
    sys_cont = control.StateSpace(A, B, np.eye(4), np.zeros((4,1)))
    
    # Convert to discrete using Zero-Order Hold (ZOH)
    sys_disc = sys_cont.sample(dt, method='zoh')
    Ad = sys_disc.A
    Bd = sys_disc.B

    # Calculate Discrete LQR
    K_disc, S_disc, E_disc = control.dlqr(Ad, Bd, Q, R)
    # ---------------------------------------------------------

    # 3. POLE PLACEMENT (Based on Continuous)
    from scipy.signal import place_poles
    desired_poles = [E[0], E[1], -2.0 + 1.0j, -2.0 - 1.0j] 
    K_pp = place_poles(A, B, desired_poles).gain_matrix

    print(f"LIMITS: x: {x_max}, xdot: {x_dot_max}, theta: {np.degrees(theta_max):.1f}deg, theta_dot: {np.degrees(theta_dot_max):.1f}deg, F: {F_max:.3f}N\n")
    
    print(f"closed loop poles (Cont): {E[0].real:.3f}, {E[1].real:.3f}, {E[2]:.3f}, {E[3]:.3f}\n")
    
    print("--- GAIN COMPARISON ---")
    print(f"LQR Continuous: {K[0,0]:.3f}, {K[0,1]:.3f}, {K[0,2]:.3f}, {K[0,3]:.3f}")
    print(f"LQR Discrete:   {K_disc[0,0]:.3f}, {K_disc[0,1]:.3f}, {K_disc[0,2]:.3f}, {K_disc[0,3]:.3f}  (using dt={dt}s)")
    print(f"POLE Cont:      {K_pp[0,0]:.3f}, {K_pp[0,1]:.3f}, {K_pp[0,2]:.3f}, {K_pp[0,3]:.3f}\n")
    
    # print("-" * 55)
    # print(" LQR GAINS (force output, matches Kalman filter dynamics)")
    # print("-" * 55)
    # print(f"  M_cart:   {M_cart:.4f} kg")
    # print(f"  m_pend:   {m_pend:.4f} kg")
    # ... (rest of your original print block remains untouched) ...

if __name__ == "__main__":
    calculate_lqr_gains()