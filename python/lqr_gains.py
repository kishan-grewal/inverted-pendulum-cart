import numpy as np
import control

def calculate_lqr_gains():
    # 1. PHYSICAL PARAMETERS
    M_TOTAL = 1.515
    L_rod     = 0.6
    m_rod     = 0.043
    m_tip     = 0.050
    m_pend    = m_rod + m_tip
    m_cart = M_TOTAL - m_pend
    g         = 9.81

    l = (m_rod * L_rod / 2 + m_tip * L_rod) / m_pend
    I_pivot = (1/3)*m_rod*L_rod**2 + m_tip*L_rod**2
    b_theta = 0.001       # [N.m.s/rad]

    # 2. STATE-SPACE (ACCELERATION INPUT)
    A = np.array([
        [0, 1, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 1],
        [0, 0, (m_pend*g*l)/I_pivot, -b_theta/I_pivot]
    ])

    B = np.array([
        [0],
        [1],
        [0],
        [-(m_pend*l)/I_pivot]
    ])

    # 3. BRYSON'S RULE
    x_max         = 0.10              # [m]
    x_dot_max     = 1.5               # [m/s]
    theta_max     = np.radians(3.0)   # [rad]
    theta_dot_max = np.radians(30.0)  # [rad/s]
    u_max         = 3.0               # [m/s^2]

    Q = np.diag([
        1.0 / x_max**2,
        1.0 / x_dot_max**2,
        1.0 / theta_max**2,
        1.0 / theta_dot_max**2
    ])

    R = np.array([[1.0 / u_max**2]])

    K, S, E = control.lqr(A, B, Q, R)  # K is for u = -Kx
    K *= -1

    print("-" * 40)
    print(" COMP0216 LQR GAIN FINDER (ACCEL MODE) ")
    print("-" * 40)
    print("Calculated Parameters:")
    print(f"  Pendulum Mass: {m_pend:.4f} kg")
    print(f"  CoM Distance:  {l:.4f} m")
    print(f"  Inertia:       {I_pivot:.4f} kg.m^2")
    print("-" * 40)
    print("Gains for Arduino Implementation (u = -K*x):")
    print(f"  k1 (x):         {K[0,0]:>10.4f}")
    print(f"  k2 (x_dot):     {K[0,1]:>10.4f}")
    print(f"  k3 (theta):     {K[0,2]:>10.4f}")
    print(f"  k4 (theta_dot): {K[0,3]:>10.4f}")
    print("-" * 40)
    print("ARDUINO PSEUDOCODE:")
    print("  accel = -(k1*x + k2*x_dot + k3*theta + k4*theta_dot);")
    print("  v_target += accel * dt;")
    print("  motorPID.setSetpoint(v_target);")

if __name__ == "__main__":
    calculate_lqr_gains()