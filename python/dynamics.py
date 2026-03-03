import numpy as np

# Physical parameters
M = 0.828  # cart mass [kg]
m_rod = 0.0225  # rod mass [kg]
m_tip = 0.050  # tip mass [kg]
m_pend = m_rod + m_tip
L_rod = 0.6  # rod length [m]
l = (m_rod * L_rod / 2 + m_tip * L_rod) / m_pend  # CoM from pivot
I_rod_com = (1 / 12) * m_rod * L_rod ** 2
I_pivot = I_rod_com + m_rod * (L_rod / 2) ** 2 + m_tip * L_rod ** 2
b_x = 0.1  # cart friction [N·s/m]
b_theta = 0.001  # pivot friction [N·m·s/rad]
c_drag = 0.002  # air drag coefficient
g = 9.81
M_t = M + m_pend
ml = m_pend * l


def state_derivative(t, state, F=0.0, enable_air_drag=True):
    x, x_dot, theta, theta_dot = state
    
    c = np.cos(theta)
    s = np.sin(theta)
    
    # Mass matrix determinant
    D = M_t * I_pivot - (ml * c) ** 2
    
    tau_drag = -c_drag * theta_dot * np.abs(theta_dot) if enable_air_drag else 0.0
    
    # RHS terms
    f1 = F - b_x * x_dot + ml * theta_dot ** 2 * s
    f2 = ml * g * s - b_theta * theta_dot + tau_drag
    
    # Solve mass matrix: [M_t, ml*c; ml*c, I] @ [x_ddot; theta_ddot] = [f1; f2]
    x_ddot = (I_pivot * f1 - ml * c * f2) / D
    theta_ddot = (M_t * f2 - ml * c * f1) / D
    
    return np.array([x_dot, x_ddot, theta_dot, theta_ddot])


def linearise():
    # At theta=0: cos=1, sin=0, theta_dot=0
    D0 = M_t * I_pivot - ml ** 2
    
    A = np.array([
        [0, 1, 0, 0],
        [0, -I_pivot * b_x / D0, -ml ** 2 * g / D0, ml * b_theta / D0],
        [0, 0, 0, 1],
        [0, ml * b_x / D0, M_t * ml * g / D0, -M_t * b_theta / D0],
    ])
    
    B = np.array([
        [0],
        [I_pivot / D0],
        [0],
        [-ml / D0],
    ])
    
    return A, B


def get_parameters():
    return {
        'M': M, 'm_rod': m_rod, 'm_tip': m_tip, 'm_pend': m_pend,
        'L_rod': L_rod, 'l': l, 'I_pivot': I_pivot,
        'b_x': b_x, 'b_theta': b_theta, 'c_drag': c_drag,
        'g': g, 'M_t': M_t, 'ml': ml,
    }