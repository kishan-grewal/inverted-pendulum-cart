#include "observer.h"
#include "config.h"

static float x_hat[4] = {0.0, 0.0, 0.0, 0.0};
static float A[4][4];
static float B[4];
static float L[4][2];

void observer_init() {
    float m_pend = PEND_ROD_MASS + PEND_TIP_MASS;
    float M_t = CART_MASS + m_pend;
    float l = (PEND_ROD_MASS * PEND_LENGTH / 2.0 + PEND_TIP_MASS * PEND_LENGTH) / m_pend;
    float ml = m_pend * l;
    float I_rod_com = (1.0 / 12.0) * PEND_ROD_MASS * PEND_LENGTH * PEND_LENGTH;
    float I_pivot = I_rod_com + PEND_ROD_MASS * (PEND_LENGTH / 2.0) * (PEND_LENGTH / 2.0) + PEND_TIP_MASS * PEND_LENGTH * PEND_LENGTH;
    
    float b_x = 0.1;
    float b_theta = 0.001;
    
    float D0 = M_t * I_pivot - ml * ml;
    
    A[0][0] = 0.0; A[0][1] = 1.0; A[0][2] = 0.0; A[0][3] = 0.0;
    A[1][0] = 0.0; A[1][1] = -I_pivot * b_x / D0; A[1][2] = -ml * ml * GRAVITY / D0; A[1][3] = ml * b_theta / D0;
    A[2][0] = 0.0; A[2][1] = 0.0; A[2][2] = 0.0; A[2][3] = 1.0;
    A[3][0] = 0.0; A[3][1] = ml * b_x / D0; A[3][2] = M_t * ml * GRAVITY / D0; A[3][3] = -M_t * b_theta / D0;
    
    B[0] = 0.0;
    B[1] = I_pivot / D0;
    B[2] = 0.0;
    B[3] = -ml / D0;
    
    L[0][0] = 20.0; L[0][1] = 0.0;
    L[1][0] = 100.0; L[1][1] = 0.0;
    L[2][0] = 0.0; L[2][1] = 30.0;
    L[3][0] = 0.0; L[3][1] = 150.0;
    
    observer_reset();
}

void observer_reset() {
    x_hat[0] = 0.0;
    x_hat[1] = 0.0;
    x_hat[2] = 0.0;
    x_hat[3] = 0.0;
}

void observer_update(float y_measured[2], float u_velocity, float dt, float state_out[4]) {
    float y_pred[2];
    y_pred[0] = x_hat[0];
    y_pred[1] = x_hat[2];
    
    float innovation[2];
    innovation[0] = y_measured[0] - y_pred[0];
    innovation[1] = y_measured[1] - y_pred[1];
    
    float x_dot[4];
    for (int i = 0; i < 4; i++) {
        x_dot[i] = 0.0;
        for (int j = 0; j < 4; j++) {
            x_dot[i] += A[i][j] * x_hat[j];
        }
        x_dot[i] += B[i] * u_velocity;
        for (int j = 0; j < 2; j++) {
            x_dot[i] += L[i][j] * innovation[j];
        }
    }
    
    for (int i = 0; i < 4; i++) {
        x_hat[i] += x_dot[i] * dt;
    }
    
    state_out[0] = x_hat[0];
    state_out[1] = x_hat[1];
    state_out[2] = x_hat[2];
    state_out[3] = x_hat[3];
}