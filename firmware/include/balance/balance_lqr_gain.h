#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Offline / generated LQR gain: u_tau = sum K[0][i]*x[i]
 * x = [theta, theta_dot, phi, phi_dot]
 * Regenerate:
 *   python3 tools/lqr_gain_gen.py
 * writes firmware/src/balance/balance_lqr_gain.c
 */
extern const float BALANCE_LQR_K[2][4];
extern float g_balance_theta_bias_rad;

#ifdef __cplusplus
}
#endif
