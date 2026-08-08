#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Offline-designed LQR gain K for u = -K x
 * state x = [theta, theta_dot, phi, phi_dot] (or extended offline)
 * Replace after system identification / MATLAB dlqr.
 */
extern const float BALANCE_LQR_K[2][4];

/** Pitch upright bias (rad) after mechanical trim. */
extern float g_balance_theta_bias_rad;

#ifdef __cplusplus
}
#endif
