#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Offline-designed LQR gain K for u = -K x
 * state x = [theta, theta_dot, phi, phi_dot]
 * 起始值依 6.5" / 15 對極輪轂調整；正式量產前請辨識後覆寫。
 */
extern const float BALANCE_LQR_K[2][4];
extern float g_balance_theta_bias_rad;

#ifdef __cplusplus
}
#endif
