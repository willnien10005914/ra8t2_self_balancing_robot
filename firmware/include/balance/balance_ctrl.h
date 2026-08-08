#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "imu/imu.h"
#include "motor/motor_iface.h"
#include "control/pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 外環平衡控制器：
 * - LQR / PID 正式輸出左右輪 **扭矩 τ (N·m)** → Iq* = τ / Kt
 * - 可選 debug：速度輸出 v_l/v_r（APP_CFG_BALANCE_USE_TORQUE=0）
 * - 內環 FSP FOC 電流 PI（扭矩模式旁路速度 PI）
 */

typedef enum {
    BALANCE_MODE_OFF = 0,
    BALANCE_MODE_LQR,
    BALANCE_MODE_PID
} balance_mode_t;

typedef struct {
    float vx_ref_mps;
    float wz_ref_radps;
    bool  brake;
    bool  enable;
} balance_cmd_t;

typedef struct {
    float theta;
    float theta_dot;
    float phi;
    float phi_dot;
    float psi;
    float psi_dot;
} balance_state_t;

typedef struct {
    float tau_l;     /**< Left wheel torque command (N·m). */
    float tau_r;     /**< Right wheel torque command (N·m). */
    float iq_l_a;    /**< Iq* left (A), derived from τ/Kt. */
    float iq_r_a;    /**< Iq* right (A). */
    float v_l_mps;   /**< Diagnostic / speed-mode fallback. */
    float v_r_mps;
} balance_output_t;

typedef struct {
    float pitch_kp, pitch_ki, pitch_kd;
    float vel_kp, vel_ki, vel_kd;
    float yaw_kp, yaw_ki, yaw_kd;
} balance_pid_gains_t;

void balance_init(void);
void balance_set_mode(balance_mode_t mode);
balance_mode_t balance_get_mode(void);
void balance_set_cmd(const balance_cmd_t *cmd);
void balance_step(const imu_sample_t *imu,
                  const motor_feedback_t *fb,
                  balance_output_t *out);
void balance_get_state(balance_state_t *st);
bool balance_enabled(void);

void balance_pid_get_gains(balance_pid_gains_t *g);
void balance_pid_set_gains(const balance_pid_gains_t *g);
void balance_pid_reset(void);

#ifdef __cplusplus
}
#endif
