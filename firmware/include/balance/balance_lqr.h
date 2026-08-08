#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "imu/imu.h"
#include "motor/motor_iface.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float vx_ref_mps;   /**< Body forward (+). */
    float wz_ref_radps; /**< Yaw rate (+ left). */
    bool  brake;        /**< Force vx=wz=0 but keep balancer. */
    bool  lqr_enable;
} balance_cmd_t;

typedef struct {
    float theta;      /**< Pitch rad, upright ~0. */
    float theta_dot;
    float phi;        /**< Avg wheel angle rad. */
    float phi_dot;
    float psi;        /**< Yaw rad (integrated). */
    float psi_dot;
} balance_state_t;

typedef struct {
    float tau_l; /**< Left torque / speed proxy command. */
    float tau_r;
    float v_l_mps;
    float v_r_mps;
} balance_output_t;

void balance_init(void);
void balance_set_cmd(const balance_cmd_t *cmd);
void balance_step(const imu_sample_t *imu,
                  const motor_feedback_t *fb,
                  balance_output_t *out);
void balance_get_state(balance_state_t *st);
bool balance_enabled(void);

#ifdef __cplusplus
}
#endif
