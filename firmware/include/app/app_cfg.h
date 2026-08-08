#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "motor/motor_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Application configuration (tune per chassis / spec). */

/* 0=off 1=LQR 2=PID — 開機預設平衡模式 */
#define APP_CFG_BALANCE_MODE_DEFAULT  (1)
#define APP_CFG_BALANCE_ON_BOOT       (1)
#define APP_CFG_BALANCE_HZ            (500)
#define APP_CFG_IMU_ODR_HZ            (833)

/* 幾何：輪距自測；半徑取自 6.5" 規格書 */
#define APP_CFG_WHEEL_TRACK_M         (0.45f)
#define APP_CFG_WHEEL_RADIUS_M        (MOTOR_SPEC_RADIUS_M)

/* 運動限制：額定 ~4.32m/s，自平衡取較低可操作區 */
#define APP_CFG_VX_MAX_MPS            (1.50f)
#define APP_CFG_WZ_MAX_RADPS          (2.5f)
#define APP_CFG_RPM_CMD_MAX           (MOTOR_SPEC_RATED_RPM + MOTOR_SPEC_RPM_TOL) /* 550 */
#define APP_CFG_THETA_FAULT_RAD       (0.55f)

#define APP_CFG_AI_TIMEOUT_MS         (300u)
#define APP_CFG_RC_TIMEOUT_MS         (200u)

/* 相容舊宏 */
#define APP_CFG_LQR_ON_BOOT           (APP_CFG_BALANCE_ON_BOOT)

/* LQR 調參 */
#define APP_CFG_LQR_VEL_BLEND         (0.06f)
#define APP_CFG_LQR_YAW_KP            (2.2f)

/* 外環 PID（balmode pid）起始增益 — 上板後用 console 調 */
#define APP_CFG_PID_PITCH_KP          (8.0f)
#define APP_CFG_PID_PITCH_KI          (0.0f)
#define APP_CFG_PID_PITCH_KD          (0.35f)
#define APP_CFG_PID_VEL_KP            (0.12f)
#define APP_CFG_PID_VEL_KI            (0.02f)
#define APP_CFG_PID_VEL_KD            (0.0f)
#define APP_CFG_PID_YAW_KP            (1.8f)
#define APP_CFG_PID_YAW_KI            (0.0f)
#define APP_CFG_PID_YAW_KD            (0.05f)
#define APP_CFG_PID_LEAN_MAX_RAD      (0.12f) /* ~7 deg */

/** Ethernet AI box（RA8T2 GbE）。 */
#define APP_CFG_NET_ENABLE            (1)
#define APP_CFG_NET_TCP_PORT          (9000u)
#define APP_CFG_NET_UDP_PORT          (9000u)
#define APP_CFG_NET_STATIC_IP         "192.168.0.50"
#define APP_CFG_NET_STATIC_MASK       "255.255.255.0"
#define APP_CFG_NET_STATIC_GW         "192.168.0.1"

typedef enum {
    APP_STATE_BOOT = 0,
    APP_STATE_IMU_INIT,
    APP_STATE_MOTOR_INIT,
    APP_STATE_IDLE,
    APP_STATE_BALANCING,
    APP_STATE_TRACKING,
    APP_STATE_HOLD,
    APP_STATE_FAULT_SAFE
} app_state_t;

void app_main(void);
app_state_t app_state_get(void);
void app_request_estop(void);
void app_clear_fault(void);

#ifdef __cplusplus
}
#endif
