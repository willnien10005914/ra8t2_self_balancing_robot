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

/*
 * 速度安全鎖（防爆衝）
 * - APP_CFG_SPEED_SAFETY_LOCK=1：最高線速度鎖在 0.5 m/s（依輪徑換算 rpm）
 * - 起步/加減速採線性斜率限制（APP_CFG_AX_MAX_MPS2）
 * - 解禁：必須改 FW（見下方註解）後重新編譯燒錄，runtime 無法解鎖
 */
#define APP_CFG_SPEED_SAFETY_LOCK     (1)

#if APP_CFG_SPEED_SAFETY_LOCK
#define APP_CFG_VX_MAX_MPS            (0.50f)   /* 硬上限：每秒 0.5 公尺 */
#define APP_CFG_WZ_MAX_RADPS          (1.00f)   /* 轉向角速度上限（rad/s） */
#define APP_CFG_AX_MAX_MPS2           (0.25f)   /* 線加速度：0→0.5 m/s 約 2 秒 */
#define APP_CFG_AWZ_MAX_RADPS2        (0.80f)   /* 角加速度斜率 */
#else
/* ===== 解禁區：僅在明確需求時改 SPEED_SAFETY_LOCK=0 並調大以下值 ===== */
#define APP_CFG_VX_MAX_MPS            (1.50f)
#define APP_CFG_WZ_MAX_RADPS          (2.50f)
#define APP_CFG_AX_MAX_MPS2           (0.80f)
#define APP_CFG_AWZ_MAX_RADPS2        (2.00f)
#endif

/* rpm = v / r * 60 / (2π)；與輪徑連動，避免 speed 模式繞過線速度鎖 */
#define APP_CFG_RPM_CMD_MAX \
    ((APP_CFG_VX_MAX_MPS / APP_CFG_WHEEL_RADIUS_M) * (60.0f / 6.28318530718f))

#define APP_CFG_THETA_FAULT_RAD       (0.55f)

#define APP_CFG_AI_TIMEOUT_MS         (300u)
#define APP_CFG_RC_TIMEOUT_MS         (200u)

/* 相容舊宏 */
#define APP_CFG_LQR_ON_BOOT           (APP_CFG_BALANCE_ON_BOOT)

/* 外環輸出執行動器：1=扭矩(τ→Iq*) 建議；0=速度(rpm，僅調機) */
#define APP_CFG_BALANCE_USE_TORQUE    (1)

/* LQR：輸出視為扭矩(Nm)；yaw 通道為差速扭矩比例 */
#define APP_CFG_LQR_YAW_KP            (1.8f)
#define APP_CFG_LQR_TAU_FF_VX         (0.8f)   /* N·m per (m/s) 速度跟蹤前饋 */
#define APP_CFG_TAU_CMD_MAX_NM        (MOTOR_SPEC_MAX_TORQUE_NM)

/* 外環 PID：pitch→共同扭矩(Nm)；vel→lean(rad)；yaw→差速扭矩(Nm) */
#define APP_CFG_PID_PITCH_KP          (45.0f)
#define APP_CFG_PID_PITCH_KI          (0.0f)
#define APP_CFG_PID_PITCH_KD          (2.5f)
#define APP_CFG_PID_VEL_KP            (0.12f)
#define APP_CFG_PID_VEL_KI            (0.02f)
#define APP_CFG_PID_VEL_KD            (0.0f)
#define APP_CFG_PID_YAW_KP            (6.0f)
#define APP_CFG_PID_YAW_KI            (0.0f)
#define APP_CFG_PID_YAW_KD            (0.20f)
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
