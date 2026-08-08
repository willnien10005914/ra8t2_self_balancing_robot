#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Application configuration (tune per chassis). */
#define APP_CFG_LQR_ON_BOOT           (1)
#define APP_CFG_BALANCE_HZ            (500)
#define APP_CFG_IMU_ODR_HZ            (833)
#define APP_CFG_VX_MAX_MPS            (1.2f)
#define APP_CFG_WZ_MAX_RADPS          (2.5f)
#define APP_CFG_THETA_FAULT_RAD       (0.55f)  /* ~31 deg */
#define APP_CFG_AI_TIMEOUT_MS         (300u)
#define APP_CFG_RC_TIMEOUT_MS         (200u)
#define APP_CFG_WHEEL_TRACK_M         (0.45f)
#define APP_CFG_WHEEL_RADIUS_M        (0.0825f)

/** Ethernet AI box（RA8T2 GbE）。預設靜態 IP 可在 FSP / net_port 覆寫。 */
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
