#include "app/app_main.h"
#include "app/app_cfg.h"
#include "imu/imu.h"
#include "balance/balance_lqr.h"
#include "motor/motor_iface.h"
#include "cmd/cmd_arbiter.h"
#include "cmd/console.h"
#include "cmd/ai_link.h"
#include "cmd/net_link.h"
#include "cmd/rc_link.h"
#include "util/util.h"

#include <string.h>

static app_state_t s_state = APP_STATE_BOOT;
static uint32_t s_last_balance_ms;

app_state_t app_state_get(void)
{
    return s_state;
}

void app_request_estop(void)
{
    motor_estop();
    s_state = APP_STATE_FAULT_SAFE;
}

void app_clear_fault(void)
{
    if (s_state == APP_STATE_FAULT_SAFE)
    {
        s_state = APP_STATE_IDLE;
    }
}

static void app_enter_balancing(void)
{
    balance_cmd_t bc;
    memset(&bc, 0, sizeof(bc));
    bc.lqr_enable = true;
    balance_set_cmd(&bc);
    s_state = APP_STATE_BALANCING;
}

static void balance_period(uint32_t now_ms)
{
    const uint32_t dt_ms = 1000u / APP_CFG_BALANCE_HZ;
    if ((now_ms - s_last_balance_ms) < dt_ms)
    {
        return;
    }
    s_last_balance_ms = now_ms;

    imu_sample_t imu;
    motor_feedback_t fb;
    balance_output_t out;
    motion_cmd_t mc;
    motor_direct_cmd_t dc;

    if (!imu_update(&imu) || !imu.valid)
    {
        app_request_estop();
        console_printf("FAULT: imu\r\n");
        return;
    }

    if (imu.pitch_rad > APP_CFG_THETA_FAULT_RAD ||
        imu.pitch_rad < -APP_CFG_THETA_FAULT_RAD)
    {
        app_request_estop();
        console_printf("FAULT: tilt\r\n");
        return;
    }

    motor_get_feedback(&fb);
    cmd_arbiter_get_motion(&mc);

    if (mc.estop)
    {
        app_request_estop();
        return;
    }
    if (mc.clear_fault)
    {
        app_clear_fault();
    }

    if (s_state == APP_STATE_FAULT_SAFE)
    {
        return;
    }

    /* Direct wheel control only when LQR off. */
    if (!mc.lqr_enable && cmd_arbiter_get_direct(&dc) && dc.active)
    {
        motor_id_t id = (dc.side == 0) ? MOTOR_ID_LEFT :
                        (dc.side == 1) ? MOTOR_ID_RIGHT : MOTOR_ID_BOTH;
        if (dc.set_mode)
        {
            motor_set_mode(id, (motor_mode_t)dc.mode);
        }
        if (dc.set_speed)
        {
            motor_set_speed_rpm(id, dc.speed_rpm);
        }
        if (dc.set_pos)
        {
            motor_set_position_counts(id, dc.pos_counts);
        }
        s_state = APP_STATE_IDLE;
        return;
    }

    balance_cmd_t bc;
    bc.vx_ref_mps = mc.brake ? 0.0f : mc.vx_mps;
    bc.wz_ref_radps = mc.brake ? 0.0f : mc.wz_radps;
    bc.brake = mc.brake;
    bc.lqr_enable = mc.lqr_enable;
    balance_set_cmd(&bc);

    if (!mc.lqr_enable)
    {
        s_state = APP_STATE_IDLE;
        return;
    }

    balance_step(&imu, &fb, &out);

    /* Map LQR output to wheel speed mode (NM→rpm tuning later). */
    const float rpm_l = (out.v_l_mps / APP_CFG_WHEEL_RADIUS_M) * (60.0f / 6.2831853f);
    const float rpm_r = (out.v_r_mps / APP_CFG_WHEEL_RADIUS_M) * (60.0f / 6.2831853f);
    motor_set_mode(MOTOR_ID_BOTH, MOTOR_MODE_SPEED);
    motor_set_speed_rpm(MOTOR_ID_LEFT, rpm_l);
    motor_set_speed_rpm(MOTOR_ID_RIGHT, rpm_r);

    if (mc.brake || (mc.vx_mps == 0.0f && mc.wz_radps == 0.0f))
    {
        s_state = APP_STATE_HOLD;
    }
    else
    {
        s_state = APP_STATE_TRACKING;
    }
}

void app_main(void)
{
    console_init();
    ai_link_init();
#if APP_CFG_NET_ENABLE
    net_link_init();
#endif
    rc_link_init();
    cmd_arbiter_init();
    balance_init();

    s_state = APP_STATE_IMU_INIT;
    console_printf("ra8t2 balance boot\r\n");
#if APP_CFG_NET_ENABLE
    console_printf("net AI TCP/UDP port %u ip %s\r\n",
                   (unsigned)APP_CFG_NET_TCP_PORT, APP_CFG_NET_STATIC_IP);
#endif

    if (!imu_init())
    {
        console_printf("imu init fail\r\n");
        s_state = APP_STATE_FAULT_SAFE;
    }

    s_state = APP_STATE_MOTOR_INIT;
    if (!motor_init())
    {
        console_printf("motor init fail\r\n");
        s_state = APP_STATE_FAULT_SAFE;
    }

    motion_cmd_t boot;
    memset(&boot, 0, sizeof(boot));
    boot.lqr_enable = (APP_CFG_LQR_ON_BOOT != 0);
    boot.lqr_valid = true;
    boot.source = CMD_SRC_CONSOLE;
    boot.stamp_ms = time_ms_get();
    cmd_arbiter_inject_motion(&boot);

    if (s_state != APP_STATE_FAULT_SAFE)
    {
        if (APP_CFG_LQR_ON_BOOT)
        {
            app_enter_balancing();
        }
        else
        {
            s_state = APP_STATE_IDLE;
        }
    }

    s_last_balance_ms = time_ms_get();

    for (;;)
    {
        const uint32_t now = time_ms_get();
        console_poll();
        ai_link_poll(now);
#if APP_CFG_NET_ENABLE
        net_link_poll(now);
#endif
        rc_link_poll(now);
        cmd_arbiter_tick(now);
        balance_period(now);
        /* FSP: idle / WFI or RTOS yield here. */
    }
}
