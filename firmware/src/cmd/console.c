#include "cmd/console.h"
#include "cmd/cmd_arbiter.h"
#include "imu/imu.h"
#include "motor/motor_iface.h"
#include "motor/motor_params.h"
#include "balance/balance_ctrl.h"
#include "app/app_main.h"
#include "app/app_cfg.h"
#include "util/util.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>

#define LINE_MAX 96

static char s_line[LINE_MAX];
static unsigned s_len;
static void console_handle_line(char *line);

__attribute__((weak)) void console_uart_write(const uint8_t *data, uint32_t len)
{
    (void)data;
    (void)len;
}

void console_printf(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0)
    {
        if (n > (int)sizeof(buf)) n = (int)sizeof(buf);
        console_uart_write((const uint8_t *)buf, (uint32_t)n);
    }
}

void console_init(void)
{
    s_len = 0;
}

void console_rx_byte(uint8_t b)
{
    if (b == '\r') return;
    if (b == '\n')
    {
        s_line[s_len < LINE_MAX ? s_len : LINE_MAX - 1] = 0;
        console_handle_line(s_line);
        s_len = 0;
        return;
    }
    if (s_len + 1 < LINE_MAX)
    {
        s_line[s_len++] = (char)b;
    }
}

static int parse_side(const char *s)
{
    if (!s) return -1;
    if (!strcmp(s, "l") || !strcmp(s, "L") || !strcmp(s, "left")) return 0;
    if (!strcmp(s, "r") || !strcmp(s, "R") || !strcmp(s, "right")) return 1;
    if (!strcmp(s, "both") || !strcmp(s, "b")) return 2;
    return -1;
}

static void inject_motion(float vx, float wz, bool brake, int lqr_set)
{
    motion_cmd_t m;
    memset(&m, 0, sizeof(m));
    m.vx_mps = vx;
    m.wz_radps = wz;
    m.brake = brake;
    m.source = CMD_SRC_CONSOLE;
    m.stamp_ms = time_ms_get();
    if (lqr_set >= 0)
    {
        m.lqr_valid = true;
        m.lqr_enable = (lqr_set != 0);
    }
    cmd_arbiter_inject_motion(&m);
}

static void console_handle_line(char *line)
{
    char *argv[8];
    int argc = 0;
    char *p = strtok(line, " \t");
    while (p && argc < 8)
    {
        argv[argc++] = p;
        p = strtok(NULL, " \t");
    }
    if (argc == 0) return;

    if (!strcmp(argv[0], "help"))
    {
        console_printf("cmds: status imu pos mode speed posset lqr balmode pid fwd back left right turn brake estop clear ai rc\r\n");
    }
    else if (!strcmp(argv[0], "status"))
    {
        const imu_sample_t *imu = imu_latest();
        motor_feedback_t fb;
        motion_cmd_t m;
        const char *bm = "?";
        motor_get_feedback(&fb);
        cmd_arbiter_get_motion(&m);
        switch (balance_get_mode())
        {
            case BALANCE_MODE_OFF: bm = "off"; break;
            case BALANCE_MODE_LQR: bm = "lqr"; break;
            case BALANCE_MODE_PID: bm = "pid"; break;
        }
        console_printf("st=%d pitch=%.3f iqL=%.2f iqR=%.2f en=%d mode=%s act=%s vx=%.2f wz=%.2f\r\n",
                       (int)app_state_get(),
                       imu->pitch_rad,
                       fb.current_a[0], fb.current_a[1],
                       (int)m.lqr_enable, bm,
#if APP_CFG_BALANCE_USE_TORQUE
                       "torque",
#else
                       "speed",
#endif
                       m.vx_mps, m.wz_radps);
    }
    else if (!strcmp(argv[0], "imu"))
    {
        const imu_sample_t *imu = imu_latest();
        console_printf("a=%.2f,%.2f,%.2f g=%.1f,%.1f,%.1f pitch=%.3f\r\n",
                       imu->ax_g, imu->ay_g, imu->az_g,
                       imu->gx_dps, imu->gy_dps, imu->gz_dps,
                       imu->pitch_rad);
    }
    else if (!strcmp(argv[0], "pos"))
    {
        motor_feedback_t fb;
        motor_get_feedback(&fb);
        console_printf("L=%ld R=%ld (%.3f, %.3f rad)\r\n",
                       (long)fb.hall_counts[0], (long)fb.hall_counts[1],
                       fb.pos_rad[0], fb.pos_rad[1]);
    }
    else if (!strcmp(argv[0], "mode") && argc >= 3)
    {
        int side = parse_side(argv[1]);
        motor_direct_cmd_t d;
        memset(&d, 0, sizeof(d));
        d.active = true;
        d.side = (uint8_t)side;
        d.set_mode = true;
        if (!strcmp(argv[2], "speed")) d.mode = MOTOR_MODE_SPEED;
        else if (!strcmp(argv[2], "position")) d.mode = MOTOR_MODE_POSITION;
        else if (!strcmp(argv[2], "torque")) d.mode = MOTOR_MODE_TORQUE;
        else { console_printf("bad mode\r\n"); return; }
        inject_motion(0, 0, true, 0);
        cmd_arbiter_inject_direct(&d);
    }
    else if (!strcmp(argv[0], "speed") && argc >= 3)
    {
        int side = parse_side(argv[1]);
        motor_direct_cmd_t d;
        memset(&d, 0, sizeof(d));
        d.active = true;
        d.side = (uint8_t)side;
        d.set_speed = true;
        d.speed_rpm = strtof(argv[2], NULL);
        inject_motion(0, 0, true, 0);
        cmd_arbiter_inject_direct(&d);
    }
    else if (!strcmp(argv[0], "posset") && argc >= 3)
    {
        int side = parse_side(argv[1]);
        motor_direct_cmd_t d;
        memset(&d, 0, sizeof(d));
        d.active = true;
        d.side = (uint8_t)side;
        d.set_pos = true;
        d.pos_counts = (int32_t)strtol(argv[2], NULL, 0);
        inject_motion(0, 0, true, 0);
        cmd_arbiter_inject_direct(&d);
    }
    else if (!strcmp(argv[0], "lqr") && argc >= 2)
    {
        /* lqr on|off：啟停平衡；若 on 且目前 off 模式則切回 LQR */
        const int on = !strcmp(argv[1], "on");
        if (on && balance_get_mode() == BALANCE_MODE_OFF)
        {
            balance_set_mode(BALANCE_MODE_LQR);
        }
        inject_motion(0, 0, true, on);
    }
    else if (!strcmp(argv[0], "balmode") && argc >= 2)
    {
        if (!strcmp(argv[1], "lqr"))
        {
            balance_set_mode(BALANCE_MODE_LQR);
            inject_motion(0, 0, true, 1);
            console_printf("balmode=lqr\r\n");
        }
        else if (!strcmp(argv[1], "pid"))
        {
            balance_set_mode(BALANCE_MODE_PID);
            inject_motion(0, 0, true, 1);
            console_printf("balmode=pid\r\n");
        }
        else if (!strcmp(argv[1], "off"))
        {
            balance_set_mode(BALANCE_MODE_OFF);
            inject_motion(0, 0, true, 0);
            console_printf("balmode=off\r\n");
        }
        else
        {
            console_printf("balmode lqr|pid|off\r\n");
        }
    }
    else if (!strcmp(argv[0], "pid"))
    {
        balance_pid_gains_t g;
        balance_pid_get_gains(&g);
        if (argc == 1)
        {
            console_printf("pitch=%.2f/%.2f/%.2f vel=%.2f/%.2f/%.2f yaw=%.2f/%.2f/%.2f\r\n",
                           g.pitch_kp, g.pitch_ki, g.pitch_kd,
                           g.vel_kp, g.vel_ki, g.vel_kd,
                           g.yaw_kp, g.yaw_ki, g.yaw_kd);
            console_printf("foc tip: Kt=%.2f Nm/A Iqlim=%.1fA poles=%u act=torque\r\n",
                           MOTOR_SPEC_KT_NM_PER_A, FOC_SPEED_OUT_MAX_A,
                           (unsigned)MOTOR_SPEC_POLE_PAIRS);
        }
        else if (argc >= 5 && !strcmp(argv[1], "pitch"))
        {
            g.pitch_kp = strtof(argv[2], NULL);
            g.pitch_ki = strtof(argv[3], NULL);
            g.pitch_kd = strtof(argv[4], NULL);
            balance_pid_set_gains(&g);
            balance_pid_reset();
            console_printf("ok pitch\r\n");
        }
        else if (argc >= 5 && !strcmp(argv[1], "vel"))
        {
            g.vel_kp = strtof(argv[2], NULL);
            g.vel_ki = strtof(argv[3], NULL);
            g.vel_kd = strtof(argv[4], NULL);
            balance_pid_set_gains(&g);
            balance_pid_reset();
            console_printf("ok vel\r\n");
        }
        else if (argc >= 5 && !strcmp(argv[1], "yaw"))
        {
            g.yaw_kp = strtof(argv[2], NULL);
            g.yaw_ki = strtof(argv[3], NULL);
            g.yaw_kd = strtof(argv[4], NULL);
            balance_pid_set_gains(&g);
            balance_pid_reset();
            console_printf("ok yaw\r\n");
        }
        else
        {
            console_printf("pid | pid pitch|vel|yaw <kp> <ki> <kd>\r\n");
        }
    }
    else if (!strcmp(argv[0], "fwd") && argc >= 2)
    {
        inject_motion(strtof(argv[1], NULL), 0, false, -1);
    }
    else if (!strcmp(argv[0], "back") && argc >= 2)
    {
        inject_motion(-strtof(argv[1], NULL), 0, false, -1);
    }
    else if (!strcmp(argv[0], "left") && argc >= 2)
    {
        inject_motion(0, strtof(argv[1], NULL), false, -1);
    }
    else if (!strcmp(argv[0], "right") && argc >= 2)
    {
        inject_motion(0, -strtof(argv[1], NULL), false, -1);
    }
    else if (!strcmp(argv[0], "turn") && argc >= 2)
    {
        inject_motion(0, strtof(argv[1], NULL), false, -1);
    }
    else if (!strcmp(argv[0], "brake"))
    {
        inject_motion(0, 0, true, -1);
    }
    else if (!strcmp(argv[0], "estop"))
    {
        motion_cmd_t m;
        memset(&m, 0, sizeof(m));
        m.estop = true;
        m.source = CMD_SRC_CONSOLE;
        m.stamp_ms = time_ms_get();
        cmd_arbiter_inject_motion(&m);
        app_request_estop();
    }
    else if (!strcmp(argv[0], "clear"))
    {
        motion_cmd_t m;
        memset(&m, 0, sizeof(m));
        m.clear_fault = true;
        m.source = CMD_SRC_CONSOLE;
        m.stamp_ms = time_ms_get();
        cmd_arbiter_inject_motion(&m);
        app_clear_fault();
    }
    else if (!strcmp(argv[0], "ai") && argc >= 2)
    {
        cmd_arbiter_set_ai_enabled(!strcmp(argv[1], "on"));
    }
    else if (!strcmp(argv[0], "rc") && argc >= 2)
    {
        cmd_arbiter_set_rc_enabled(!strcmp(argv[1], "on"));
    }
    else
    {
        console_printf("?\r\n");
    }
}

void console_poll(void)
{
    /* RX handled in IRQ via console_rx_byte; nothing required. */
}
