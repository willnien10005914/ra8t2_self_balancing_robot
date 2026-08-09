#include "balance/balance_ctrl.h"
#include "balance/balance_lqr_gain.h"
#include "app/app_cfg.h"
#include "motor/motor_params.h"

#include <string.h>

/* BALANCE_LQR_K + g_balance_theta_bias_rad: balance_lqr_gain.c */

static balance_mode_t s_mode = BALANCE_MODE_LQR;
static balance_cmd_t s_cmd;
static balance_state_t s_st;
static float s_vx_slew;
static float s_wz_slew;

static pid_t s_pid_pitch;
static pid_t s_pid_vel;
static pid_t s_pid_yaw;

static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/** 線性斜率限制：每步最多改變 ax*dt，避免起步突衝。 */
static float slew_toward(float cur, float tgt, float rate_abs, float dt)
{
    const float max_step = rate_abs * dt;
    const float err = tgt - cur;
    if (err > max_step) return cur + max_step;
    if (err < -max_step) return cur - max_step;
    return tgt;
}

static void clamp_cmd(float *vx, float *wz)
{
    *vx = clampf(*vx, -APP_CFG_VX_MAX_MPS, APP_CFG_VX_MAX_MPS);
    *wz = clampf(*wz, -APP_CFG_WZ_MAX_RADPS, APP_CFG_WZ_MAX_RADPS);
}

static void pack_torque(balance_output_t *out, float tau_l, float tau_r,
                        float vx, float wz)
{
    tau_l = clampf(tau_l, -APP_CFG_TAU_CMD_MAX_NM, APP_CFG_TAU_CMD_MAX_NM);
    tau_r = clampf(tau_r, -APP_CFG_TAU_CMD_MAX_NM, APP_CFG_TAU_CMD_MAX_NM);
    out->tau_l = tau_l;
    out->tau_r = tau_r;
    out->iq_l_a = MOTOR_SPEC_IQ_FROM_TAU(tau_l);
    out->iq_r_a = MOTOR_SPEC_IQ_FROM_TAU(tau_r);
    out->iq_l_a = clampf(out->iq_l_a, -MOTOR_SPEC_PEAK_CURRENT_A, MOTOR_SPEC_PEAK_CURRENT_A);
    out->iq_r_a = clampf(out->iq_r_a, -MOTOR_SPEC_PEAK_CURRENT_A, MOTOR_SPEC_PEAK_CURRENT_A);
    /* 診斷用運動學參考；線速度硬限（含差速單側） */
    out->v_l_mps = clampf(vx - 0.5f * APP_CFG_WHEEL_TRACK_M * wz,
                          -APP_CFG_VX_MAX_MPS, APP_CFG_VX_MAX_MPS);
    out->v_r_mps = clampf(vx + 0.5f * APP_CFG_WHEEL_TRACK_M * wz,
                          -APP_CFG_VX_MAX_MPS, APP_CFG_VX_MAX_MPS);
}

void balance_init(void)
{
    memset(&s_cmd, 0, sizeof(s_cmd));
    memset(&s_st, 0, sizeof(s_st));
    s_vx_slew = 0.0f;
    s_wz_slew = 0.0f;
#if APP_CFG_BALANCE_MODE_DEFAULT == 2
    s_mode = BALANCE_MODE_PID;
#elif APP_CFG_BALANCE_MODE_DEFAULT == 0
    s_mode = BALANCE_MODE_OFF;
#else
    s_mode = BALANCE_MODE_LQR;
#endif

    pid_init(&s_pid_pitch,
             APP_CFG_PID_PITCH_KP, APP_CFG_PID_PITCH_KI, APP_CFG_PID_PITCH_KD,
             -APP_CFG_TAU_CMD_MAX_NM, APP_CFG_TAU_CMD_MAX_NM, APP_CFG_TAU_CMD_MAX_NM);
    pid_init(&s_pid_vel,
             APP_CFG_PID_VEL_KP, APP_CFG_PID_VEL_KI, APP_CFG_PID_VEL_KD,
             -APP_CFG_PID_LEAN_MAX_RAD, APP_CFG_PID_LEAN_MAX_RAD,
             APP_CFG_PID_LEAN_MAX_RAD);
    pid_init(&s_pid_yaw,
             APP_CFG_PID_YAW_KP, APP_CFG_PID_YAW_KI, APP_CFG_PID_YAW_KD,
             -APP_CFG_TAU_CMD_MAX_NM, APP_CFG_TAU_CMD_MAX_NM, APP_CFG_TAU_CMD_MAX_NM);
}

void balance_set_mode(balance_mode_t mode)
{
    s_mode = mode;
    s_vx_slew = 0.0f;
    s_wz_slew = 0.0f;
    balance_pid_reset();
}

balance_mode_t balance_get_mode(void)
{
    return s_mode;
}

void balance_set_cmd(const balance_cmd_t *cmd)
{
    if (cmd)
    {
        s_cmd = *cmd;
    }
}

bool balance_enabled(void)
{
    return s_cmd.enable && (s_mode != BALANCE_MODE_OFF);
}

void balance_get_state(balance_state_t *st)
{
    if (st) *st = s_st;
}

void balance_pid_reset(void)
{
    pid_reset(&s_pid_pitch);
    pid_reset(&s_pid_vel);
    pid_reset(&s_pid_yaw);
}

void balance_pid_get_gains(balance_pid_gains_t *g)
{
    if (!g) return;
    g->pitch_kp = s_pid_pitch.kp;
    g->pitch_ki = s_pid_pitch.ki;
    g->pitch_kd = s_pid_pitch.kd;
    g->vel_kp = s_pid_vel.kp;
    g->vel_ki = s_pid_vel.ki;
    g->vel_kd = s_pid_vel.kd;
    g->yaw_kp = s_pid_yaw.kp;
    g->yaw_ki = s_pid_yaw.ki;
    g->yaw_kd = s_pid_yaw.kd;
}

void balance_pid_set_gains(const balance_pid_gains_t *g)
{
    if (!g) return;
    pid_set_gains(&s_pid_pitch, g->pitch_kp, g->pitch_ki, g->pitch_kd);
    pid_set_gains(&s_pid_vel, g->vel_kp, g->vel_ki, g->vel_kd);
    pid_set_gains(&s_pid_yaw, g->yaw_kp, g->yaw_ki, g->yaw_kd);
}

static void fill_state(const imu_sample_t *imu, const motor_feedback_t *fb)
{
    s_st.theta = imu->pitch_rad - g_balance_theta_bias_rad;
    s_st.theta_dot = imu->pitch_rate_radps;
    s_st.phi = 0.5f * (fb->pos_rad[0] + fb->pos_rad[1]);
    s_st.phi_dot = 0.5f * (fb->vel_radps[0] + fb->vel_radps[1]);
    s_st.psi_dot = imu->yaw_rate_radps;
}

static void step_lqr(float vx, float wz, balance_output_t *out)
{
    const float phi_dot_ref = vx / MOTOR_SPEC_RADIUS_M;
    const float x[4] = {
        s_st.theta,
        s_st.theta_dot,
        s_st.phi,
        s_st.phi_dot - phi_dot_ref
    };

    float u_common = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        u_common += BALANCE_LQR_K[0][i] * x[i];
    }

    /* 跟蹤前饋：需要前進時略加共同力矩（加速輪到腳下） */
    const float v_meas = s_st.phi_dot * MOTOR_SPEC_RADIUS_M;
    u_common += APP_CFG_LQR_TAU_FF_VX * (vx - v_meas);

    const float u_diff = APP_CFG_LQR_YAW_KP * (wz - s_st.psi_dot);
    pack_torque(out, u_common - 0.5f * u_diff, u_common + 0.5f * u_diff, vx, wz);
}

static void step_pid(float vx, float wz, float dt, balance_output_t *out)
{
    const float v_meas = s_st.phi_dot * MOTOR_SPEC_RADIUS_M;
    const float theta_ref = pid_step(&s_pid_vel, vx - v_meas, dt);
    const float tau_common = pid_step(&s_pid_pitch, theta_ref - s_st.theta, dt);
    const float tau_diff = pid_step(&s_pid_yaw, wz - s_st.psi_dot, dt);
    pack_torque(out, tau_common - 0.5f * tau_diff, tau_common + 0.5f * tau_diff, vx, wz);
}

void balance_step(const imu_sample_t *imu,
                  const motor_feedback_t *fb,
                  balance_output_t *out)
{
    if (!out || !imu || !fb)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    fill_state(imu, fb);

    if (!balance_enabled())
    {
        return;
    }

    float vx = s_cmd.brake ? 0.0f : s_cmd.vx_ref_mps;
    float wz = s_cmd.brake ? 0.0f : s_cmd.wz_ref_radps;
    clamp_cmd(&vx, &wz);

    const float dt = 1.0f / (float)APP_CFG_BALANCE_HZ;
    /* 線性加減速：指令可瞬間變，實際參考以 AX/AWZ 斜率爬升 */
    s_vx_slew = slew_toward(s_vx_slew, vx, APP_CFG_AX_MAX_MPS2, dt);
    s_wz_slew = slew_toward(s_wz_slew, wz, APP_CFG_AWZ_MAX_RADPS2, dt);
    clamp_cmd(&s_vx_slew, &s_wz_slew);
    vx = s_vx_slew;
    wz = s_wz_slew;

    if (s_mode == BALANCE_MODE_PID)
    {
        step_pid(vx, wz, dt, out);
    }
    else
    {
        step_lqr(vx, wz, out);
    }
}
