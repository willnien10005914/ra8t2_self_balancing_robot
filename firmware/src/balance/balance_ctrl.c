#include "balance/balance_ctrl.h"
#include "balance/balance_lqr_gain.h"
#include "app/app_cfg.h"
#include "motor/motor_params.h"

#include <string.h>

float g_balance_theta_bias_rad = 0.0f;

const float BALANCE_LQR_K[2][4] = {
    /* theta, theta_dot, phi, phi_dot — 依 6.5" 高極對數輪轂微調起始值 */
    { -48.0f, -7.5f, -1.5f, -1.1f },
    { 0.0f, 0.0f, 0.0f, 0.0f }
};

static balance_mode_t s_mode = BALANCE_MODE_LQR;
static balance_cmd_t s_cmd;
static balance_state_t s_st;

static pid_t s_pid_pitch;
static pid_t s_pid_vel;
static pid_t s_pid_yaw;

static void clamp_cmd(float *vx, float *wz)
{
    if (*vx > APP_CFG_VX_MAX_MPS) *vx = APP_CFG_VX_MAX_MPS;
    if (*vx < -APP_CFG_VX_MAX_MPS) *vx = -APP_CFG_VX_MAX_MPS;
    if (*wz > APP_CFG_WZ_MAX_RADPS) *wz = APP_CFG_WZ_MAX_RADPS;
    if (*wz < -APP_CFG_WZ_MAX_RADPS) *wz = -APP_CFG_WZ_MAX_RADPS;
}

void balance_init(void)
{
    memset(&s_cmd, 0, sizeof(s_cmd));
    memset(&s_st, 0, sizeof(s_st));
#if APP_CFG_BALANCE_MODE_DEFAULT == 2
    s_mode = BALANCE_MODE_PID;
#elif APP_CFG_BALANCE_MODE_DEFAULT == 0
    s_mode = BALANCE_MODE_OFF;
#else
    s_mode = BALANCE_MODE_LQR;
#endif

    /* 外環 PID：節點 pitch → 輪線速度加減量；vel → pitch lean；yaw → 差速 */
    pid_init(&s_pid_pitch,
             APP_CFG_PID_PITCH_KP, APP_CFG_PID_PITCH_KI, APP_CFG_PID_PITCH_KD,
             -APP_CFG_VX_MAX_MPS, APP_CFG_VX_MAX_MPS, APP_CFG_VX_MAX_MPS);
    pid_init(&s_pid_vel,
             APP_CFG_PID_VEL_KP, APP_CFG_PID_VEL_KI, APP_CFG_PID_VEL_KD,
             -APP_CFG_PID_LEAN_MAX_RAD, APP_CFG_PID_LEAN_MAX_RAD,
             APP_CFG_PID_LEAN_MAX_RAD);
    pid_init(&s_pid_yaw,
             APP_CFG_PID_YAW_KP, APP_CFG_PID_YAW_KI, APP_CFG_PID_YAW_KD,
             -APP_CFG_WZ_MAX_RADPS, APP_CFG_WZ_MAX_RADPS, APP_CFG_WZ_MAX_RADPS);
}

void balance_set_mode(balance_mode_t mode)
{
    s_mode = mode;
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
    const float u_diff = APP_CFG_LQR_YAW_KP * (wz - s_st.psi_dot);

    float v_l = vx - 0.5f * APP_CFG_WHEEL_TRACK_M * wz;
    float v_r = vx + 0.5f * APP_CFG_WHEEL_TRACK_M * wz;
    v_l += APP_CFG_LQR_VEL_BLEND * (u_common - 0.5f * u_diff);
    v_r += APP_CFG_LQR_VEL_BLEND * (u_common + 0.5f * u_diff);

    out->tau_l = u_common - 0.5f * u_diff;
    out->tau_r = u_common + 0.5f * u_diff;
    out->v_l_mps = v_l;
    out->v_r_mps = v_r;
}

static void step_pid(float vx, float wz, float dt, balance_output_t *out)
{
    /* 速度誤差 → 目標傾角（前傾加速），再用 pitch PID → 共同線速度 */
    const float v_meas = s_st.phi_dot * MOTOR_SPEC_RADIUS_M;
    const float theta_ref = pid_step(&s_pid_vel, vx - v_meas, dt);
    const float v_common = pid_step(&s_pid_pitch, theta_ref - s_st.theta, dt);
    const float w_cmd = pid_step(&s_pid_yaw, wz - s_st.psi_dot, dt);

    const float v_l = v_common - 0.5f * APP_CFG_WHEEL_TRACK_M * w_cmd;
    const float v_r = v_common + 0.5f * APP_CFG_WHEEL_TRACK_M * w_cmd;

    out->tau_l = v_l;
    out->tau_r = v_r;
    out->v_l_mps = v_l;
    out->v_r_mps = v_r;
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

    if (s_mode == BALANCE_MODE_PID)
    {
        step_pid(vx, wz, dt, out);
    }
    else
    {
        step_lqr(vx, wz, out);
    }

    /* 限速：不超過額定轉速對應線速度的可配置比例 */
    const float vmax = APP_CFG_VX_MAX_MPS;
    if (out->v_l_mps > vmax) out->v_l_mps = vmax;
    if (out->v_l_mps < -vmax) out->v_l_mps = -vmax;
    if (out->v_r_mps > vmax) out->v_r_mps = vmax;
    if (out->v_r_mps < -vmax) out->v_r_mps = -vmax;
}
