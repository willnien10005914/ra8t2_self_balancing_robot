#include "balance/balance_lqr.h"
#include "balance/balance_lqr_gain.h"
#include "app/app_cfg.h"

#include <string.h>

float g_balance_theta_bias_rad = 0.0f;

/* Placeholder gains — replace after identification. Rows: common / differential. */
const float BALANCE_LQR_K[2][4] = {
    { -40.0f, -6.0f, -1.2f, -0.8f },
    { 0.0f, 0.0f, 0.0f, 0.0f }
};

static balance_cmd_t s_cmd;
static balance_state_t s_st;
static bool s_en;

void balance_init(void)
{
    memset(&s_cmd, 0, sizeof(s_cmd));
    memset(&s_st, 0, sizeof(s_st));
    s_en = false;
}

void balance_set_cmd(const balance_cmd_t *cmd)
{
    if (cmd)
    {
        s_cmd = *cmd;
        s_en = cmd->lqr_enable;
    }
}

bool balance_enabled(void)
{
    return s_en;
}

void balance_get_state(balance_state_t *st)
{
    if (st)
    {
        *st = s_st;
    }
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
    if (!s_en)
    {
        return;
    }

    const float theta = imu->pitch_rad - g_balance_theta_bias_rad;
    const float theta_dot = imu->pitch_rate_radps;
    const float phi = 0.5f * (fb->pos_rad[0] + fb->pos_rad[1]);
    const float phi_dot = 0.5f * (fb->vel_radps[0] + fb->vel_radps[1]);
    const float psi_dot = imu->yaw_rate_radps;

    s_st.theta = theta;
    s_st.theta_dot = theta_dot;
    s_st.phi = phi;
    s_st.phi_dot = phi_dot;
    s_st.psi_dot = psi_dot;

    /* Tracking: convert vx/wz into state references (phi_dot, psi_dot). */
    float vx = s_cmd.brake ? 0.0f : s_cmd.vx_ref_mps;
    float wz = s_cmd.brake ? 0.0f : s_cmd.wz_ref_radps;
    if (vx > APP_CFG_VX_MAX_MPS) vx = APP_CFG_VX_MAX_MPS;
    if (vx < -APP_CFG_VX_MAX_MPS) vx = -APP_CFG_VX_MAX_MPS;
    if (wz > APP_CFG_WZ_MAX_RADPS) wz = APP_CFG_WZ_MAX_RADPS;
    if (wz < -APP_CFG_WZ_MAX_RADPS) wz = -APP_CFG_WZ_MAX_RADPS;

    const float phi_dot_ref = vx / APP_CFG_WHEEL_RADIUS_M;
    const float x[4] = {
        theta,
        theta_dot,
        phi,                 /* position regulation toward standstill/path later */
        phi_dot - phi_dot_ref
    };

    float u_common = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        u_common += BALANCE_LQR_K[0][i] * x[i];
    }

    /* Differential channel: P on yaw-rate error (extend LQR offline later). */
    const float u_diff = 2.0f * (wz - psi_dot);

    float v_l = vx - 0.5f * APP_CFG_WHEEL_TRACK_M * wz;
    float v_r = vx + 0.5f * APP_CFG_WHEEL_TRACK_M * wz;

    /* Blend LQR common correction into wheel linear velocities. */
    const float k_vel = 0.05f;
    v_l += k_vel * (u_common - 0.5f * u_diff);
    v_r += k_vel * (u_common + 0.5f * u_diff);

    out->tau_l = u_common - 0.5f * u_diff;
    out->tau_r = u_common + 0.5f * u_diff;
    out->v_l_mps = v_l;
    out->v_r_mps = v_r;
}
