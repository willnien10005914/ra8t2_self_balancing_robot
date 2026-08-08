#include "cmd/cmd_arbiter.h"
#include "app/app_cfg.h"

#include <string.h>

static motion_cmd_t s_motion;
static motor_direct_cmd_t s_direct;
static bool s_ai_en = true;
static bool s_rc_en = false;
static uint32_t s_ai_stamp;
static uint32_t s_rc_stamp;

void cmd_arbiter_init(void)
{
    memset(&s_motion, 0, sizeof(s_motion));
    memset(&s_direct, 0, sizeof(s_direct));
    s_motion.lqr_enable = (APP_CFG_LQR_ON_BOOT != 0);
    s_motion.lqr_valid = true;
}

void cmd_arbiter_set_ai_enabled(bool en) { s_ai_en = en; }
void cmd_arbiter_set_rc_enabled(bool en) { s_rc_en = en; }
bool cmd_arbiter_ai_enabled(void) { return s_ai_en; }
bool cmd_arbiter_rc_enabled(void) { return s_rc_en; }

static int prio(cmd_source_t s)
{
    switch (s)
    {
        case CMD_SRC_CONSOLE: return 3;
        case CMD_SRC_AI:      return 2;
        case CMD_SRC_RC:      return 1;
        default:              return 0;
    }
}

void cmd_arbiter_inject_motion(const motion_cmd_t *cmd)
{
    if (!cmd) return;
    if (cmd->estop)
    {
        s_motion = *cmd;
        s_motion.brake = true;
        s_motion.vx_mps = 0.0f;
        s_motion.wz_radps = 0.0f;
        return;
    }
    if (cmd->source == CMD_SRC_AI && !s_ai_en) return;
    if (cmd->source == CMD_SRC_RC && !s_rc_en) return;

    /* Higher or equal priority overwrites; console always wins for lqr toggles. */
    if (prio(cmd->source) >= prio(s_motion.source) || cmd->source == CMD_SRC_CONSOLE)
    {
        const bool prev_lqr = s_motion.lqr_enable;
        s_motion.vx_mps = cmd->vx_mps;
        s_motion.wz_radps = cmd->wz_radps;
        s_motion.brake = cmd->brake;
        s_motion.estop = cmd->estop;
        s_motion.clear_fault = cmd->clear_fault;
        s_motion.source = cmd->source;
        s_motion.stamp_ms = cmd->stamp_ms;
        if (cmd->lqr_valid)
        {
            s_motion.lqr_enable = cmd->lqr_enable;
        }
        else
        {
            s_motion.lqr_enable = prev_lqr;
        }
        if (cmd->source == CMD_SRC_AI) s_ai_stamp = cmd->stamp_ms;
        if (cmd->source == CMD_SRC_RC) s_rc_stamp = cmd->stamp_ms;
    }
}

void cmd_arbiter_inject_direct(const motor_direct_cmd_t *cmd)
{
    if (!cmd) return;
    s_direct = *cmd;
    s_direct.active = true;
}

void cmd_arbiter_tick(uint32_t now_ms)
{
    if (s_motion.source == CMD_SRC_AI &&
        (now_ms - s_ai_stamp) > APP_CFG_AI_TIMEOUT_MS)
    {
        s_motion.vx_mps = 0.0f;
        s_motion.wz_radps = 0.0f;
        s_motion.brake = true;
        s_motion.source = CMD_SRC_NONE;
    }
    if (s_motion.source == CMD_SRC_RC &&
        (now_ms - s_rc_stamp) > APP_CFG_RC_TIMEOUT_MS)
    {
        s_motion.vx_mps = 0.0f;
        s_motion.wz_radps = 0.0f;
        s_motion.brake = true;
        s_motion.source = CMD_SRC_NONE;
    }
}

void cmd_arbiter_get_motion(motion_cmd_t *out)
{
    if (out) *out = s_motion;
}

bool cmd_arbiter_get_direct(motor_direct_cmd_t *out)
{
    if (!out) return false;
    *out = s_direct;
    const bool a = s_direct.active;
    s_direct.active = false;
    return a;
}
