#include "motor/motor_iface.h"
#include "motor/motor_port.h"
#include "motor/motor_params.h"

#include <string.h>

static motor_mode_t s_mode[2] = { MOTOR_MODE_DISABLED, MOTOR_MODE_DISABLED };
static bool s_en[2];

static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

float motor_tau_to_iq_a(float tau_nm)
{
    float iq = MOTOR_SPEC_IQ_FROM_TAU(tau_nm);
    return clampf(iq, -MOTOR_SPEC_PEAK_CURRENT_A, MOTOR_SPEC_PEAK_CURRENT_A);
}

bool motor_init(void)
{
    memset(s_mode, 0, sizeof(s_mode));
    s_en[0] = s_en[1] = false;
    return motor_port_open();
}

bool motor_enable(motor_id_t id, bool en)
{
    if (id == MOTOR_ID_LEFT || id == MOTOR_ID_BOTH)
    {
        s_en[0] = en;
        if (en) (void)motor_port_run(MOTOR_ID_LEFT);
        else (void)motor_port_stop(MOTOR_ID_LEFT);
    }
    if (id == MOTOR_ID_RIGHT || id == MOTOR_ID_BOTH)
    {
        s_en[1] = en;
        if (en) (void)motor_port_run(MOTOR_ID_RIGHT);
        else (void)motor_port_stop(MOTOR_ID_RIGHT);
    }
    return true;
}

bool motor_set_mode(motor_id_t id, motor_mode_t mode)
{
    if (id == MOTOR_ID_LEFT || id == MOTOR_ID_BOTH) s_mode[0] = mode;
    if (id == MOTOR_ID_RIGHT || id == MOTOR_ID_BOTH) s_mode[1] = mode;
    if (mode != MOTOR_MODE_DISABLED)
    {
        return motor_enable(id, true);
    }
    return motor_enable(id, false);
}

bool motor_set_speed_rpm(motor_id_t id, float rpm)
{
    /* 安全鎖：換算不得超過 APP_CFG_VX_MAX_MPS 對應的 rpm */
    if (rpm > APP_CFG_RPM_CMD_MAX) rpm = APP_CFG_RPM_CMD_MAX;
    if (rpm < -APP_CFG_RPM_CMD_MAX) rpm = -APP_CFG_RPM_CMD_MAX;
    bool ok = true;
    if (id == MOTOR_ID_LEFT || id == MOTOR_ID_BOTH)
    {
        ok = motor_port_speed_set(MOTOR_ID_LEFT, rpm) && ok;
    }
    if (id == MOTOR_ID_RIGHT || id == MOTOR_ID_BOTH)
    {
        ok = motor_port_speed_set(MOTOR_ID_RIGHT, rpm) && ok;
    }
    return ok;
}

bool motor_set_position_counts(motor_id_t id, int32_t counts)
{
    bool ok = true;
    if (id == MOTOR_ID_LEFT || id == MOTOR_ID_BOTH)
    {
        ok = motor_port_position_set(MOTOR_ID_LEFT, counts) && ok;
    }
    if (id == MOTOR_ID_RIGHT || id == MOTOR_ID_BOTH)
    {
        ok = motor_port_position_set(MOTOR_ID_RIGHT, counts) && ok;
    }
    return ok;
}

bool motor_set_torque_nm(motor_id_t id, float tau_nm)
{
    const float iq = motor_tau_to_iq_a(tau_nm);
    bool ok = true;
    if (id == MOTOR_ID_LEFT || id == MOTOR_ID_BOTH)
    {
        ok = motor_port_torque_set(MOTOR_ID_LEFT, iq) && ok;
    }
    if (id == MOTOR_ID_RIGHT || id == MOTOR_ID_BOTH)
    {
        ok = motor_port_torque_set(MOTOR_ID_RIGHT, iq) && ok;
    }
    return ok;
}

bool motor_set_torque(motor_id_t id, float nm_or_norm)
{
    return motor_set_torque_nm(id, nm_or_norm);
}

void motor_get_feedback(motor_feedback_t *fb)
{
    if (!fb) return;
    motor_port_read_fb(fb);
}

void motor_estop(void)
{
    (void)motor_port_stop(MOTOR_ID_BOTH);
    (void)motor_port_torque_set(MOTOR_ID_BOTH, 0.0f);
    s_en[0] = s_en[1] = false;
    s_mode[0] = s_mode[1] = MOTOR_MODE_DISABLED;
}
