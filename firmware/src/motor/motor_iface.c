#include "motor/motor_iface.h"
#include "motor/motor_port.h"

#include <string.h>

static motor_mode_t s_mode[2] = { MOTOR_MODE_DISABLED, MOTOR_MODE_DISABLED };
static bool s_en[2];

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

bool motor_set_torque(motor_id_t id, float nm_or_norm)
{
    bool ok = true;
    if (id == MOTOR_ID_LEFT || id == MOTOR_ID_BOTH)
    {
        ok = motor_port_torque_set(MOTOR_ID_LEFT, nm_or_norm) && ok;
    }
    if (id == MOTOR_ID_RIGHT || id == MOTOR_ID_BOTH)
    {
        ok = motor_port_torque_set(MOTOR_ID_RIGHT, nm_or_norm) && ok;
    }
    return ok;
}

void motor_get_feedback(motor_feedback_t *fb)
{
    if (!fb) return;
    motor_port_read_fb(fb);
}

void motor_estop(void)
{
    (void)motor_port_stop(MOTOR_ID_BOTH);
    s_en[0] = s_en[1] = false;
    s_mode[0] = s_mode[1] = MOTOR_MODE_DISABLED;
}
