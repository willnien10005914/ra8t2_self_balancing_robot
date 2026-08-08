#include "motor/motor_port.h"

#include <string.h>

/*
 * FSP port stubs. Replace body with RM_MOTOR_HALL_* after importing
 * MCK-RA8T2 Hall vector-control sample (dual instance).
 *
 * Example (pseudo):
 *   RM_MOTOR_HALL_Open(g_motor_hall_l.p_ctrl, g_motor_hall_l.p_cfg);
 *   RM_MOTOR_HALL_SpeedSet(g_motor_hall_l.p_ctrl, rpm);
 */

static motor_feedback_t s_fb;
static float s_rpm[2];
static int32_t s_pos_cmd[2];

bool motor_port_open(void)
{
    memset(&s_fb, 0, sizeof(s_fb));
    s_fb.alive[0] = s_fb.alive[1] = true;
    return true;
}

void motor_port_close(void)
{
}

bool motor_port_run(motor_id_t id)
{
    (void)id;
    return true;
}

bool motor_port_stop(motor_id_t id)
{
    if (id == MOTOR_ID_LEFT || id == MOTOR_ID_BOTH) s_rpm[0] = 0.0f;
    if (id == MOTOR_ID_RIGHT || id == MOTOR_ID_BOTH) s_rpm[1] = 0.0f;
    return true;
}

bool motor_port_speed_set(motor_id_t id, float rpm)
{
    /* Invert one side in hardware mapping if mechanical dirs oppose. */
    if (id == MOTOR_ID_LEFT || id == MOTOR_ID_BOTH) s_rpm[0] = rpm;
    if (id == MOTOR_ID_RIGHT || id == MOTOR_ID_BOTH) s_rpm[1] = -rpm; /* often mirrored */
    return true;
}

bool motor_port_position_set(motor_id_t id, int32_t counts)
{
    if (id == MOTOR_ID_LEFT || id == MOTOR_ID_BOTH) s_pos_cmd[0] = counts;
    if (id == MOTOR_ID_RIGHT || id == MOTOR_ID_BOTH) s_pos_cmd[1] = counts;
    return true;
}

bool motor_port_torque_set(motor_id_t id, float u)
{
    /* Map normalized torque to q-axis current when FSP exposes it. */
    (void)id;
    (void)u;
    return true;
}

void motor_port_read_fb(motor_feedback_t *fb)
{
    if (!fb) return;
    /* TODO: read hall counts / speed from FSP motor APIs. */
    s_fb.vel_radps[0] = s_rpm[0] * (6.2831853f / 60.0f);
    s_fb.vel_radps[1] = s_rpm[1] * (6.2831853f / 60.0f);
    s_fb.pos_rad[0] += s_fb.vel_radps[0] * 0.002f;
    s_fb.pos_rad[1] += s_fb.vel_radps[1] * 0.002f;
    s_fb.hall_counts[0] = (int32_t)(s_fb.pos_rad[0] * 1000.0f);
    s_fb.hall_counts[1] = (int32_t)(s_fb.pos_rad[1] * 1000.0f);
    *fb = s_fb;
}

void motor_port_fault_reset(void)
{
    s_fb.fault_flags = 0;
}
