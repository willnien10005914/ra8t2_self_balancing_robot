#include "safety/safety.h"
#include "app/app_cfg.h"
#include "motor/motor_params.h"

#include <string.h>

#ifndef SAFETY_UV_V
#define SAFETY_UV_V   (30.0f)
#endif
#ifndef SAFETY_OV_V
#define SAFETY_OV_V   (43.0f)
#endif
#ifndef SAFETY_OC_A
#define SAFETY_OC_A   (25.0f)
#endif
#ifndef SAFETY_OT_C
#define SAFETY_OT_C   (70.0f)
#endif
#ifndef SAFETY_OC_MOTOR_A
#define SAFETY_OC_MOTOR_A (MOTOR_SPEC_PEAK_CURRENT_A)
#endif

static uint16_t s_fault;
static safety_sensors_t s_last;

__attribute__((weak)) bool safety_port_read(safety_sensors_t *out)
{
    if (!out) return false;
    /* Defaults allow desktop builds; board must override with ADC. */
    out->vbus_v = MOTOR_SPEC_RATED_VOLTAGE_V;
    out->ibus_a = 0.0f;
    out->temp_c = 25.0f;
    out->i_motor_l_a = 0.0f;
    out->i_motor_r_a = 0.0f;
    out->valid = true;
    return true;
}

__attribute__((weak)) void safety_port_hw_shutdown(void)
{
    /* Wire to POEG / GPT output disable on MCK-RA8T2. */
}

void safety_init(void)
{
    s_fault = SAFETY_OK;
    memset(&s_last, 0, sizeof(s_last));
}

uint16_t safety_fault_mask(void)
{
    return s_fault;
}

bool safety_ok(void)
{
    return s_fault == SAFETY_OK;
}

void safety_clear_faults(void)
{
    s_fault = SAFETY_OK;
}

uint16_t safety_update(void)
{
    safety_sensors_t s;
    if (!safety_port_read(&s) || !s.valid)
    {
        s_fault |= SAFETY_FAULT_MOTOR;
        safety_port_hw_shutdown();
        return s_fault;
    }
    s_last = s;

    if (s.vbus_v < SAFETY_UV_V)
    {
        s_fault |= SAFETY_FAULT_UV;
    }
    if (s.vbus_v > SAFETY_OV_V)
    {
        s_fault |= SAFETY_FAULT_OV;
    }
    if (s.ibus_a > SAFETY_OC_A || s.ibus_a < -SAFETY_OC_A)
    {
        s_fault |= SAFETY_FAULT_OC;
    }
    if (s.temp_c > SAFETY_OT_C)
    {
        s_fault |= SAFETY_FAULT_OT;
    }
    if (s.i_motor_l_a > SAFETY_OC_MOTOR_A || s.i_motor_l_a < -SAFETY_OC_MOTOR_A ||
        s.i_motor_r_a > SAFETY_OC_MOTOR_A || s.i_motor_r_a < -SAFETY_OC_MOTOR_A)
    {
        s_fault |= SAFETY_FAULT_OC;
    }

    if (s_fault != SAFETY_OK)
    {
        safety_port_hw_shutdown();
    }
    return s_fault;
}
