#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "motor/motor_iface.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Board / FSP adaptation layer.
 * Implemented against RM_MOTOR_HALL_* on MCK-RA8T2.
 */
bool motor_port_open(void);
void motor_port_close(void);
bool motor_port_run(motor_id_t id);
bool motor_port_stop(motor_id_t id);
bool motor_port_speed_set(motor_id_t id, float rpm);
bool motor_port_position_set(motor_id_t id, int32_t counts);
bool motor_port_torque_set(motor_id_t id, float u);
void motor_port_read_fb(motor_feedback_t *fb);
void motor_port_fault_reset(void);

#ifdef __cplusplus
}
#endif
