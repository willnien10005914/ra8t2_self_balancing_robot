#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_ID_LEFT = 0,
    MOTOR_ID_RIGHT = 1,
    MOTOR_ID_BOTH = 2
} motor_id_t;

typedef enum {
    MOTOR_MODE_DISABLED = 0,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_POSITION,
    MOTOR_MODE_TORQUE
} motor_mode_t;

typedef struct {
    float pos_rad[2];
    float vel_radps[2];
    float current_a[2];
    int32_t hall_counts[2];
    uint16_t fault_flags;
    bool alive[2];
} motor_feedback_t;

bool motor_init(void);
bool motor_enable(motor_id_t id, bool en);
bool motor_set_mode(motor_id_t id, motor_mode_t mode);
bool motor_set_speed_rpm(motor_id_t id, float rpm);
bool motor_set_position_counts(motor_id_t id, int32_t counts);
bool motor_set_torque(motor_id_t id, float nm_or_norm);
void motor_get_feedback(motor_feedback_t *fb);
void motor_estop(void);

#ifdef __cplusplus
}
#endif
