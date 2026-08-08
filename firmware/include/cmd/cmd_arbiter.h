#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CMD_SRC_NONE = 0,
    CMD_SRC_CONSOLE,
    CMD_SRC_AI,
    CMD_SRC_RC
} cmd_source_t;

typedef struct {
    float vx_mps;
    float wz_radps;
    bool  brake;
    bool  lqr_enable;
    bool  lqr_valid;     /**< If false, leave previous lqr_enable. */
    bool  estop;
    bool  clear_fault;
    cmd_source_t source;
    uint32_t stamp_ms;
} motion_cmd_t;

/** Direct per-wheel requests (only when LQR off). */
typedef struct {
    bool active;
    bool set_mode;
    uint8_t mode; /* motor_mode_t */
    bool set_speed;
    float speed_rpm;
    bool set_pos;
    int32_t pos_counts;
    uint8_t side; /* 0 L 1 R 2 BOTH */
} motor_direct_cmd_t;

void cmd_arbiter_init(void);
void cmd_arbiter_inject_motion(const motion_cmd_t *cmd);
void cmd_arbiter_inject_direct(const motor_direct_cmd_t *cmd);
void cmd_arbiter_tick(uint32_t now_ms);
void cmd_arbiter_get_motion(motion_cmd_t *out);
bool cmd_arbiter_get_direct(motor_direct_cmd_t *out);
bool cmd_arbiter_ai_enabled(void);
bool cmd_arbiter_rc_enabled(void);
void cmd_arbiter_set_ai_enabled(bool en);
void cmd_arbiter_set_rc_enabled(bool en);

#ifdef __cplusplus
}
#endif
