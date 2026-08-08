#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float i_limit;     /**< Absolute integral clamp. */
    float out_min;
    float out_max;
    float integrator;
    float prev_err;
    bool  has_prev;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd,
              float out_min, float out_max, float i_limit);
void pid_reset(pid_t *pid);
void pid_set_gains(pid_t *pid, float kp, float ki, float kd);
float pid_step(pid_t *pid, float error, float dt);

#ifdef __cplusplus
}
#endif
