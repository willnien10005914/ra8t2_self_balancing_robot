#include "control/pid.h"

void pid_init(pid_t *pid, float kp, float ki, float kd,
              float out_min, float out_max, float i_limit)
{
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->i_limit = (i_limit > 0.0f) ? i_limit : out_max;
    pid_reset(pid);
}

void pid_reset(pid_t *pid)
{
    if (!pid) return;
    pid->integrator = 0.0f;
    pid->prev_err = 0.0f;
    pid->has_prev = false;
}

void pid_set_gains(pid_t *pid, float kp, float ki, float kd)
{
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float pid_step(pid_t *pid, float error, float dt)
{
    if (!pid || dt <= 0.0f) return 0.0f;

    pid->integrator += error * dt * pid->ki;
    if (pid->integrator > pid->i_limit) pid->integrator = pid->i_limit;
    if (pid->integrator < -pid->i_limit) pid->integrator = -pid->i_limit;

    float deriv = 0.0f;
    if (pid->has_prev)
    {
        deriv = (error - pid->prev_err) / dt;
    }
    pid->prev_err = error;
    pid->has_prev = true;

    float u = pid->kp * error + pid->integrator + pid->kd * deriv;
    if (u > pid->out_max) u = pid->out_max;
    if (u < pid->out_min) u = pid->out_min;
    return u;
}
