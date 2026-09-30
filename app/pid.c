/**
 * @file    pid.c
 * @brief   PID controller.
 */
#include "pid.h"

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, float dt,
              float d_filter, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->d_filter = d_filter;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid_reset(pid);
}

void pid_reset(pid_ctrl_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_input = 0.0f;
    pid->d_state = 0.0f;
    pid->primed = false;
}

float pid_update(pid_ctrl_t *pid, float setpoint, float input)
{
    float error = setpoint - input;

    /* Derivative on measurement: no kick when the setpoint knob moves */
    if (!pid->primed) {
        pid->prev_input = input;
        pid->primed = true;
    }
    float d_raw = -(input - pid->prev_input) / pid->dt;
    pid->prev_input = input;
    pid->d_state += pid->d_filter * (d_raw - pid->d_state);

    float p = pid->kp * error;
    float d = pid->kd * pid->d_state;
    float out = p + pid->integral + d;

    /* Anti-windup: integrate only if the output is not saturated in the
     * direction the error would push it further. */
    bool sat_high = (out >= pid->out_max) && (error > 0.0f);
    bool sat_low  = (out <= pid->out_min) && (error < 0.0f);
    if (!sat_high && !sat_low) {
        pid->integral = clampf(pid->integral + pid->ki * error * pid->dt,
                               pid->out_min, pid->out_max);
        out = p + pid->integral + d;
    }

    return clampf(out, pid->out_min, pid->out_max);
}
