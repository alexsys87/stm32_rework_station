/**
 * @file    pid.h
 * @brief   PID controller with derivative on measurement, filtered
 *          derivative and conditional-integration anti-windup.
 */
#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    float kp, ki, kd;       /* Gains, ki and kd in seconds */
    float dt;               /* Sample period, s */
    float d_filter;         /* Derivative low-pass factor, 0..1 (1 = no filter) */
    float out_min, out_max;

    float integral;
    float prev_input;
    float d_state;
    bool  primed;
} pid_ctrl_t;

void  pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, float dt,
               float d_filter, float out_min, float out_max);
void  pid_reset(pid_ctrl_t *pid);
float pid_update(pid_ctrl_t *pid, float setpoint, float input);

#endif /* PID_H */
