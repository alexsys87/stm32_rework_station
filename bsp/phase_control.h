/**
 * @file    phase_control.h
 * @brief   Mains zero-cross detection and phase-angle control of the
 *          heater and pump triacs.
 *
 * Timing: TIM3 runs at 1 MHz and is reset on every zero-cross edge.
 * For every channel with non-zero power, a compare interrupt turns the
 * gate on at the firing angle and a second match turns it off after the
 * gate pulse width. The firing angle is taken from a table so that the
 * requested percentage is proportional to the delivered power, not to
 * the phase angle.
 */
#ifndef PHASE_CONTROL_H
#define PHASE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PHASE_CH_HEATER = 0,
    PHASE_CH_PUMP   = 1,
    PHASE_CH_COUNT
} phase_ch_t;

void phase_init(void);

/* Power 0..100 %, applied from the next mains half-cycle */
void phase_set_power(phase_ch_t ch, uint8_t percent);

/* True if valid zero-cross pulses were seen during the last 100 ms */
bool phase_mains_present(void);

/* Last measured mains half period, microseconds (10000 for 50 Hz) */
uint16_t phase_half_period_us(void);

#endif /* PHASE_CONTROL_H */
