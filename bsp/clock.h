/**
 * @file    clock.h
 * @brief   System clock (PLL) setup, peripheral clock enables and the
 *          1 ms SysTick time base.
 */
#ifndef CLOCK_H
#define CLOCK_H

#include <stdbool.h>
#include <stdint.h>

/* Configure SYSCLK from PLL and enable the peripheral clocks used by the
 * firmware. Returns true if the 8 MHz crystal (HSE) is used, false if the
 * code fell back to the internal RC oscillator (HSI). */
bool clock_init(void);

/* Frequency of the APB1 timers (TIM2..TIM4) in Hz */
uint32_t clock_apb1_timer_hz(void);

/* Milliseconds since start-up (wraps after ~49 days, use differences) */
uint32_t clock_millis(void);

/* Blocking delay, milliseconds (requires clock_init) */
void clock_delay_ms(uint32_t ms);

#endif /* CLOCK_H */
