/**
 * @file    main.c
 * @brief   Hot air rework station firmware, STM32F103C8 (Blue Pill).
 *
 * SystemInit() (vector table, reset clock state) is called by the startup
 * code before main(); clock_init() then switches to the PLL.
 */
#include "stm32f1xx.h"
#include "board.h"
#include "clock.h"
#include "adc.h"
#include "phase_control.h"
#include "config.h"
#include "station.h"
#include "ui.h"

#define DISPLAY_POWER_UP_MS     50U

int main(void)
{
    /* Keep the debugger connected while the core sleeps in WFI */
    DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP;

    clock_init();
    board_gpio_init();
    phase_init();
    adc_init();

    clock_delay_ms(DISPLAY_POWER_UP_MS);    /* Let the MAX7219 supply settle */
    ui_init();
    station_init(clock_millis());
    board_watchdog_init();

    uint32_t last_ctrl_ms = clock_millis();
    uint32_t last_disp_ms = last_ctrl_ms;

    for (;;) {
        uint32_t now = clock_millis();

        if ((uint32_t)(now - last_ctrl_ms) >= CONTROL_PERIOD_MS) {
            last_ctrl_ms = now;
            station_step(now);
        }

        if ((uint32_t)(now - last_disp_ms) >= DISPLAY_PERIOD_MS) {
            last_disp_ms = now;
            ui_update(station_status(), now);
        }

        board_watchdog_kick();
        __WFI();    /* Sleep until the next interrupt (SysTick at the latest) */
    }
}
