/**
 * @file    fault_handlers.c
 * @brief   Cortex-M3 exception handlers.
 *
 * Every fatal exception turns both triac gates off first: a gate left on
 * would keep the heater running at full power until the watchdog resets
 * the MCU. The handlers then spin; the independent watchdog (~0.4 s)
 * performs the reset.
 */
#include "board.h"

static void fatal_stop(void)
{
    __disable_irq();
    board_triacs_off();
    for (;;) {
    }
}

/* NMI is raised by the clock security system when the HSE crystal fails.
 * The core is switched to HSI 8 MHz and all timing becomes wrong, so shut
 * the outputs off and restart (clock_init() then falls back to HSI). */
void NMI_Handler(void)
{
    board_triacs_off();
    if (RCC->CIR & RCC_CIR_CSSF) {
        RCC->CIR = RCC_CIR_CSSC;
        NVIC_SystemReset();
    }
}

void HardFault_Handler(void)  { fatal_stop(); }
void MemManage_Handler(void)  { fatal_stop(); }
void BusFault_Handler(void)   { fatal_stop(); }
void UsageFault_Handler(void) { fatal_stop(); }

void SVC_Handler(void)      {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void)   {}
