/**
 * @file    clock.c
 * @brief   System clock and SysTick time base.
 *
 * Normal mode: HSE 8 MHz x9 -> 72 MHz. If the crystal does not start,
 * the clock falls back to HSI/2 x16 -> 64 MHz. All timing in the firmware
 * is derived from SystemCoreClock, so both variants work.
 *
 *   SYSCLK = HCLK = 72 MHz, APB1 = 36 MHz (timers 72 MHz),
 *   APB2 = 72 MHz, ADC = 12 MHz (limit 14 MHz)
 */
#include "clock.h"
#include "stm32f1xx.h"

#define HSE_STARTUP_TIMEOUT     0x20000UL
#define FLASH_LATENCY_2WS       (2UL << FLASH_ACR_LATENCY_Pos)

static volatile uint32_t s_millis;

void SysTick_Handler(void)
{
    s_millis++;
}

bool clock_init(void)
{
    bool hse_ok = false;

    /* 2 wait states are required above 48 MHz. Note: FLASH_ACR_LATENCY_2
     * from the CMSIS header is a single bit (value 4), not "2 wait states". */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_LATENCY_2WS;

    RCC->CR |= RCC_CR_HSEON;
    for (uint32_t i = 0; i < HSE_STARTUP_TIMEOUT; i++) {
        if (RCC->CR & RCC_CR_HSERDY) {
            hse_ok = true;
            break;
        }
    }

    uint32_t cfgr = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 |
                    RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_ADCPRE_DIV6;
    if (hse_ok) {
        cfgr |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;    /* 8 MHz * 9 = 72 MHz */
    } else {
        RCC->CR &= ~RCC_CR_HSEON;
        cfgr |= RCC_CFGR_PLLMULL16;                     /* 4 MHz * 16 = 64 MHz */
    }
    RCC->CFGR = cfgr;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
    }

    RCC->CFGR = cfgr | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }

    /* Clock security system: on crystal failure an NMI is raised
     * (see NMI_Handler), which shuts the triacs off and resets the MCU. */
    if (hse_ok) {
        RCC->CR |= RCC_CR_CSSON;
    }

    SystemCoreClockUpdate();

    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
                    RCC_APB2ENR_AFIOEN | RCC_APB2ENR_ADC1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_SPI2EN;
    (void)RCC->APB1ENR;     /* Make sure the enables are done before use */

    SysTick_Config(SystemCoreClock / 1000U);

    return hse_ok;
}

uint32_t clock_apb1_timer_hz(void)
{
    /* Timer clock is doubled when the APB1 prescaler is not 1 */
    uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    uint32_t pclk1 = SystemCoreClock >> APBPrescTable[ppre1];
    return (ppre1 < 4U) ? pclk1 : pclk1 * 2U;
}

uint32_t clock_millis(void)
{
    return s_millis;
}

void clock_delay_ms(uint32_t ms)
{
    uint32_t start = s_millis;
    while ((uint32_t)(s_millis - start) < ms) {
    }
}
