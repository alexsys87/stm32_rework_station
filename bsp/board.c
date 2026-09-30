/**
 * @file    board.c
 * @brief   GPIO configuration and independent watchdog.
 */
#include "board.h"

/* STM32F1 per-pin configuration nibble: CNF[1:0] | MODE[1:0] */
#define GPIO_CFG_ANALOG         0x0U
#define GPIO_CFG_INPUT_PULL     0x8U    /* Pull direction is taken from ODR */
#define GPIO_CFG_OUT_PP_2MHZ    0x2U
#define GPIO_CFG_OUT_PP_10MHZ   0x1U
#define GPIO_CFG_OUT_OD_10MHZ   0x5U
#define GPIO_CFG_AF_PP_10MHZ    0x9U
#define GPIO_CFG_AF_OD_10MHZ    0xDU

static void gpio_config(GPIO_TypeDef *port, uint32_t pin, uint32_t cfg)
{
    volatile uint32_t *reg = (pin < 8U) ? &port->CRL : &port->CRH;
    uint32_t shift = (pin & 7U) * 4U;
    *reg = (*reg & ~(0xFUL << shift)) | (cfg << shift);
}

void board_gpio_init(void)
{
    /* Output levels are set before the pins are switched to output mode,
     * so that the triacs never see a glitch after reset. */
    board_triacs_off();
    gpio_set(MAX7219_CS_PORT, MAX7219_CS_PIN);
    board_led(0);

    /* Analog inputs */
    gpio_config(ADC_GPIO_PORT, ADC_CH_PUMP_POT, GPIO_CFG_ANALOG);
    gpio_config(ADC_GPIO_PORT, ADC_CH_TEMP_POT, GPIO_CFG_ANALOG);
    gpio_config(ADC_GPIO_PORT, ADC_CH_THERMOCOUPLE, GPIO_CFG_ANALOG);
    gpio_config(ADC_GPIO_PORT, ADC_CH_NTC, GPIO_CFG_ANALOG);

    /* Digital inputs with pull-up */
    gpio_set(CTRL_INPUT_PORT, CTRL_INPUT_PIN);
    gpio_config(CTRL_INPUT_PORT, CTRL_INPUT_PIN, GPIO_CFG_INPUT_PULL);
    gpio_set(ZC_PORT, ZC_PIN);
    gpio_config(ZC_PORT, ZC_PIN, GPIO_CFG_INPUT_PULL);

    /* Triac drivers: slow edges are enough and reduce EMI */
    gpio_config(HEATER_TRIAC_PORT, HEATER_TRIAC_PIN, GPIO_CFG_OUT_PP_2MHZ);
    gpio_config(PUMP_TRIAC_PORT, PUMP_TRIAC_PIN, GPIO_CFG_OUT_PP_2MHZ);

    /* MAX7219: CS as GPIO, SCK/MOSI as SPI2 alternate function */
#if MAX7219_OPEN_DRAIN
    gpio_config(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_CFG_OUT_OD_10MHZ);
    gpio_config(GPIOB, MAX7219_SCK_PIN, GPIO_CFG_AF_OD_10MHZ);
    gpio_config(GPIOB, MAX7219_MOSI_PIN, GPIO_CFG_AF_OD_10MHZ);
#else
    gpio_config(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_CFG_OUT_PP_10MHZ);
    gpio_config(GPIOB, MAX7219_SCK_PIN, GPIO_CFG_AF_PP_10MHZ);
    gpio_config(GPIOB, MAX7219_MOSI_PIN, GPIO_CFG_AF_PP_10MHZ);
#endif

    /* Status LED */
    gpio_config(LED_PORT, LED_PIN, GPIO_CFG_OUT_PP_2MHZ);
}

void board_watchdog_init(void)
{
    /* Keep the watchdog frozen while the core is halted by a debugger */
    DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;

    /* LSI ~40 kHz / 32 = 1.25 kHz, reload 500 -> ~400 ms timeout
     * (LSI varies 30..60 kHz, so the real timeout is 270..530 ms) */
    IWDG->KR  = 0x5555U;    /* Unlock PR and RLR */
    IWDG->PR  = 3U;         /* Prescaler /32 */
    IWDG->RLR = 500U;
    IWDG->KR  = 0xAAAAU;    /* Reload */
    IWDG->KR  = 0xCCCCU;    /* Start */
}
