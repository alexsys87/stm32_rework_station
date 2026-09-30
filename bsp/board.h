/**
 * @file    board.h
 * @brief   Board support: pin map and hardware-level constants for the
 *          STM32F103C8 "Blue Pill" based hot air rework station.
 *
 * The PCB is not routed yet, so every pin assignment is collected here.
 * When moving a signal to another pin, keep the notes next to each group
 * in mind (ADC-capable pins, EXTI line/IRQ, 5 V tolerance, SPI2 AF pins).
 */
#ifndef BOARD_H
#define BOARD_H

#include "stm32f1xx.h"

/* =========================================================================
 *  Pin map
 * ========================================================================= */

/* ---- Analog inputs (ADC1, all on GPIOA, configured as analog) ----------
 *  Index in the scan sequence  | Pin | ADC channel | Signal
 *  ADC_IDX_PUMP_POT       (0)  | PA0 | IN0         | Air flow (pump) potentiometer
 *  ADC_IDX_TEMP_POT       (1)  | PA1 | IN1         | Temperature setpoint potentiometer
 *  ADC_IDX_THERMOCOUPLE   (2)  | PA2 | IN2         | Thermocouple amplifier output
 *  ADC_IDX_NTC            (3)  | PA3 | IN3         | Cold junction NTC divider
 */
#define ADC_IDX_PUMP_POT        0U
#define ADC_IDX_TEMP_POT        1U
#define ADC_IDX_THERMOCOUPLE    2U
#define ADC_IDX_NTC             3U
#define ADC_CHANNEL_COUNT       4U

#define ADC_GPIO_PORT           GPIOA
/* ADC channel number for each scan index (PAx == ADC_INx for x = 0..7) */
#define ADC_CH_PUMP_POT         0U
#define ADC_CH_TEMP_POT         1U
#define ADC_CH_THERMOCOUPLE     2U
#define ADC_CH_NTC              3U

/* ---- Work / standby input (handle cradle reed switch or toggle switch) --
 *  Input with internal pull-up. Contact closed to GND (low) = WORK,
 *  open (high) = STANDBY. */
#define CTRL_INPUT_PORT         GPIOA
#define CTRL_INPUT_PIN          4U

/* ---- Mains zero-cross detector (optocoupler, open collector) ----------
 *  Input with internal pull-up, falling edge interrupt.
 *  If the pin is moved, update ZC_EXTI_IRQn / ZC_IRQHandler too:
 *  lines 0..4 have their own IRQs, 5..9 share EXTI9_5, 10..15 share EXTI15_10. */
#define ZC_PORT                 GPIOA
#define ZC_PIN                  6U
#define ZC_EXTI_PORT_SEL        0U                  /* 0 = PA, 1 = PB, 2 = PC */
#define ZC_EXTI_IRQn            EXTI9_5_IRQn
#define ZC_IRQHandler           EXTI9_5_IRQHandler

/* ---- Triac drivers (MOC30xx random-phase opto-triac, active high) -----
 *  Add an external pull-down (10 kOhm) on each of these pins: GPIOs float
 *  during reset and while the MCU is being programmed. */
#define HEATER_TRIAC_PORT       GPIOB
#define HEATER_TRIAC_PIN        1U
#define PUMP_TRIAC_PORT         GPIOB
#define PUMP_TRIAC_PIN          10U

/* ---- MAX7219 display (SPI2 remap-free pins, transmit only) ------------
 *  PB13 = SCK, PB15 = MOSI are fixed by SPI2. PB14 (MISO) is not used.
 *  PB12..PB15 are 5 V tolerant, see MAX7219_OPEN_DRAIN below. */
#define MAX7219_SPI             SPI2
#define MAX7219_CS_PORT         GPIOB
#define MAX7219_CS_PIN          12U
#define MAX7219_SCK_PIN         13U
#define MAX7219_MOSI_PIN        15U

/* MAX7219 powered from 5 V needs VIH >= 3.5 V, which a 3.3 V push-pull
 * output does not guarantee. Set to 1 to drive CS/SCK/MOSI as open-drain
 * with external 1 kOhm pull-ups to 5 V (the pins are 5 V tolerant).
 * Set to 0 for push-pull (level shifter such as 74HCT125, or MAX7219 at 3.3 V). */
#define MAX7219_OPEN_DRAIN      0

/* ---- Status LED (on-board Blue Pill LED, active low) ------------------ */
#define LED_PORT                GPIOC
#define LED_PIN                 13U

/* =========================================================================
 *  Hardware-level constants
 * ========================================================================= */

#define ADC_FULL_SCALE          4095.0f
#define ADC_VREF_MV             3300.0f  /* VDDA, measure and adjust for accuracy */

/* Thermocouple amplifier: non-inverting op-amp, gain = 1 + Rf / Rg.
 * Default: Rf = 100 kOhm, Rg = 1 kOhm -> 101 (full scale ~785 C). */
#define TC_AMP_GAIN             101.0f
#define TC_AMP_OFFSET_MV        0.0f     /* Amplifier output with shorted input */

/* Cold junction NTC: NTC from PA3 to GND, fixed resistor from PA3 to 3.3 V */
#define NTC_PULLUP_OHM          10000.0f
#define NTC_R25_OHM             10000.0f
#define NTC_BETA                3950.0f

/* Triac gate pulse widths, microseconds */
#define HEATER_GATE_PULSE_US    100U     /* Resistive load */
#define PUMP_GATE_PULSE_US      1000U    /* Inductive load needs a longer pulse */

/* =========================================================================
 *  Helpers
 * ========================================================================= */

#define PIN_MASK(pin)           (1UL << (pin))

static inline void gpio_set(GPIO_TypeDef *port, uint32_t pin)   { port->BSRR = PIN_MASK(pin); }
static inline void gpio_reset(GPIO_TypeDef *port, uint32_t pin) { port->BRR  = PIN_MASK(pin); }
static inline uint32_t gpio_read(const GPIO_TypeDef *port, uint32_t pin)
{
    return (port->IDR >> pin) & 1UL;
}

/* Force both triac gates off. Safe to call from any context, including
 * fault handlers: it touches only GPIO registers. */
static inline void board_triacs_off(void)
{
    gpio_reset(HEATER_TRIAC_PORT, HEATER_TRIAC_PIN);
    gpio_reset(PUMP_TRIAC_PORT, PUMP_TRIAC_PIN);
}

static inline void board_led(int on)
{
    if (on) gpio_reset(LED_PORT, LED_PIN);   /* Active low */
    else    gpio_set(LED_PORT, LED_PIN);
}

static inline void board_led_toggle(void) { LED_PORT->ODR ^= PIN_MASK(LED_PIN); }

/* 1 = WORK (contact closed to GND), 0 = STANDBY */
static inline int board_ctrl_input_active(void)
{
    return gpio_read(CTRL_INPUT_PORT, CTRL_INPUT_PIN) == 0U;
}

void board_gpio_init(void);
void board_watchdog_init(void);

/* Reload the independent watchdog */
static inline void board_watchdog_kick(void) { IWDG->KR = 0xAAAAU; }

#endif /* BOARD_H */
