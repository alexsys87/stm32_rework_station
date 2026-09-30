/**
 * @file    max7219.c
 * @brief   MAX7219 driver: 16-bit SPI frames, transmit-only SPI2.
 */
#include "max7219.h"
#include "board.h"

#define REG_DIGIT0              0x01U
#define REG_DECODE_MODE         0x09U
#define REG_INTENSITY           0x0AU
#define REG_SCAN_LIMIT          0x0BU
#define REG_SHUTDOWN            0x0CU
#define REG_DISPLAY_TEST        0x0FU

#define DISPLAY_INTENSITY       0x08U   /* 0..15 */

/* Digit register used for frame[0] (leftmost). The original wiring has
 * DIG5 on the left; change to 0 and flip the index if DIG0 is leftmost. */
#define LEFTMOST_DIGIT          (MAX7219_DIGITS - 1U)

static uint8_t s_shadow[MAX7219_DIGITS];

static void write_reg(uint8_t reg, uint8_t data)
{
    SPI_TypeDef *spi = MAX7219_SPI;

    gpio_reset(MAX7219_CS_PORT, MAX7219_CS_PIN);
    spi->DR = ((uint16_t)reg << 8) | data;
    while (!(spi->SR & SPI_SR_TXE)) {
    }
    while (spi->SR & SPI_SR_BSY) {
    }
    gpio_set(MAX7219_CS_PORT, MAX7219_CS_PIN);  /* Data latched on CS rising edge */
}

void max7219_refresh_config(void)
{
    write_reg(REG_DISPLAY_TEST, 0x00U);
    write_reg(REG_DECODE_MODE, 0x00U);
    write_reg(REG_SCAN_LIMIT, MAX7219_DIGITS - 1U);
    write_reg(REG_INTENSITY, DISPLAY_INTENSITY);
    write_reg(REG_SHUTDOWN, 0x01U);
}

void max7219_init(void)
{
    /* Master, 16-bit frames, CPOL = 0 / CPHA = 0, fPCLK1/8 = 4.5 MHz
     * (MAX7219 max 10 MHz; use /32 with open-drain outputs).
     * One-line bidirectional mode with output enabled = transmit only,
     * so MISO is not needed and no RX overrun is generated. */
    uint32_t br = MAX7219_OPEN_DRAIN ? (SPI_CR1_BR_2) : (SPI_CR1_BR_1);
    MAX7219_SPI->CR1 = SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_DFF |
                       SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | br;
    MAX7219_SPI->CR1 |= SPI_CR1_SPE;

    max7219_refresh_config();

    static const uint8_t blank[MAX7219_DIGITS] = {0};
    max7219_write_frame(blank, 1);
}

void max7219_write_frame(const uint8_t frame[MAX7219_DIGITS], int force)
{
    for (uint32_t i = 0; i < MAX7219_DIGITS; i++) {
        if (force || frame[i] != s_shadow[i]) {
            s_shadow[i] = frame[i];
            write_reg((uint8_t)(REG_DIGIT0 + LEFTMOST_DIGIT - i), frame[i]);
        }
    }
}
