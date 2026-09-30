/**
 * @file    max7219.h
 * @brief   MAX7219 7-segment driver (no-decode mode) on SPI2.
 */
#ifndef MAX7219_H
#define MAX7219_H

#include <stdint.h>

#define MAX7219_DIGITS          6U

/* Segment bits in no-decode mode: DP A B C D E F G = D7..D0 */
#define SEG_DP                  0x80U

void max7219_init(void);

/* Send the frame to the display. frame[0] is the leftmost digit.
 * Only digits that differ from the last written frame are sent, unless
 * force is non-zero. */
void max7219_write_frame(const uint8_t frame[MAX7219_DIGITS], int force);

/* Re-send the configuration registers. The MAX7219 is known to lose its
 * settings on EMI from triac switching, so this is called periodically. */
void max7219_refresh_config(void);

#endif /* MAX7219_H */
