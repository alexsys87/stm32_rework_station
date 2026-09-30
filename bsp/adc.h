/**
 * @file    adc.h
 * @brief   Continuous ADC1 scan of all analog inputs via circular DMA.
 *
 * The DMA interrupt accumulates every sample; adc_read_average() returns
 * the mean over all samples since the previous call. Called every 50 ms
 * it averages ~600 samples per channel over 2.5 mains periods, which
 * removes 50/100 Hz ripple and triac switching noise.
 */
#ifndef ADC_H
#define ADC_H

#include <stdbool.h>
#include <stdint.h>
#include "board.h"

void adc_init(void);

/* Fill out[ADC_CHANNEL_COUNT] with averaged raw values (0..4095).
 * Returns false if no new samples arrived since the last call
 * (ADC/DMA stalled); out[] is left unchanged in that case. */
bool adc_read_average(uint16_t out[ADC_CHANNEL_COUNT]);

#endif /* ADC_H */
