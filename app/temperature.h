/**
 * @file    temperature.h
 * @brief   Raw ADC value to temperature conversion: K-type thermocouple
 *          with cold junction compensation, NTC cold junction sensor.
 */
#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <stdbool.h>
#include <stdint.h>

/* Cold junction temperature from the NTC divider. Returns false (and
 * 25 C in *out_c) if the NTC is open or shorted. */
bool temperature_ntc_c(uint16_t adc_raw, float *out_c);

/* Hot junction temperature from the amplified thermocouple voltage and
 * the cold junction temperature. */
float temperature_thermocouple_c(uint16_t adc_raw, float cold_junction_c);

/* K-type conversions (NIST ITS-90), exposed for testing */
float ktype_mv_to_c(float mv);
float ktype_c_to_mv_cj(float c);

#endif /* TEMPERATURE_H */
