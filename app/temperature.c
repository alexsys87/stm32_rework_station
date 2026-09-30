/**
 * @file    temperature.c
 * @brief   Thermocouple and NTC math.
 *
 * Cold junction compensation is done in the voltage domain, which is the
 * correct way for a non-linear thermocouple:
 *   E_total = E_measured + E(T_cold);  T_hot = E^-1(E_total)
 */
#include "temperature.h"
#include <math.h>
#include "board.h"

#define KELVIN_0C               273.15f
#define NTC_T25_K               298.15f
#define NTC_ADC_MIN             20U
#define NTC_ADC_MAX             4075U

#define KTYPE_RANGE1_MAX_MV     20.644f     /* 500 C */

/* NIST ITS-90 inverse polynomial, 0..500 C, error < 0.04 C */
static const float s_inv_0_500[] = {
    0.0f, 2.508355e+01f, 7.860106e-02f, -2.503131e-01f, 8.315270e-02f,
    -1.228034e-02f, 9.804036e-04f, -4.413030e-05f, 1.057734e-06f, -1.052755e-08f,
};

/* NIST ITS-90 inverse polynomial, 500..1372 C, error < 0.06 C */
static const float s_inv_500_1372[] = {
    -1.318058e+02f, 4.830222e+01f, -1.646031e+00f, 5.464731e-02f,
    -9.650715e-04f, 8.802193e-06f, -3.110810e-08f,
};

static float poly(const float *c, uint32_t n, float x)
{
    float y = c[n - 1U];
    for (uint32_t i = n - 1U; i > 0U; i--) {
        y = y * x + c[i - 1U];
    }
    return y;
}

float ktype_mv_to_c(float mv)
{
    if (mv < 0.0f) {
        return mv * 25.08355f;          /* Linear below 0 C, good enough here */
    }
    if (mv < KTYPE_RANGE1_MAX_MV) {
        return poly(s_inv_0_500, sizeof s_inv_0_500 / sizeof s_inv_0_500[0], mv);
    }
    return poly(s_inv_500_1372, sizeof s_inv_500_1372 / sizeof s_inv_500_1372[0], mv);
}

/* Quadratic fit of the NIST forward function for -20..85 C
 * (cold junction range), error < 7 uV (0.2 C). */
float ktype_c_to_mv_cj(float c)
{
    return 0.0019475f + c * (0.039590179f + c * 1.5692480e-05f);
}

bool temperature_ntc_c(uint16_t adc_raw, float *out_c)
{
    if (adc_raw < NTC_ADC_MIN || adc_raw > NTC_ADC_MAX) {
        *out_c = 25.0f;
        return false;
    }
    /* Ratiometric: NTC to GND, pull-up to VDDA, reference independent */
    float r = NTC_PULLUP_OHM * (float)adc_raw / (ADC_FULL_SCALE - (float)adc_raw);
    float inv_t = logf(r / NTC_R25_OHM) / NTC_BETA + 1.0f / NTC_T25_K;
    *out_c = 1.0f / inv_t - KELVIN_0C;
    return true;
}

float temperature_thermocouple_c(uint16_t adc_raw, float cold_junction_c)
{
    float out_mv = (float)adc_raw * (ADC_VREF_MV / ADC_FULL_SCALE);
    float tc_mv = (out_mv - TC_AMP_OFFSET_MV) / TC_AMP_GAIN;
    return ktype_mv_to_c(tc_mv + ktype_c_to_mv_cj(cold_junction_c));
}
