/**
 * @file    adc.c
 * @brief   ADC1 continuous scan + DMA1 channel 1 (circular, half/full IRQ).
 */
#include "adc.h"

/* Scan sequences per DMA half buffer. One sequence (4 channels at
 * 239.5 + 12.5 cycles, 12 MHz ADC clock) takes 84 us, so a half buffer
 * is ready every ~2.7 ms. */
#define SEQ_PER_HALF            32U
#define DMA_HALF_LEN            (SEQ_PER_HALF * ADC_CHANNEL_COUNT)
#define DMA_BUF_LEN             (DMA_HALF_LEN * 2U)

/* Sample time code 7 = 239.5 cycles: high source impedance is fine */
#define ADC_SMP_239_5           7U

static volatile uint16_t s_dma_buf[DMA_BUF_LEN];
static volatile uint32_t s_acc[ADC_CHANNEL_COUNT];
static volatile uint32_t s_acc_count;

void adc_init(void)
{
    /* Power the ADC up and wait tSTAB (1 us) before calibration.
     * At this point the prescaler is already set in clock_init(). */
    ADC1->CR2 = ADC_CR2_ADON;
    for (volatile uint32_t i = 0; i < 1000U; i++) {
    }

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while (ADC1->CR2 & ADC_CR2_RSTCAL) {
    }
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL) {
    }

    /* Regular sequence: 4 channels in scan-index order */
    ADC1->CR1   = ADC_CR1_SCAN;
    ADC1->SMPR2 = (ADC_SMP_239_5 << (3U * ADC_CH_PUMP_POT)) |
                  (ADC_SMP_239_5 << (3U * ADC_CH_TEMP_POT)) |
                  (ADC_SMP_239_5 << (3U * ADC_CH_THERMOCOUPLE)) |
                  (ADC_SMP_239_5 << (3U * ADC_CH_NTC));
    ADC1->SQR1  = (ADC_CHANNEL_COUNT - 1U) << ADC_SQR1_L_Pos;
    ADC1->SQR3  = (ADC_CH_PUMP_POT     << (5U * ADC_IDX_PUMP_POT)) |
                  (ADC_CH_TEMP_POT     << (5U * ADC_IDX_TEMP_POT)) |
                  (ADC_CH_THERMOCOUPLE << (5U * ADC_IDX_THERMOCOUPLE)) |
                  (ADC_CH_NTC          << (5U * ADC_IDX_NTC));

    DMA1_Channel1->CCR   = 0;
    DMA1_Channel1->CPAR  = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR  = (uint32_t)s_dma_buf;
    DMA1_Channel1->CNDTR = DMA_BUF_LEN;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    DMA1_Channel1->CCR = DMA_CCR_PL_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 |
                         DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_HTIE | DMA_CCR_TCIE |
                         DMA_CCR_EN;

    NVIC_SetPriority(DMA1_Channel1_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* SWSTART only starts a conversion when the external trigger is
     * enabled and EXTSEL = 111 (SWSTART). */
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA |
                ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL;
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

static void accumulate(uint32_t offset)
{
    uint32_t sum[ADC_CHANNEL_COUNT] = {0};
    const volatile uint16_t *p = &s_dma_buf[offset];

    for (uint32_t i = 0; i < SEQ_PER_HALF; i++) {
        for (uint32_t ch = 0; ch < ADC_CHANNEL_COUNT; ch++) {
            sum[ch] += *p++;
        }
    }
    for (uint32_t ch = 0; ch < ADC_CHANNEL_COUNT; ch++) {
        s_acc[ch] += sum[ch];
    }
    s_acc_count += SEQ_PER_HALF;
}

void DMA1_Channel1_IRQHandler(void)
{
    uint32_t isr = DMA1->ISR;
    DMA1->IFCR = DMA_IFCR_CGIF1;    /* Clears GIF, TCIF, HTIF, TEIF */

    /* Process the half that the DMA is NOT writing right now */
    if (isr & DMA_ISR_HTIF1) {
        accumulate(0);
    }
    if (isr & DMA_ISR_TCIF1) {
        accumulate(DMA_HALF_LEN);
    }
}

bool adc_read_average(uint16_t out[ADC_CHANNEL_COUNT])
{
    uint32_t acc[ADC_CHANNEL_COUNT];
    uint32_t count;

    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    for (uint32_t ch = 0; ch < ADC_CHANNEL_COUNT; ch++) {
        acc[ch] = s_acc[ch];
        s_acc[ch] = 0;
    }
    count = s_acc_count;
    s_acc_count = 0;
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    if (count == 0U) {
        return false;
    }
    for (uint32_t ch = 0; ch < ADC_CHANNEL_COUNT; ch++) {
        out[ch] = (uint16_t)((acc[ch] + count / 2U) / count);
    }
    return true;
}
