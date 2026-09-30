/**
 * @file    phase_control.c
 * @brief   Zero-cross synchronised triac phase control (EXTI + TIM3).
 */
#include "phase_control.h"
#include "board.h"
#include "clock.h"

#define PHASE_TIM               TIM3
#define PHASE_TIM_IRQn          TIM3_IRQn

/* Accepted half period range: 42..71 Hz mains. Edges that come earlier
 * than ZC_MIN_HALF_PERIOD_US after the previous one are treated as noise. */
#define ZC_MIN_HALF_PERIOD_US   7000U
#define ZC_MAX_HALF_PERIOD_US   12000U
#define ZC_TIMEOUT_MS           100U

/* Delay between the real mains zero and the detected edge. The pull-up
 * optocoupler output rises around the zero and falls (our edge) shortly
 * after it. Measure with a scope and adjust; 0 is a safe default. */
#define ZC_EDGE_OFFSET_US       0U

/* Earliest firing time (the triac needs some voltage to latch) and the
 * guard before the next zero: a gate that is still on at the zero crossing
 * would turn the triac on for the whole next half-cycle. */
#define PHASE_MIN_FIRE_US       100U
#define PHASE_END_GUARD_US      300U

#define CC_FLAG(ch)             (TIM_SR_CC1IF << (ch))  /* CC1IF/CC1IE share bit positions */
#define CC_ALL_FLAGS            (CC_FLAG(PHASE_CH_HEATER) | CC_FLAG(PHASE_CH_PUMP))
#define CCR(ch)                 ((&PHASE_TIM->CCR1)[ch])

typedef struct {
    GPIO_TypeDef *port;
    uint32_t pin;
    uint16_t gate_us;
} phase_out_t;

static const phase_out_t s_out[PHASE_CH_COUNT] = {
    [PHASE_CH_HEATER] = { HEATER_TRIAC_PORT, HEATER_TRIAC_PIN, HEATER_GATE_PULSE_US },
    [PHASE_CH_PUMP]   = { PUMP_TRIAC_PORT,   PUMP_TRIAC_PIN,   PUMP_GATE_PULSE_US   },
};

/* Firing delay as a fraction of the half period (x65536) for 0..100 %
 * power into a resistive load: solves 1 - a/pi + sin(2a)/(2pi) = P. */
static const uint16_t s_fire_lut[101] = {
    65535, 57934, 55907, 54462, 53297, 52300, 51419, 50621, 49889, 49208,  /*  0.. */
    48568, 47964, 47390, 46841, 46314, 45807, 45317, 44842, 44381, 43933,  /* 10.. */
    43495, 43068, 42650, 42240, 41838, 41443, 41055, 40672, 40295, 39923,  /* 20.. */
    39556, 39193, 38834, 38479, 38127, 37778, 37432, 37089, 36748, 36409,  /* 30.. */
    36072, 35737, 35403, 35071, 34740, 34410, 34080, 33752, 33424, 33096,  /* 40.. */
    32768, 32440, 32112, 31784, 31456, 31126, 30796, 30465, 30133, 29799,  /* 50.. */
    29464, 29127, 28788, 28447, 28104, 27758, 27409, 27057, 26702, 26343,  /* 60.. */
    25980, 25613, 25241, 24864, 24481, 24093, 23698, 23296, 22886, 22468,  /* 70.. */
    22041, 21603, 21155, 20694, 20219, 19729, 19222, 18695, 18146, 17572,  /* 80.. */
    16968, 16328, 15647, 14915, 14117, 13236, 12239, 11074,  9629,  7602,  /* 90.. */
        0,                                                                  /* 100 */
};

static volatile uint8_t  s_power[PHASE_CH_COUNT];
static volatile uint8_t  s_gate_on[PHASE_CH_COUNT];
static volatile uint16_t s_half_period = 10000U;
static volatile uint32_t s_last_zc_ms;
static volatile bool     s_zc_seen;

void phase_init(void)
{
    board_triacs_off();

    /* 1 MHz free-running counter, reset on every zero cross */
    PHASE_TIM->CR1  = 0;
    PHASE_TIM->PSC  = (uint16_t)(clock_apb1_timer_hz() / 1000000U - 1U);
    PHASE_TIM->ARR  = 0xFFFFU;
    PHASE_TIM->DIER = 0;
    PHASE_TIM->EGR  = TIM_EGR_UG;   /* Load the prescaler */
    PHASE_TIM->SR   = 0;
    PHASE_TIM->CR1  = TIM_CR1_CEN;

    /* Zero-cross input: falling edge */
    uint32_t idx = ZC_PIN / 4U;
    uint32_t shift = (ZC_PIN % 4U) * 4U;
    AFIO->EXTICR[idx] = (AFIO->EXTICR[idx] & ~(0xFUL << shift)) | (ZC_EXTI_PORT_SEL << shift);
    EXTI->RTSR &= ~PIN_MASK(ZC_PIN);
    EXTI->FTSR |= PIN_MASK(ZC_PIN);
    EXTI->PR    = PIN_MASK(ZC_PIN);
    EXTI->IMR  |= PIN_MASK(ZC_PIN);

    /* Same priority for both: neither can preempt the other, so the
     * gate state can never be modified half-way by the other handler. */
    NVIC_SetPriority(ZC_EXTI_IRQn, 0);
    NVIC_SetPriority(PHASE_TIM_IRQn, 0);
    NVIC_EnableIRQ(PHASE_TIM_IRQn);
    NVIC_EnableIRQ(ZC_EXTI_IRQn);
}

void phase_set_power(phase_ch_t ch, uint8_t percent)
{
    s_power[ch] = (percent > 100U) ? 100U : percent;
}

bool phase_mains_present(void)
{
    return s_zc_seen && (uint32_t)(clock_millis() - s_last_zc_ms) < ZC_TIMEOUT_MS;
}

uint16_t phase_half_period_us(void)
{
    return s_half_period;
}

void ZC_IRQHandler(void)
{
    if (!(EXTI->PR & PIN_MASK(ZC_PIN))) {
        return;
    }
    EXTI->PR = PIN_MASK(ZC_PIN);

    uint32_t elapsed  = PHASE_TIM->CNT;
    bool     overflow = (PHASE_TIM->SR & TIM_SR_UIF) != 0U;

    if (!overflow && elapsed < ZC_MIN_HALF_PERIOD_US) {
        return;     /* Noise spike inside the half-cycle */
    }

    /* New half-cycle: restart timing and make sure no gate stays on */
    PHASE_TIM->CNT  = 0;
    PHASE_TIM->DIER = 0;
    PHASE_TIM->SR   = 0;
    board_triacs_off();
    s_gate_on[PHASE_CH_HEATER] = 0;
    s_gate_on[PHASE_CH_PUMP]   = 0;

    /* Fire only after a plausible period: this rejects the first edge
     * after power-up / mains loss and single noise pulses. */
    if (overflow || elapsed > ZC_MAX_HALF_PERIOD_US) {
        return;
    }
    s_half_period = (uint16_t)elapsed;
    s_last_zc_ms  = clock_millis();
    s_zc_seen     = true;

    uint32_t dier = 0;
    for (uint32_t ch = 0; ch < PHASE_CH_COUNT; ch++) {
        uint32_t power = s_power[ch];
        if (power == 0U) {
            continue;
        }

        uint32_t fire = (elapsed * s_fire_lut[power]) >> 16;
        fire = (fire > PHASE_MIN_FIRE_US + ZC_EDGE_OFFSET_US) ? fire - ZC_EDGE_OFFSET_US
                                                              : PHASE_MIN_FIRE_US;
        uint32_t last_fire = elapsed - ZC_EDGE_OFFSET_US - PHASE_END_GUARD_US - s_out[ch].gate_us;
        if (fire > last_fire) {
            continue;   /* Too close to the next zero, skip this half-cycle */
        }
        CCR(ch) = fire;
        dier |= CC_FLAG(ch);
    }
    PHASE_TIM->DIER = dier;
}

void TIM3_IRQHandler(void)
{
    uint32_t pending = PHASE_TIM->SR & PHASE_TIM->DIER & CC_ALL_FLAGS;
    PHASE_TIM->SR = ~pending;       /* rc_w0: clears only the handled flags */

    for (uint32_t ch = 0; ch < PHASE_CH_COUNT; ch++) {
        if (!(pending & CC_FLAG(ch))) {
            continue;
        }
        const phase_out_t *out = &s_out[ch];
        if (!s_gate_on[ch]) {
            gpio_set(out->port, out->pin);
            s_gate_on[ch] = 1;
            CCR(ch) += out->gate_us;
        } else {
            gpio_reset(out->port, out->pin);
            s_gate_on[ch] = 0;
            PHASE_TIM->DIER &= ~CC_FLAG(ch);
        }
    }
}
