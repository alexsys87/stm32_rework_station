/**
 * @file    ui.c
 * @brief   Display rendering and status LED.
 */
#include "ui.h"
#include "config.h"
#include "board.h"
#include "max7219.h"

/* Segment codes, no-decode mode: DP A B C D E F G */
enum {
    SEG_BLANK = 0x00,
    SEG_C     = 0x4E,
    SEG_O     = 0x7E,
    SEG_L     = 0x0E,
    SEG_E     = 0x4F,
    SEG_R     = 0x05,   /* Lowercase r */
    SEG_MINUS = 0x01,
};

static const uint8_t s_digits[10] = {
    0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B,
};

static float    s_shown_temp;
static bool     s_shown_init;
static uint32_t s_last_reinit_ms;

void ui_init(void)
{
    max7219_init();
}

/* Three right-aligned digits without leading zeros into out[0..2] */
static void put_number(uint8_t *out, float value)
{
    int32_t v = (int32_t)(value + 0.5f);
    if (v < 0)   v = 0;
    if (v > 999) v = 999;

    out[0] = (v >= 100) ? s_digits[v / 100] : SEG_BLANK;
    out[1] = (v >= 10)  ? s_digits[(v / 10) % 10] : SEG_BLANK;
    out[2] = s_digits[v % 10];
}

static void put_text3(uint8_t *out, uint8_t a, uint8_t b, uint8_t c)
{
    out[0] = a;
    out[1] = b;
    out[2] = c;
}

static void render_readout(uint8_t frame[MAX7219_DIGITS], const station_status_t *st)
{
    if (st->temp_valid) {
        put_number(&frame[0], s_shown_temp);
    } else {
        put_text3(&frame[0], SEG_MINUS, SEG_MINUS, SEG_MINUS);
    }
    frame[2] |= SEG_DP;     /* Separator between actual and setpoint */
    put_number(&frame[3], st->setpoint_c);
}

static void update_led(const station_status_t *st, uint32_t now_ms)
{
    switch (st->mode) {
    case STATION_WORK:
        board_led(((now_ms / 500U) & 1U) == 0U);
        break;
    case STATION_FAULT:
        board_led(((now_ms / 125U) & 1U) == 0U);
        break;
    default:
        board_led(0);
        break;
    }
}

void ui_update(const station_status_t *st, uint32_t now_ms)
{
    uint8_t frame[MAX7219_DIGITS] = {0};

    if (!s_shown_init) {
        s_shown_temp = st->temperature_c;
        s_shown_init = true;
    }
    s_shown_temp += DISPLAY_TEMP_FILTER * (st->temperature_c - s_shown_temp);

    bool alt_fast = ((now_ms / 500U) & 1U) == 0U;
    bool alt_slow = ((now_ms / 1000U) & 1U) == 0U;

    switch (st->mode) {
    case STATION_FAULT:
        if (alt_fast) {
            put_text3(&frame[0], SEG_E, SEG_R, SEG_R);
            frame[5] = s_digits[st->fault % 10U];
        } else {
            render_readout(frame, st);
        }
        break;

    case STATION_COOLING:
        if (alt_slow) {
            put_text3(&frame[0], SEG_C, SEG_O, SEG_L);
        } else {
            render_readout(frame, st);
        }
        break;

    case STATION_STANDBY:
        put_text3(&frame[0], SEG_C, SEG_O, SEG_L);
        break;

    case STATION_WORK:
    default:
        render_readout(frame, st);
        break;
    }

    /* Periodically re-send the configuration and the whole frame to
     * recover from EMI-induced glitches of the MAX7219. */
    int force = 0;
    if ((uint32_t)(now_ms - s_last_reinit_ms) >= DISPLAY_REINIT_MS) {
        s_last_reinit_ms = now_ms;
        max7219_refresh_config();
        force = 1;
    }
    max7219_write_frame(frame, force);

    update_led(st, now_ms);
}
