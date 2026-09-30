/**
 * @file    station.c
 * @brief   Station control logic.
 */
#include "station.h"
#include "config.h"
#include "pid.h"
#include "temperature.h"
#include "adc.h"
#include "board.h"
#include "phase_control.h"

#define MAINS_GRACE_MS          500U    /* Time to see the first zero crosses */

static station_status_t s_st;
static pid_ctrl_t s_pid;

static uint32_t s_init_ms;
static bool     s_setpoint_init;
static bool     s_work_request;         /* Debounced work switch */
static uint8_t  s_switch_count;
static bool     s_cooling;              /* Standby pump state (hysteresis) */
static fault_t  s_latched_fault;

static bool     s_runaway_armed;
static uint32_t s_runaway_start_ms;
static float    s_runaway_start_c;

void station_init(uint32_t now_ms)
{
    s_init_ms = now_ms;
    pid_init(&s_pid, PID_KP, PID_KI, PID_KD, CONTROL_PERIOD_MS / 1000.0f,
             PID_D_FILTER, 0.0f, 100.0f);
    s_st.mode = STATION_STANDBY;
    s_st.fault = FAULT_NONE;
    s_st.setpoint_c = SETPOINT_MIN_C;
    s_cooling = true;       /* Blow until the first measurement says cold */
}

const station_status_t *station_status(void)
{
    return &s_st;
}

/* Quantised setpoint with hysteresis: the value only moves when the knob
 * is turned by more than 3/4 of a step, so the display does not flicker. */
static void update_setpoint(uint16_t raw)
{
    float x = SETPOINT_MIN_C + (float)raw * ((SETPOINT_MAX_C - SETPOINT_MIN_C) / ADC_FULL_SCALE);
    float diff = x - s_st.setpoint_c;

    if (!s_setpoint_init || diff > 0.75f * SETPOINT_STEP_C || diff < -0.75f * SETPOINT_STEP_C) {
        float q = (float)(int32_t)(x / SETPOINT_STEP_C + 0.5f) * SETPOINT_STEP_C;
        if (q < SETPOINT_MIN_C) q = SETPOINT_MIN_C;
        if (q > SETPOINT_MAX_C) q = SETPOINT_MAX_C;
        s_st.setpoint_c = q;
        s_setpoint_init = true;
    }
}

static uint8_t pump_from_pot(uint16_t raw)
{
    uint32_t span = 100U - PUMP_MIN_PERCENT;
    return (uint8_t)(PUMP_MIN_PERCENT + (raw * span + 2047U) / 4095U);
}

static void debounce_switch(void)
{
    bool raw = board_ctrl_input_active() != 0;

    if (raw == s_work_request) {
        s_switch_count = 0;
    } else if (++s_switch_count >= SWITCH_DEBOUNCE_SAMPLES) {
        s_work_request = raw;
        s_switch_count = 0;
    }
}

static bool runaway_detected(uint32_t now_ms)
{
    bool heating_hard = s_st.mode == STATION_WORK &&
                        s_st.heater_percent >= RUNAWAY_POWER_PERCENT &&
                        (s_st.setpoint_c - s_st.temperature_c) >= RUNAWAY_MIN_ERROR_C;
    if (!heating_hard) {
        s_runaway_armed = false;
        return false;
    }
    if (!s_runaway_armed || (s_st.temperature_c - s_runaway_start_c) >= RUNAWAY_MIN_RISE_C) {
        /* Start a new observation window */
        s_runaway_armed = true;
        s_runaway_start_ms = now_ms;
        s_runaway_start_c = s_st.temperature_c;
        return false;
    }
    return (uint32_t)(now_ms - s_runaway_start_ms) >= RUNAWAY_TIME_MS;
}

static void supervise(uint32_t now_ms, bool adc_ok)
{
    if (s_latched_fault == FAULT_NONE) {
        if (!adc_ok || !s_st.temp_valid) {
            s_latched_fault = FAULT_SENSOR;
        } else if (s_st.temperature_c > OVERHEAT_C) {
            s_latched_fault = FAULT_OVERHEAT;
        } else if (runaway_detected(now_ms)) {
            s_latched_fault = FAULT_RUNAWAY;
        }
    } else if (!s_work_request) {
        /* A latched fault is cleared only in standby, once its cause is gone */
        bool cause_gone = adc_ok && s_st.temp_valid &&
                          (s_latched_fault != FAULT_OVERHEAT ||
                           s_st.temperature_c < OVERHEAT_CLEAR_C);
        if (cause_gone) {
            s_latched_fault = FAULT_NONE;
        }
    }

    s_st.fault = s_latched_fault;
    if (s_st.fault == FAULT_NONE && !phase_mains_present() &&
        (uint32_t)(now_ms - s_init_ms) >= MAINS_GRACE_MS) {
        s_st.fault = FAULT_NO_MAINS;    /* Not latched: clears when mains returns */
    }
}

void station_step(uint32_t now_ms)
{
    uint16_t raw[ADC_CHANNEL_COUNT];
    float cj_c;

    bool adc_ok = adc_read_average(raw);
    if (adc_ok) {
        (void)temperature_ntc_c(raw[ADC_IDX_NTC], &cj_c);  /* Falls back to 25 C */
        s_st.temp_valid = raw[ADC_IDX_THERMOCOUPLE] < TC_OPEN_ADC_THRESHOLD;
        s_st.temperature_c = temperature_thermocouple_c(raw[ADC_IDX_THERMOCOUPLE], cj_c);
        update_setpoint(raw[ADC_IDX_TEMP_POT]);
    }

    debounce_switch();
    supervise(now_ms, adc_ok);

    uint8_t heater = 0;
    uint8_t pump = 0;

    if (s_st.fault != FAULT_NONE) {
        s_st.mode = STATION_FAULT;
        pump = PUMP_FAULT_PERCENT;
    } else if (s_work_request) {
        s_st.mode = STATION_WORK;
        heater = (uint8_t)(pid_update(&s_pid, s_st.setpoint_c, s_st.temperature_c) + 0.5f);
        pump = pump_from_pot(raw[ADC_IDX_PUMP_POT]);
        s_cooling = true;   /* Cool down after work regardless of temperature */
    } else {
        if (s_cooling && s_st.temperature_c < COOLING_TARGET_C) {
            s_cooling = false;
        } else if (!s_cooling && s_st.temperature_c > COOLING_TARGET_C + COOLING_HYSTERESIS_C) {
            s_cooling = true;
        }
        s_st.mode = s_cooling ? STATION_COOLING : STATION_STANDBY;
        pump = s_cooling ? PUMP_COOLING_PERCENT : 0U;
    }

    if (s_st.mode != STATION_WORK) {
        pid_reset(&s_pid);
    }

    s_st.heater_percent = heater;
    s_st.pump_percent = pump;
    phase_set_power(PHASE_CH_PUMP, pump);
    phase_set_power(PHASE_CH_HEATER, heater);
}
