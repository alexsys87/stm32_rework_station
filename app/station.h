/**
 * @file    station.h
 * @brief   Station control logic: sensors, setpoint, PID, air flow,
 *          cool-down and safety supervision.
 */
#ifndef STATION_H
#define STATION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    STATION_STANDBY = 0,    /* Heater off, cooled down, pump off */
    STATION_COOLING,        /* Heater off, pump blowing until cool */
    STATION_WORK,           /* Regulating temperature */
    STATION_FAULT           /* Heater off, pump at full flow */
} station_mode_t;

/* Fault codes, shown on the display as "Err  N" */
typedef enum {
    FAULT_NONE     = 0,
    FAULT_OVERHEAT = 1,     /* Temperature above OVERHEAT_C */
    FAULT_SENSOR   = 2,     /* Thermocouple open or ADC stalled */
    FAULT_RUNAWAY  = 3,     /* Full power but no temperature rise */
    FAULT_NO_MAINS = 4      /* No zero-cross pulses */
} fault_t;

typedef struct {
    station_mode_t mode;
    fault_t fault;
    bool    temp_valid;
    float   temperature_c;
    float   setpoint_c;
    uint8_t heater_percent;
    uint8_t pump_percent;
} station_status_t;

void station_init(uint32_t now_ms);

/* Run one control cycle; call every CONTROL_PERIOD_MS */
void station_step(uint32_t now_ms);

const station_status_t *station_status(void);

#endif /* STATION_H */
