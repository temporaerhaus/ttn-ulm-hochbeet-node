#ifndef LOGIC_H
#define LOGIC_H

#include <Arduino.h>

// Enum holds name of states
typedef enum {
    READ_SENSORS,
    SEND_DATA,
    STANDBY,
    PUMP_START,
    PUMP_RUN,
    PUMP_STOP,
    NUM_STATES } state_t;

typedef struct {
    uint8_t lang;
    char const *name;
} state_names_t;

void logic_job_init(void);

#endif

