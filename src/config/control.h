#ifndef HOCHBEET_CONFIG_H
#define HOCHBEET_CONFIG_H

#include "../jobs/logic_job.h"



// config of hochbeet
hochbeet_config_t hochbeet_config = {
    .irrigationIntervalSec = (uint32_t) 2 * 60, // 8 h
    .irrigationDurationSec = (uint32_t) 1 * 60, // 5 min
    .irrigationPauseSec = (uint32_t) 30, // 30 s
    .txIntervalSec = (uint32_t) 1 * 30, // 5 * 60  // 5 min
    .defaultSleepTimeSec = 1, // 1 s
    .tensiometerMinPressure = 1000.0f, // 70 mBar
};

#endif