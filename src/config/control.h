#ifndef HOCHBEET_CONFIG_H
#define HOCHBEET_CONFIG_H

#include "../jobs/logic_job.h"



// config of hochbeet
hochbeet_config_t hochbeet_config = {
    .irrigationIntervalSec = (uint32_t) 6 * 60 * 60, // 8 h
    .irrigationDurationSec = (uint32_t) 1 * 45, // 5 min
    .irrigationPauseSec = (uint32_t) 30, // 30 s
    .txIntervalSec = (uint32_t) 10 * 60, // 5 min
    .defaultSleepTimeSec = 1, // 1 s
    .tensiometerMinPressure = 70.0f, // 70 mBar
};

// config soil moistore tensiometer
// https://www.fujikura.co.jp/eng/products/electronics/sensors/04/2052856_13675.html
// Fujikura AP20R-500KV
tensiometer_config_t tensiometer_config_soil_moistor = {
    .VminTyp = 0.2f,
    .VmaxTyp = 4.7f,
    .VrangeTyp = 4.5f,
    .maxPressure = 500.0f, // mBar
};

// config water tank tensiometer
// https://www.metrodynemems.com/proimages/product/PDF/MIS-2500series.pdf
// MIS-2500-015G
tensiometer_config_t tensiometer_config_water_tank = {
    .VminTyp = 0.25f,
    .VmaxTyp = 4.75f,
    .VrangeTyp = 4.5f,
    .maxPressure = 1034.21f, // mBar
};


#endif