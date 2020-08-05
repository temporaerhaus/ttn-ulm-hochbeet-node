#include "control.h"


// config of hochbeet
hochbeet_config_t hochbeet_config = {
        .irrigationIntervalSec = (uint32_t) 8 * 60 * 60,
        .irrigationDurationSec = (uint32_t) 45,
        .irrigationPauseSec = (uint32_t) 30, // 30 s
        .txIntervalSec = (uint32_t) 10 * 60, // 5 min
        .defaultSleepTimeSec = 1, // 1 s
        .tensiometerMaxPressure = 70.0f, // 70 mBar
        // @TODO maxPressure changed temporary until correct value is evaluated. This low value ensures regular irrigation
};

// default values for the data object
instance_data_t hochbeet_data = {
        &hochbeet_config,
        0,     // time; last time the pump was started
        0,     // time; last time the pump was stopped
        0,     // time; last time the irrigation was started
        0.0f,  // temperature (bme280)
        0.0f,  // humidity (bme280)
        0.0f,  // air pressure (bme280)
        0.0f,  // tensio meter pressure
        0.0f,  // tank distance
        0.0f,  // watertank pressure
        0,     // time; last time data was sent via lorawan
        0,     // internal water level of the tensiometer
        false,  // is water tank empty
        false,  // is the bed full of water
        false, // is the irrigation running
        true,  // bme available
        false  // was the bed watered on the last cycle
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

