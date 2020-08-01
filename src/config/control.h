#ifndef HOCHBEET_CONFIG_H
#define HOCHBEET_CONFIG_H

#include <Arduino.h>

// strcut to store config & parameters
typedef struct {
    uint32_t            irrigationIntervalSec, // in milliseconds
    irrigationDurationSec,
            irrigationPauseSec,
            txIntervalSec,
            defaultSleepTimeSec;
    float               tensiometerMaxPressure;
} hochbeet_config_t;

// struct to store current state of hochbeet
typedef struct {
    hochbeet_config_t   *config;
    uint32_t            timeLastPumpStart,
            timeLastPumpStop,
            timeLastIrrigationStart;
    float               temperature,
            humidity,
            airPressure,
            tensiometerPressure,
            tankDistance,
            watertankPressure;
    uint32_t            timeLastDataSent;
    uint8_t             tensiometerInternalWaterLevel;
    boolean             waterTankEmpty;
    boolean             flowerPotFull;
    boolean             irrigationRunning;
    boolean             bmeAvailable;
    boolean             wasWateredOnLastCycle;
} instance_data_t;

extern instance_data_t hochbeet_data;


// tensiometer config
typedef struct {
    float   VminTyp,
            VmaxTyp,
            VrangeTyp,
            maxPressure;
} tensiometer_config_t;

extern tensiometer_config_t tensiometer_config_water_tank;
extern tensiometer_config_t tensiometer_config_soil_moistor;


#endif