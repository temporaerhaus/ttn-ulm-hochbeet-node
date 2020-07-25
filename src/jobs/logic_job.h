#ifndef LOGIC_H
#define LOGIC_H

#include <Arduino.h>



// strcut to store config & parameters
typedef struct {
    uint32_t            irrigationIntervalSec, // in milliseconds
                        irrigationDurationSec,
                        irrigationPauseSec,
                        txIntervalSec,
                        defaultSleepTimeSec;
    float               tensiometerMinPressure;
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
} instance_data_t ;


// tensiometer config
typedef struct {
    float   VminTyp,
            VmaxTyp,
            VrangeTyp,
            maxPressure;
} tensiometer_config_t ;


void logic_job_init(void);

#endif

