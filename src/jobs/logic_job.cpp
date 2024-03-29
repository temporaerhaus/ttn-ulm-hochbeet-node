#include "config/control.h"
#include "logic_job.h"
#include "lora_job.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "config/pins.h"
#include <RTCZero.h>
#include <VL6180X.h>
#include <Adafruit_ADS1X15.h>
#include "sensors.h"
Adafruit_ADS1115 ads ; // (default gain = 2/3x to read +/- 6.144V  1 bit = 3mV)

static osjob_t logicjob;
Adafruit_BME280 bme;  // I2C, depending on your BME, you have to use address 0x77 (default) or 0x76, see below
const int BME_ADDR = 0x76; // use address 0x77 (default) or 0x76
VL6180X s_vlx6180; // I2C, Pololu VL6180X Time-of-Flight Distance Sensor adress 0x29
RTCZero rtc;

// unpretty workaround to access state ojects within LMIC OS jobs
state_t cur_state; // set initial state to READ_SENSORS
bool standByEntered = false;

typedef state_t state_func_t( instance_data_t *data );

state_t do_state_read_sensors( instance_data_t *data );
state_t do_state_send_data( instance_data_t *data );
state_t do_state_standby( instance_data_t *data );
state_t do_state_pump_start( instance_data_t *data );
state_t do_state_pump_run( instance_data_t *data );
state_t do_state_pump_stop( instance_data_t *data );

void printState(state_t  cur_state, state_t prev_state);

// declaring functions
void sleepForSeconds(int milliSeconds);
void setRelay(int state);
void pumpStart();
void pumpStop();
void writeDisplay();
float readTensiometerInternalWaterLevel();
void printDigit(int number);
uint32_t getTime();
float readTankDistance();
void quicksort(float number[20], int first, int last);

const state_names_t state_names[] {
        {0, "[S] Read sensors"},
        {1, "[S] Send data"},
        {2, "[S] Standby"},
        {3, "[S] Pump start"},
        {4, "[S] Pump run"},
        {5, "[S] Pump stop"},
};

state_func_t* const state_table[ NUM_STATES ] = {
    do_state_read_sensors, do_state_send_data, do_state_standby, do_state_pump_start, do_state_pump_run, do_state_pump_stop
};


// Executes given state and returns next state
state_t run_state( state_t cur_state, instance_data_t *data ) {
    #ifdef DEBUG
        //Serial.println("run_state");
    #endif

    return state_table[ cur_state ]( data );
};


// reads sensors and stores measurements
state_t do_state_read_sensors(instance_data_t *data) {
    //Serial.println("do_state_read_sensors");

    if (data->bmeAvailable) {
        Serial.println("Reading BME sensor values.");
        bme.takeForcedMeasurement();
        data->temperature = bme.readTemperature();
        data->humidity = bme.readHumidity();
        data->airPressure = bme.readPressure();
        Serial.print("Temp: ");
        Serial.print(data->temperature);
        Serial.print(" C\t Hum: ");
        Serial.print(data->humidity);
        Serial.print(" %\t Pressure: ");
        Serial.print(data->airPressure);
        Serial.println("hpa");
    } else {
        data->temperature = -273.16;
        data->humidity = -1;
        data->airPressure = -1;
        Serial.println("No BME sensor available. Not reading data from it.");
    }

    // Water Tank @todo validate correct assignment (the nail sensor)
    //data->waterTankEmpty = read_water_tank_status();
    data->waterTankEmpty = false;
    Serial.print("Status Water Tank Empty: ");
    Serial.println(data->waterTankEmpty);

    // Flower Pot @todo validate correct assignment
    data->flowerPotFull  = read_flower_pot_status();
    Serial.print("Status Flower Pot Full: ");
    Serial.println(data->flowerPotFull);

    // Read tensiometer (soil moisture)
    data->tensiometerPressure = read_tensiometer_pressure(ads);
    Serial.print("Tensiometer ");
    Serial.println(data->tensiometerPressure);

    // Read watertank Pressure
    data->watertankPressure = read_watertank_pressure(ads);
    Serial.print("WassertankDruck ");
    Serial.println(data->watertankPressure);

    // No need for sub-mm accuracy
    data->tensiometerInternalWaterLevel = (uint8_t) round(read_tensiometer_internal_water_level(s_vlx6180));

    // Read distance sensor in water tank
    data->tankDistance = read_tank_distance_sensor();
    Serial.println(data->tankDistance);

    // Read battery
    data->battery = read_battery(ads);
    Serial.print("Battery ");
    Serial.println(data->battery);

    return SEND_DATA;
}


// prepares measurements for transmission and starts send_job
state_t do_state_send_data(instance_data_t *data) {
    //Serial.println("do_state_send_data");
    /*
                        timeLastIrrigationStart;        4
    float               temperature,                    2
                        humidity,                       2
                        airPressure,                    2
                        tensiometerPressure,            2
                        tensiometerInternalWaterLevel;  1
    boolean             waterTankEmpty = true;          1
    boolean             flowerPotFull = false;          1
    */
    byte payload[PAYLOAD_SIZE];

    // time of last irrigation
    payload[0] = (byte) ((data->timeLastIrrigationStart & 0xFF000000) >> 24 );
    payload[1] = (byte) ((data->timeLastIrrigationStart & 0x00FF0000) >> 16 );
    payload[2] = (byte) ((data->timeLastIrrigationStart & 0x0000FF00) >> 8  );
    payload[3] = (byte) ((data->timeLastIrrigationStart & 0X000000FF)       );

    // temperature
    int temp = round((data->temperature +50) * 100);
    payload[4] = highByte(temp);
    payload[5] = lowByte(temp);

    // air pressure
    int pressure = round(data->airPressure/100);
    payload[6] = highByte(pressure);
    payload[7] = lowByte(pressure);

    // humidity
    int humidity = round(data->humidity * 100);
    payload[8] = highByte(humidity);
    payload[9] = lowByte(humidity);

    // tensiometer internal pressure
    int tensioPressure = round(data->tensiometerPressure);
    payload[10] = highByte(tensioPressure);
    payload[11] = lowByte(tensioPressure);
    
    // tensiometer internal distance
    payload[12] = data->tensiometerInternalWaterLevel;

    // bools
    payload[13] = 0;
    if (data->waterTankEmpty)  {
        payload[13] |= 1 << 0;
    }
    if (data->wasWateredOnLastCycle)  {
        payload[13] |= 1 << 1;
        data->wasWateredOnLastCycle = false; // reset value
    }

    // tank distance
    int tankDistance = round(data->tankDistance * 100); // example: 19.26 cm, * 100 = 1926. divide by 100 on TTN console
    payload[14] = highByte(tankDistance);
    payload[15] = lowByte(tankDistance);
    int watertankPressure = round(data->watertankPressure);
    payload[16] = highByte(watertankPressure);
    payload[17] = lowByte(watertankPressure);

    int battery = round(data->battery*100);
    payload[18] = highByte(battery);
    payload[19] = lowByte(battery);

    // Schedule transmission in 3s
    lora_job_send_status(payload);

    data->timeLastDataSent = getTime();



    return STANDBY;
}

state_t do_state_standby( instance_data_t *data ) {
    //Serial.println("do_state_standby");

    #ifdef DEBUG1
    Serial.print("Current Time: ");
    Serial.println(getTime());
    Serial.print("Time Last Data Sent: ");
    Serial.println(data->timeLastDataSent);
    Serial.print("TX Intervall in Sec: ");
    Serial.println(data->config->txIntervalSec);
    Serial.print("Time next Data Sending: ");
    Serial.println(data->timeLastDataSent + data->config->txIntervalSec);
    Serial.print("Verbleibende Sekunden: ");
    Serial.println(data->timeLastDataSent + data->config->txIntervalSec-getTime());
    Serial.println();
    #endif

    if(getTime() > data->timeLastDataSent + data->config->txIntervalSec) {
        return READ_SENSORS;
    }

    // TODO
    // todo ms print values and reasons

    //Serial.print("Current: ");
    //Serial.println(getTime());
    //Serial.print("Stop at: ");
    //Serial.println(data->timeLastIrrigationStart+ data->config->irrigationDurationSec);
    //Serial.print("Diff   : ");
    //Serial.println(getTime() - (data->timeLastIrrigationStart + data->config->irrigationDurationSec));

    if(!data->waterTankEmpty
        && (data->timeLastIrrigationStart == 0 || getTime() > data->timeLastIrrigationStart + data->config->irrigationIntervalSec)
        && data->tensiometerPressure >= data->config->tensiometerMaxPressure) {
        return PUMP_START;
    }

    // sleeping is temporary disabled
    //sleepForSeconds(data->config->defaultSleepTimeSec);
    return STANDBY;
}

// starts pump
state_t do_state_pump_start( instance_data_t *data ) {
    //Serial.println("do_state_pump_start");
    // If starting a new irrigation, set start time (note: a single irrigation
    // consists of multiple irrigation intervalls)
    if(!data->irrigationRunning) {
        data->irrigationRunning = true;
        data->wasWateredOnLastCycle = true;
        data->timeLastIrrigationStart = getTime();
    }

    // Set start of current irrigation intervall
    data->timeLastPumpStart = getTime();

    Serial.print("[I] Pumping for "); Serial.print(data->config->irrigationDurationSec); Serial.println("s");
    pumpStart();

    return PUMP_RUN;
}

state_t do_state_pump_run(instance_data_t *data) {
    //Serial.println("do_state_pump_run");
    // Water Tank @TODO validate correct assignment
    //data->waterTankEmpty = digitalRead(PIN_WATER_TANK_EMPTY) == 1 ? false : true;
    data->waterTankEmpty = false;
    Serial.print("[I] Water tank: ");
    Serial.println(data->waterTankEmpty);

    // Flower Pot @TODO validate correct assignment
    //data->flowerPotFull  = digitalRead(PIN_FLOWER_POT_FULL)  == 1 ? true : false;
    data->flowerPotFull = false;
    Serial.print("[I] Flower pot full: ");
    Serial.println(data->flowerPotFull);

    if( data->waterTankEmpty
        || data->flowerPotFull
        || getTime() > data->timeLastPumpStart + data->config->irrigationDurationSec) {

            if(data->waterTankEmpty) {
                Serial.println("EXIT: Water Tank Empty");
            }
            if(data->flowerPotFull) {
                Serial.println("EXIT: Flower Pot Full");
            }
            if(getTime() > data->timeLastPumpStart + data->config->irrigationIntervalSec) {
                Serial.println("EXIT: irrigationIntervalSec");
            }

            return PUMP_STOP;
    }

    return PUMP_RUN;
}

state_t do_state_pump_stop ( instance_data_t *data ) {
    //Serial.println("do_state_pump_stop");
    pumpStop();
    data->timeLastPumpStop = getTime();

    
    // Irrigation complete or water tank empty (has to be checked first!)
    if(data->waterTankEmpty
        || getTime() > data->timeLastIrrigationStart + data->config->irrigationDurationSec) {
            data->irrigationRunning = false;
            return STANDBY;
    }

    // Continue irrigation after pause
    if(getTime() > data->timeLastPumpStop + data->config->irrigationPauseSec
        && getTime() < data->timeLastIrrigationStart + data->config->irrigationDurationSec) {
            return PUMP_START;
    }

    sleepForSeconds(data->config->defaultSleepTimeSec);
    return PUMP_STOP;
}


void do_logic(osjob_t *j) {
    state_t prev_state = cur_state;
    cur_state = run_state(cur_state, &hochbeet_data);

    printState(cur_state, prev_state);

    // Start next logic job here
    os_setTimedCallback(&logicjob, os_getTime()+sec2osticks(hochbeet_data.config->defaultSleepTimeSec), do_logic);
}


/*
Starts first run of logic job after lmic has joined ttn
*/
void onJoinedCallback(void *pUserData, ev_t ev) {
    switch (ev) {
    case EV_JOINED:
    case EV_JOIN_FAILED: // start logic even if join failed
        os_setTimedCallback(&logicjob, os_getTime()+sec2osticks(1), do_logic);
        break;
    default:
        // prevents compiler warnings
        break;
    }
}

void logic_job_init() {
    //***********************
    // RTC
    //***********************
    rtc.begin();
    rtc.setTime(14, 4, 30);
    rtc.setDate(6, 10, (uint8_t)70);
    rtc.setYear(19);

    //***********************
    // BME280
    //***********************
    int maxTries = 3; int tries = 0;
    if (!bme.begin(BME_ADDR)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring! Trying a few times...");
        Serial.printf("Try ");
        while (tries <= maxTries) {
            Serial.printf("%d...", tries);
            delay(200);
            tries++;
            if (tries >= maxTries) {
                Serial.println("\n\rThis is not working. Giving up. NOT using BME sensor.");
                tries = 0;
                hochbeet_data.bmeAvailable = false;
                break;
            };
        }
    }

    // Set BME in force mode to reduce power consumption
    // force mode = measure, store results, and go into sleep mode
    // until next measurement, see
    // - http://tinkerman.cat/low-power-weather-station-bme280-moteino/
    // - https://github.com/adafruit/Adafruit_BME280_Library/blob/master/examples/advancedsettings/advancedsettings.ino
    // values taken from example of 'Weather / Climate Monitor"
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF);

    //**********************
    // ADS1115
    //**********************
    ads.begin();
    ads.startComparator_SingleEnded(0, 1000);
    
    //***********************
    // RELAYS
    //***********************
    pinMode(PIN_RELAY, OUTPUT);

    //***********************
    // VL6180X
    //***********************
    Wire.begin();
    s_vlx6180.init();
    s_vlx6180.configureDefault();
    s_vlx6180.setTimeout(500);
    // stop continuous mode if already active
    s_vlx6180.stopContinuous();
    delay(500);

    //***********************
    //Interrupts for Sensors
    //***********************
    pinMode(PIN_WATER_TANK_EMPTY, INPUT);
    pinMode(PIN_FLOWER_POT_FULL, INPUT);
    // ultrasonic distance pins
    pinMode(PIN_TANK_DISTANCE_TRIGGER, OUTPUT);
    pinMode(PIN_TANK_DISTANCE_ECHO, INPUT);

    cur_state = READ_SENSORS; // set initial state to READ_SENSORS

    #ifdef OTAA
    LMIC_registerEventCb(onJoinedCallback, NULL);
    #else
    // schedule first run only in APB mode, OTAA mode needs to wait for join, dirty hack, we should drop ABP support
    os_setTimedCallback(&logicjob, os_getTime()+sec2osticks(5), do_logic);
    #endif
}

//***************************
// set Relay state = 1 Relay on ; state=0 Relay off
//***************************
void setRelay(int state)
{
    digitalWrite(PIN_RELAY, state);
}

/**
 * Returns time since system start. Dependend on the board's 
 * capabilities using rtc or millis.
 */
uint32_t getTime() {
    return rtc.getEpoch();
}

/**
 * Starts the pump to pump water into the plant.
 */
void pumpStart()
{
    setRelay(1);
}

/**
 * Stops the pump.
 */
void pumpStop()
{
    setRelay(0);
}

/**
 * Powers down node for <milliseconds> ms.
 */
void sleepForSeconds(int seconds)
{
    // temporary disabled
}

void printState(state_t  cur_state, state_t prev_state) {

    // first time entering standby
    if (!standByEntered && cur_state == STANDBY) {
        Serial.print(state_names[cur_state].name);
        standByEntered = true;
    }
    if (cur_state == STANDBY && standByEntered) {
        Serial.print(".");
    }
    if (cur_state != STANDBY) {
        if (prev_state == STANDBY) {Serial.println("");}
        Serial.println(state_names[cur_state].name);
        standByEntered = false;
    }

}