#include "config/control.h"
#include "logic_job.h"
#include "lora_job.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "config/pins.h"
#include <RTCZero.h>
#include <VL6180X.h>
#include <Adafruit_ADS1015.h>
#include "sensors.h"
Adafruit_ADS1115 ads ; // (default gain = 2/3x to read +/- 6.144V  1 bit = 3mV)


static osjob_t logicjob;


Adafruit_BME280 bme;  // I2C, depending on your BME, you have to use address 0x77 (default) or 0x76, see below
const int BME_ADDR = 0x76; // use address 0x77 (default) or 0x76
VL6180X s_vlx6180; // I2C, Pololu VL6180X Time-of-Flight Distance Sensor adress 0x29
RTCZero rtc;

// Enum holds name of states
typedef enum { 
    READ_SENSORS,
    SEND_DATA,
    STANDBY,
    PUMP_START,
    PUMP_RUN,
    PUMP_STOP,
    NUM_STATES } state_t;

// unpretty workaround to access state ojects within LMIC OS jobs
state_t cur_state; // set initial state to READ_SENSORS

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


typedef state_t state_func_t( instance_data_t *data );

state_t do_state_read_sensors( instance_data_t *data );
state_t do_state_send_data( instance_data_t *data );
state_t do_state_standby( instance_data_t *data );
state_t do_state_pump_start( instance_data_t *data );
state_t do_state_pump_run( instance_data_t *data );
state_t do_state_pump_stop( instance_data_t *data );

// declaring functions
void sleepForSeconds(int milliSeconds);
void setRelay(int state);
void pumpeStart();
void pumpeStop();
void writeDisplay();
float readTensiometerPressure();
float readWatertankPressure();
float readTensiometerInternalWaterLevel();
void printdigit(int number);
uint32_t getTime();
float readTankDistance();
void quicksort(float number[20], int first, int last);





state_func_t* const state_table[ NUM_STATES ] = {
    do_state_read_sensors, do_state_send_data, do_state_standby, do_state_pump_start, do_state_pump_run, do_state_pump_stop
};

// Executes given state and returns next state
state_t run_state( state_t cur_state, instance_data_t *data ) {
    #ifdef DEBUG
        Serial.println("run_state");
    #endif

    return state_table[ cur_state ]( data );
};





// reads sensors and stores measurements
state_t do_state_read_sensors(instance_data_t *data) {
    Serial.println("do_state_read_sensors");

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

    // Water Tank @todo validate correct assignment
    //data->waterTankEmpty = digitalRead(PIN_WATER_TANK_EMPTY) == 1 ? false : true;
    data->waterTankEmpty = false;
    Serial.print("Status Water Tank Empty: ");
    Serial.println(data->waterTankEmpty);
    Serial.println();

    // Flower Pot @todo validate correct assignment
    data->flowerPotFull  = digitalRead(PIN_FLOWER_POT_FULL)  == 1 ? true : false;
    Serial.print("Status Flower Pot Full: ");
    Serial.println(data->flowerPotFull);
    Serial.println();


    // Read tensiometer (soil moisture)
    data->tensiometerPressure = readTensiometerPressure();
    Serial.print("Tensiometer ");
    Serial.print(data->tensiometerPressure);
    Serial.println("");

// Read watertank Pressure
    data->watertankPressure = readWatertankPressure();
    Serial.print("WassertankDruck ");
    Serial.print(data->watertankPressure);
    Serial.println("");

    // No need for sub-mm accuracy
    data->tensiometerInternalWaterLevel = (uint8_t) round(readTensiometerInternalWaterLevel());

    // Read distance sensor in water tank
    Serial.print("Reading tank distance...");
    data->tankDistance = read_tank_distance_sensor();
    Serial.print("Tank distance ");
    Serial.print(data->tankDistance);
    Serial.println("");


    return SEND_DATA;
}


// prepares measurements for transmission and starts send_job
state_t do_state_send_data(instance_data_t *data) {
    Serial.println("do_state_send_data");
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
    int tankDistance = round(data->tankDistance * 100); // is / 100 correct?
    payload[14] = highByte(tankDistance);
    payload[15] = lowByte(tankDistance);
    int watertankPressure = round(data->watertankPressure);
    payload[16] = highByte(watertankPressure);
    payload[17] = lowByte(watertankPressure);

    // Schedule transmission in 3s
    lora_job_send_status(payload);

    data->timeLastDataSent = getTime();



    return STANDBY;
}

state_t do_state_standby( instance_data_t *data ) {
    Serial.println("do_state_standby");

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
        && data->tensiometerPressure >= data->config->tensiometerMinPressure) {
        return PUMP_START;
    }

    // sleeping is temporary disabled
    //sleepForSeconds(data->config->defaultSleepTimeSec);
    return STANDBY;
}

// starts pump
state_t do_state_pump_start( instance_data_t *data ) {
    Serial.println("do_state_pump_start");
    // If starting a new irrigation, set start time (note: a single irrigation
    // consists of multiple irrigation intervalls)
    if(!data->irrigationRunning) {
        data->irrigationRunning = true;
        data->wasWateredOnLastCycle = true;
        data->timeLastIrrigationStart = getTime();
    }

    // Set start of current irrigation intervall
    data->timeLastPumpStart = getTime();

    Serial.print("Pumping for "); Serial.print(data->config->irrigationDurationSec); Serial.println("s");
    pumpeStart();

    return PUMP_RUN;
}

state_t do_state_pump_run(instance_data_t *data) {
    Serial.println("do_state_pump_run");
    // Water Tank @TODO validate correct assignment
    //data->waterTankEmpty = digitalRead(PIN_WATER_TANK_EMPTY) == 1 ? false : true;
    data->waterTankEmpty = false;

    // Flower Pot @TODO validate correct assignment
    //data->flowerPotFull  = digitalRead(PIN_FLOWER_POT_FULL)  == 1 ? true : false;
    data->flowerPotFull = false;

    // TODO hier geht er wohl aus

    //Serial.print("Current: ");
    //Serial.println(getTime());
    //Serial.print("Stop at: ");
    //int stop = data->timeLastPumpStart == 0 ? (getTime() + data->config->irrigationDurationSec) : data->timeLastPumpStart + data->config->irrigationDurationSec;
    //Serial.println(stop);
    //Serial.print("Diff   : ");
    //Serial.println(getTime() - stop);

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
    Serial.println("do_state_pump_stop");
    pumpeStop();
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
    cur_state = run_state(cur_state, &hochbeet_data);

    // Start next logic job here
    os_setTimedCallback(&logicjob, os_getTime()+sec2osticks(hochbeet_data.config->defaultSleepTimeSec), do_logic);
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

    cur_state = READ_SENSORS; // set initial state to READ_SENSORS

    // schedule first run
    os_setTimedCallback(&logicjob, os_getTime()+sec2osticks(hochbeet_data.config->defaultSleepTimeSec), do_logic);
}


/**
 * Reads the internal water level of the
 * tensiometer
 */
float readTensiometerInternalWaterLevel()
{   
    s_vlx6180.startRangeContinuous();
    uint8_t numberOfSuccesfullMeasurements = 0;
    float distance = 0.0f;

    for (uint8_t i = 0; i < 20; i++)
    {
        float currentDistance = s_vlx6180.readRangeContinuousMillimeters();
        if (!s_vlx6180.timeoutOccurred())
        {
            distance += currentDistance;
            numberOfSuccesfullMeasurements += 1;
        }
    }
    
    if (numberOfSuccesfullMeasurements > 0)
    {
        distance = distance/numberOfSuccesfullMeasurements;
    }
    
    return distance;
}

//***************************
// set Relay state = 1 Relay on ; state=0 Relay off
//***************************
void setRelay(int state)
{
    digitalWrite(PIN_RELAY, state);
}

/**
 * Reads to inner pressure of the tensiometer which
 * indicates how moist the soil is. 
 */
float readTensiometerPressure() {
    // We use external  16 Bit ADC ADS1115. It has an resolution of 188uV/bit
    int rawValue = ads.readADC_SingleEnded(0);
    float tensiometerPressure;
    float voltage = rawValue * 188.0f / 1000000.0f;

    voltage = (voltage < tensiometer_config_soil_moistor.VminTyp) ? tensiometer_config_soil_moistor.VminTyp : voltage;
    tensiometerPressure = 1.0 / tensiometer_config_soil_moistor.VrangeTyp * (voltage - tensiometer_config_soil_moistor.VminTyp) * tensiometer_config_soil_moistor.maxPressure;

#ifdef DEBUG
    Serial.print(rawValue);
    Serial.print(" / ");
    Serial.print(voltage);
    Serial.print(" V");
    Serial.print(" / ");
    Serial.print(tensiometerPressure);
    Serial.print(" kPa  ");
    Serial.println(" ");
#endif
    return tensiometerPressure;
}


float readWatertankPressure()
{
    // We use external  16 Bit ADC ADS1115. It has an resolution of 188uV/bit
    int rawValue = ads.readADC_SingleEnded(1);
    float watertankPressure;
    float voltage = rawValue * 188.0f / 1000000.0f;

    voltage = (voltage < tensiometer_config_water_tank.VminTyp) ? tensiometer_config_water_tank.VminTyp : voltage;
    watertankPressure = 1.0 / tensiometer_config_water_tank.VrangeTyp * (voltage - tensiometer_config_water_tank.VminTyp) * tensiometer_config_water_tank.maxPressure;


#ifdef DEBUG
    Serial.print(rawValue);
    Serial.print(" / ");
    Serial.print(voltage);
    Serial.print(" V");
    Serial.print(" / ");
    Serial.print(watertankPressure);
    Serial.print(" kPa  ");
    Serial.println(" ");
#endif
    return watertankPressure;
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
void pumpeStart()
{
    setRelay(1);
}

/**
 * Stops the pump.
 */
void pumpeStop()
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