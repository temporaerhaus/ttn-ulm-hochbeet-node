#include "config/control.h"
#include "logic_job.h"
#include "lora_job.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "config/pins.h"
#include <RTCZero.h>
#include <VL6180X.h>



static osjob_t logicjob;


Adafruit_BME280 bme;  // I2C, depending on your BME, you have to use address 0x77 (default) or 0x76, see below
const int BME_ADDR = 0x76; // use address 0x77 (default) or 0x76
VL6180X s_vlx6180; // I2C, Pololu VL6180X Time-of-Flight Distance Sensor adress 0x29
RTCZero rtc;

// tensiometer config
const float VminTyp = 0.2f;
const float VmaxTyp = 4.7f;
const float VrangeTyp = VmaxTyp - VminTyp;
const float maxPressure = 500.0f;

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

instance_data_t hochbeet_data = { &hochbeet_config, 0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, true, true, false };


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
float readTensiometerInternalWaterLevel();
void printdigit(int number);
uint32_t getTime();





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

    bme.takeForcedMeasurement();
    data->temperature = bme.readTemperature();
    data->humidity = bme.readHumidity();
    data->airPressure = bme.readPressure();

    Serial.print("Temp: ");
    Serial.print(data->temperature);
    Serial.print(" C\t Hum: ");
    Serial.print(data->humidity);
    Serial.print(" %\t Pressure: ");
    Serial.print(data->airPressure );
    Serial.println("hpa");

    // Water Tank @todo validate correct assignment
    data->waterTankEmpty = digitalRead(PIN_WATER_TANK_EMPTY) == 1 ? true : false;
    Serial.print("Status Water Tank Empty: ");
    Serial.println(data->waterTankEmpty);
    Serial.println();

    // Flower Pot @todo validate correct assignment
    data->flowerPotFull  = digitalRead(PIN_FLOWER_POT_FULL)  == 1 ? false : true;
    Serial.print("Status Flower Pot Full: ");
    Serial.println(data->flowerPotFull);
    Serial.println();


    // Read tensiometer (soil moisture)
    data->tensiometerPressure = readTensiometerPressure();
    Serial.print("Tensiometer ");
    Serial.print(data->tensiometerPressure );
    Serial.println("");

    // No need for sub-mm accuracy
    data->tensiometerInternalWaterLevel = (uint8_t) round(readTensiometerInternalWaterLevel());

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
    if (data->waterTankEmpty) payload[13] |= 1 << 0;
    //if (data->waterTankEmpty) payload[13] |= 1 << 1;

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
    if(!data->waterTankEmpty
        && getTime() > data->timeLastIrrigationStart + data->config->irrigationIntervalSec
        && data->tensiometerPressure <= data->config->tensiometerMinPressure) {
            
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
        data->timeLastIrrigationStart = getTime();
    }

    // Set start of current irrigation intervall
    data->timeLastPumpStart = getTime();

    pumpeStart();

    return PUMP_RUN;
}

state_t do_state_pump_run(instance_data_t *data) {
    Serial.println("do_state_pump_run");
    // Water Tank @TODO validate correct assignment
    data->waterTankEmpty = digitalRead(PIN_WATER_TANK_EMPTY) == 1 ? true : false;

    // Flower Pot @TODO validate correct assignment
    data->flowerPotFull  = digitalRead(PIN_FLOWER_POT_FULL)  == 1 ? false : true;

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
    cur_state = run_state( cur_state, &hochbeet_data );

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
    if (!bme.begin(BME_ADDR))
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1) ; // @todo – sinnvoll? Pflanzen vertrocken lassen, weil Temperatur-Sensor nicht geht?
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
float readTensiometerPressure()
{
    int rawValue = analogRead(TENSIOMETER_PRESSURE_PIN); // read the input pin
    float tensiometerPressure;
    // @todo auf 3V runterbrechen
    float voltage = (float)rawValue * (5.0 / 1023.0);
    voltage = (voltage < VminTyp) ? VminTyp : voltage;
    tensiometerPressure = 1.0 / VrangeTyp * (voltage - VminTyp) * maxPressure;

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
