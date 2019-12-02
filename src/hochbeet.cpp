/**
 * Code für den Hochbeet-Arduino für's Verschwörhaus.
 *
 *
 */
/**
*   I2C
*   BME 280 Adresse :       0x76 
*   TOFL Adresse    :       0x29    (Pololu VL6180X)
*/
/*
*   Testboard Pins  Tensiometer Eingang A1
*   Schwimmer 1 D11
*   Schwimmer 2 D12
*   Taster D13 //Wiederstand??
*   Relay OUT   A5 D?????
*/

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <VL6180X.h>

#ifdef LOW_POWER
#include "LowPower.h"
#endif

#ifdef RTC
#include <RTCZero.h>
RTCZero rtc;
#endif

#ifdef OLED
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // Pixelanzahl in der Breite
#define SCREEN_HEIGHT 64 // Pixelanzahl in der Hoehoe
#define OLED_RESET 4     // wird wohl bei diesem Display nicht
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

// declaring functions
void onEvent(ev_t ev);
void do_send(osjob_t *j);
void sleepForSeconds(int seconds);
void setRelay(int state);
void pumpeStart();
void pumpeStop();
void writeDisplay();
float readTensiometerPressure();
float readTensiometerInternalWaterLevel();
void printdigit(int number);
void deepSleep(uint32_t sleepTimeInMilliSeconds);

//***************************
// TTN und LMIC
//***************************
#include "config.h"
bool joined = false;

static osjob_t sendjob;
const unsigned TX_INTERVAL = 150;

//***************************
// Pins und Sensoren
//***************************
Adafruit_BME280 bme;  // I2C, depending on your BME, you have to use address 0x77 (default) or 0x76, see below
#define BME_ADDR 0x77 // use address 0x77 (default) or 0x76

VL6180X s_vlx6180; // I2C, Pololu VL6180X Time-of-Flight Distance Sensor adress 0x29

//#define PIN_SONIC_TRIG 3
//#define PIN_SONIC_ECHO 4

/*
#define TENSIOMETER_PRESSURE_PIN A0
const float VminTyp = 0.2f;
const float VmaxTyp = 4.7f;
const float VrangeTyp = VmaxTyp - VminTyp;
const float maxPressure = 500.0f;
*/

/*
*   Testboard Pins  Tensiometer Eingang A1
*   Schwimmer 1 D11
*   Schwimmer 2 D12
*   Taster D13 //Wiederstand??
*   Relay OUT   A5 D?????
*/

#define TENSIOMETER_PRESSURE_PIN A1
const float VminTyp = 0.2f;
const float VmaxTyp = 4.7f;
const float VrangeTyp = VmaxTyp - VminTyp;
const float maxPressure = 500.0f;

// Water Level Sensors
#define PIN_RELEASE_BUTTON 13   //Buttton to reset Error
#define PIN_WATER_TANK_EMPTY 11 //Tank (A4)
#define PIN_FLOWER_POT_FULL 12  //flowers

#define PIN_RELAY 19

typedef enum { 
    READ_SENSORS,
    SEND_DATA,
    STANDBY,
    PUMP_START,
    PUMP_STOP,
    NUM_STATES } state_t;

typedef struct {
    uint32_t            irrigationInterval, // in milliseconds
                        irrigationDuration,
                        irrigationPause,
                        durationIrrigationPause,
                        txInterval,
                        defaultSleepTime;
    float               tensiometerMinPressure;
} hochbeet_config_t;

typedef struct {
    hochbeet_config_t   *config;
    uint32_t            timeLastPumpStart,
                        timeLastIrrigationStart;
    float               temperature,
                        humidity,
                        airPressure,
                        tensiometerPressure,
                        tensiometerInternalWaterLevel;
    uint32_t            timeLastDataSent;
    boolean             waterTankEmpty = true;
    boolean             flowerPotFull = false;
} instance_data_t ;



typedef state_t state_func_t( instance_data_t *data );

state_t do_state_read_sensors( instance_data_t *data );
state_t do_state_send_data( instance_data_t *data );
state_t do_state_standby( instance_data_t *data );
state_t do_state_pump_start( instance_data_t *data );
state_t do_state_pump_stop( instance_data_t *data );


state_func_t* const state_table[ NUM_STATES ] = {
    do_state_read_sensors, do_state_send_data, do_state_standby, do_state_pump_start, do_state_pump_stop
};

state_t run_state( state_t cur_state, instance_data_t *data ) {
    return state_table[ cur_state ]( data );
};

enum STATES
{
    PumpeAus,
    PumpeAn,
    Standby,
    Error
};

state_t do_state_read_sensors(instance_data_t *data) {
    // BME 280
    bme.takeForcedMeasurement();
    data->temperature = bme.readTemperature();
    data->humidity = bme.readHumidity();
    data->airPressure = bme.readPressure();

    // Water Tank @TODO validate correct assignment
    data->waterTankEmpty = digitalRead(PIN_WATER_TANK_EMPTY) == 1 ? false : true;

    // Flower Pot @TODO validate correct assignment
    data->flowerPotFull  = digitalRead(PIN_FLOWER_POT_FULL)  == 1 ? false : true;


    // Read tensiometer (soil moisture)
    data->tensiometerPressure = readTensiometerPressure();
    data->tensiometerInternalWaterLevel = readTensiometerInternalWaterLevel();

    return run_state(SEND_DATA, data);
}

state_t do_state_send_data(instance_data_t *data) {
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

    // TODO
    data->timeLastDataSent = rtc.getEpoch();
    return run_state(STANDBY, data);
}

state_t do_state_standby( instance_data_t *data ) {
    while(true) {
        if(rtc.getEpoch() > data->timeLastDataSent + data->config->txInterval) {
            return run_state(READ_SENSORS, data);
        }

        if(!data->waterTankEmpty
            && rtc.getEpoch() > data->timeLastIrrigationStart + data->config->irrigationInterval
            && data->tensiometerPressure <= data->config->tensiometerMinPressure) {
            
            return run_state(PUMP_START, data);
        }

        deepSleep(data->config->defaultSleepTime);
    }
}

state_t do_state_pump_start( instance_data_t *data ) {
    // If starting a new irrigation, set start time (note: a single irrigation
    // consists of multiple irrigation intervalls)
    if(data->timeLastIrrigationStart == 0) {
        data->timeLastIrrigationStart = rtc.getEpoch();
    }

    // Set start of current irrigation intervall
    data->timeLastPumpStart = rtc.getEpoch();

    pumpeStart();

    while(true) {
        // Water Tank @TODO validate correct assignment
        data->waterTankEmpty = digitalRead(PIN_WATER_TANK_EMPTY) == 1 ? false : true;

        // Flower Pot @TODO validate correct assignment
        data->flowerPotFull  = digitalRead(PIN_FLOWER_POT_FULL)  == 1 ? false : true;

        if( data->waterTankEmpty
            || data->flowerPotFull
            || rtc.getEpoch() > data->timeLastPumpStart + data->config->irrigationInterval) {
                return run_state(PUMP_STOP, data);
            }
    }
}

state_t do_state_pump_stop ( instance_data_t *data ) {
    pumpeStop();

    while(true) {
        // Irrigation complete or water tank empty (has to be checked first!)
        if(data->waterTankEmpty
            || rtc.getEpoch() > data->timeLastIrrigationStart + data->config->irrigationDuration) {
                return run_state(STANDBY, data);
        }

        // Continue irrigation after pause
        if(rtc.getEpoch() > data->timeLastPumpStart + data->config->irrigationInterval + data->config->irrigationPause
            && rtc.getEpoch() < data->timeLastIrrigationStart + data->config->irrigationDuration) {
                return run_state(PUMP_START, data);
        }

        deepSleep(data->config->defaultSleepTime);
    }
}

//***************************
// LMIC Events
//***************************
void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        LMIC_setDrTxpow(DR_SF8, 14);
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        {
            joined = true;
            u4_t netid = 0;
            devaddr_t devaddr = 0;
            u1_t nwkKey[16];
            u1_t artKey[16];
            LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
            Serial.print("netid: ");
            Serial.println(netid, DEC);
            Serial.print("devaddr: ");
            Serial.println(devaddr, HEX);
            Serial.print("artKey: ");
            for (int i = 0; i < sizeof(artKey); ++i)
            {
                Serial.print(artKey[i], HEX);
            }
            Serial.println("");
            Serial.print("nwkKey: ");
            for (int i = 0; i < sizeof(nwkKey); ++i)
            {
                Serial.print(nwkKey[i], HEX);
            }
            Serial.println("");
        }
        // Disable link check validation (automatically enabled
        // during join, but because slow data rates change max TX
        // size, we don't use it in this example.
        LMIC_setLinkCheckMode(0);
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen)
        {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }

        //sleepForSeconds(TX_INTERVAL);

        // Schedule next transmission
        //os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));
        break;
    default:
        Serial.print(F("Unknown event: "));
        Serial.println((unsigned)ev);
        break;
    }
}

//***************************
// Send data callback
//***************************
void do_send(osjob_t *j)
{
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {

        //****************
        // BME Read
        //****************
        bme.takeForcedMeasurement();

        // temp
        int temp_int = round((bme.readTemperature() + 50) * 100);
        Serial.print("Temp: ");
        Serial.println(temp_int);

        // pressure
        int pressure_int = round(bme.readPressure() / 100);
        Serial.print("Pressure: ");
        Serial.println(pressure_int);

        // humidity
        int hum_int = round(bme.readHumidity() * 100);
        Serial.print("Humidity: ");
        Serial.println(hum_int);

        //****************
        // Sonic Read
        //****************
        int distance = s_vlx6180.readRangeContinuousMillimeters(); //sonic();
                                                                   //Serial.print(s_vlx6180.readRangeContinuousMillimeters());
        if (s_vlx6180.timeoutOccurred())
        {
            distance = 255;
        }

        //****************
        // Payload
        //****************
        byte payload[8];
        // dht hum and temp
        payload[0] = highByte(hum_int);
        payload[1] = lowByte(hum_int);
        payload[2] = highByte(temp_int);
        payload[3] = lowByte(temp_int);
        payload[4] = highByte(pressure_int);
        payload[5] = lowByte(pressure_int);
        // distance (could also be encoded in one byte for the hochbeet?)
        payload[6] = highByte(distance);
        payload[7] = lowByte(distance);

        LMIC_setTxData2(1, (uint8_t *)payload, sizeof(payload), 0);
        Serial.println(F("Packet queued"));
    }
}

//***************************
// Setup
//***************************
void setup()
{

    // Serial.begin(115200);
    Serial.begin(9600);
    delay(10000); //Backup Delay to transfer sketch

    Serial.println(F("Starting"));

#ifdef RTC
    // configure RTC
    rtc.begin();
    rtc.setTime(14, 4, 30);
    rtc.setDate(6, 10, (uint8_t)70);
    rtc.setYear(19);

#endif
#ifdef DEBUG
    Serial.println(rtc.getEpoch());
#endif

#ifdef OLED
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setCursor(2, 0);
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.print("Booting");
    display.display();
    delay(1000);
#endif

    //***********************
    // Pins und Sensoren
    //***********************
    // Setup BME280
    //if (!bme.begin(BME280_ADDRESS)) {   <-- hat bei mir nicht richtig funktioniert (richtige Adresse war gesetzt ;) )
    if (!bme.begin())
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1) ;
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

    //pinMode(PIN_SONIC_TRIG, OUTPUT);
    //pinMode(PIN_SONIC_ECHO, INPUT);
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
    pinMode(PIN_RELEASE_BUTTON, INPUT);
    pinMode(PIN_WATER_TANK_EMPTY, INPUT);
    pinMode(PIN_FLOWER_POT_FULL, INPUT);
    //   attachInterrupt(digitalPinToInterrupt(PIN_RELEASE_BUTTON), releaseInt, HIGH);
    //attachInterrupt(digitalPinToInterrupt(PIN_WATER_TANK_EMPTY), pumpeStop1, HIGH);
    //  attachInterrupt(digitalPinToInterrupt(PIN_FLOWER_POT_FULL), pumpeStop, HIGH);

    //***********************
    //write data to Display before TTN Join
    //***********************
#ifdef OLED
    writeDisplay();
#endif

    //***********************
    // LMIC
    //***********************
#ifdef TTN

    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    do_send(&sendjob);
#endif

    //Testing LED ON (remove if Pump is connected )
    setRelay(1);
    digitalWrite(LED_BUILTIN, LOW);
}

//***************************
// Loop
//***************************

void loop() {
    os_runloop_once();
}

/**
 * Reads the internal water level of the
 * tensiometer
 */
float readTensiometerInternalWaterLevel()
{
    s_vlx6180.startRangeContinuous();
    u_int8_t numberOfSuccesfullMeasurements = 0;
    float distance = 0.0f;

    for (u_int8_t i = 0; i < 20; i++)
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

void printRTC()
{
    printdigit(rtc.getDay());
    Serial.print(".");
    printdigit(rtc.getMonth());
    Serial.print(".");
    printdigit(rtc.getYear());
    Serial.print("-");
    printdigit(rtc.getHours());
    Serial.print(":");
    printdigit(rtc.getMinutes());
    Serial.print(":");
    printdigit(rtc.getSeconds());
    Serial.print("\t");
}


void printdigit(int number)
{

    if (number < 10)
    {
        Serial.print('0');
    }
    Serial.print(number);
}
#ifdef OLED
// @todo has to be refactored to print state object
void writeDisplay() {
    /*int range = 0;
    display.clearDisplay();
    display.setCursor(1, 0);
    display.setTextColor(WHITE);
    //display.setCursor(25,1);
    display.setTextSize(1);
    display.print("Distance: ");
    for (int i = 0; i < 10; i++)
    {
        range += s_vlx6180.readRangeContinuousMillimeters();
    }
    range = range / 10;
    display.print(range);
    display.println(" mm");
    display.print("Sonic : ");
    display.print("N/A");
    display.println(" mm");
    display.print("Temp: ");
    display.print(temp);
    display.println(" C");
    display.print("Pressure: ");
    display.print(pressure / 100.0F);
    display.println(" p");
    display.print("Hum: ");
    display.print(hum);
    display.println(" p");
    display.print("Water_Tank_Empty: ");
    display.print(Water_Tank_Empty);
    display.println(" ");
    display.print("Flower_Pot_Full: ");
    display.print(Flower_Pot_Full);
    display.println(" ");
    display.print("Teniso: ");
    display.print(TENSIOMETER_PRESSURE);
    display.print(rtc.getSeconds());
    display.println(" ");
    display.display();*/
}
#endif

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

#ifdef RTC
/**
 * Called after MCU is waked up by RTC alarm.
 * 
 */
void alarmMatch()
{
#ifdef DEBUG
    Serial.print(F("alarmMatch() Woke up.\n"));
#endif

    rtc.detachInterrupt();
}

/**
 * Sets alarm to wake up the MCU after specific
 * amout of seconds. Works only with RTC.
 */
void alarmSet(int alarmDelaySeconds)
{
    int alarmTimeSeconds = rtc.getSeconds();
    int alarmTimeMinutes = rtc.getMinutes();

    alarmTimeMinutes = (alarmTimeMinutes + (alarmDelaySeconds / 60)) % 60;
    alarmTimeSeconds = (alarmTimeSeconds + (alarmDelaySeconds % 60)) % 60;

#ifdef DEBUG
    Serial.print("Setting next alarm timer:\n  (rtc.getMinutes / rtc.getSeconds / alarmDelaySeconds / alarmTimeMinutes / alarmTimeSeconds)\n  ");
    Serial.print(rtc.getMinutes());
    Serial.print(" / ");
    Serial.print(rtc.getSeconds());
    Serial.print(" / ");
    Serial.print(alarmDelaySeconds);
    Serial.print(" / ");
    Serial.print(alarmTimeMinutes);
    Serial.print(" / ");
    Serial.print(alarmTimeMinutes);
    Serial.print("\n");
#endif

    rtc.setAlarmSeconds(alarmTimeSeconds);
    rtc.setAlarmMinutes(alarmTimeMinutes);
    rtc.enableAlarm(rtc.MATCH_SS);

    rtc.attachInterrupt(alarmMatch);

#ifdef DEBUG
    Serial.println("alarmSet() complete");
#endif
}

#endif

/**
 * Powers down node for <seconds> s.
 */
void sleepForSeconds(int seconds)
{
    // Ensure all debugging messages are sent before sleep
    Serial.flush();

    // @todo kann eigentlich weg, oder?
#ifdef LOW_POWER
    // Going into sleep for more than 8 s – any better idea?
    int sleepCycles = round(seconds / 8);
    for (int i = 0; i < sleepCycles; i++)
    {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    }
#endif

#ifdef RTC
    alarmSet(seconds);
    Serial.end();

    rtc.standbyMode();

#endif
}

void deepSleep(uint32_t sleepTimeInMilliSeconds) {
    rtc.setAlarmEpoch(rtc.getEpoch() + sleepTimeInMilliSeconds);
    rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
    rtc.attachInterrupt(alarmMatch);
    rtc.standbyMode();
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