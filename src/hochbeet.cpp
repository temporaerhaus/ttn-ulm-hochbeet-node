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
#ifdef PiBME280
#include <Adafruit_BME280.h>
#endif
#include <Wire.h>
#ifdef VL6180
#include <VL6180X.h>
#endif
#ifdef LOW_POWER
#include <util/atomic.h>
#include <LowPower.h>
#endif

#ifdef RTClock
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

#ifdef adaRTCLIB
#include <RTClib.h>
String getDate();
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
RTC_DS3231 rtc;

#endif
// declaring functions
void onEvent(ev_t ev);
void do_send(osjob_t *j);
void do_logic(osjob_t *j);
void sleepForSeconds(int milliSeconds);
void setRelay(int state);
void pumpeStart();
void pumpeStop();
void writeDisplay();
float readTensiometerPressure();
float readTensiometerInternalWaterLevel();
void printdigit(int number);
void deepSleepRTC(uint32_t sleepTimeInMilliSeconds);
uint8_t read_srf02();

uint32_t getTime();

//***************************
// TTN und LMIC
//***************************
#include "config.h"
bool joined = false;

static osjob_t sendjob;
static osjob_t logicjob;

//***************************
// Pins und Sensoren
//***************************
//LMIC

// PIN MAP for MEGA
/*
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};*/

// PIN MAP for Feather M0
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {3, 6, LMIC_UNUSED_PIN},
};

#ifdef PiBME280
Adafruit_BME280 bme;  // I2C, depending on your BME, you have to use address 0x77 (default) or 0x76, see below
#define BME_ADDR 0x76 // use address 0x77 (default) or 0x76
#endif
#ifdef VL6180
VL6180X s_vlx6180; // I2C, Pololu VL6180X Time-of-Flight Distance Sensor adress 0x29
#endif
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

#define SRF02_ADDRESS 0x70

#define TENSIOMETER_PRESSURE_PIN A1
const float VminTyp = 0.2f;
const float VmaxTyp = 4.7f;
const float VrangeTyp = VmaxTyp - VminTyp;
const float maxPressure = 500.0f;

// Water Level Sensors
//#define PIN_RELEASE_BUTTON 13   //Buttton to reset Error
#define PIN_WATER_TANK_EMPTY 12 //Tank (A4)
#define PIN_FLOWER_POT_FULL 13  //flowers

#define PIN_RELAY A5

typedef enum { 
    READ_SENSORS,
    SEND_DATA,
    STANDBY,
    PUMP_START,
    PUMP_RUN,
    PUMP_STOP,
    NUM_STATES } state_t;

// strcut to config & parameters
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
                        tensiometerPressure;
    uint32_t            timeLastDataSent;
    uint8_t             tensiometerInternalWaterLevel;                        
    boolean             waterTankEmpty;
    boolean             flowerPotFull;
    boolean             irrigationRunning;
} instance_data_t ;

// unpretty workaround to access state ojects within LMIC OC jobs
state_t cur_state = READ_SENSORS; // set initial state to READ_SENSORS
hochbeet_config_t hochbeet_config = {
    .irrigationIntervalSec = (uint32_t)2 * 60, // 8 h
    .irrigationDurationSec = (uint32_t) 1 * 60, // 5 min
    .irrigationPauseSec = (uint32_t)30, // 30 s
    .txIntervalSec = (uint32_t) 1 * 30, // 5 * 60  // 5 min
    .defaultSleepTimeSec = 1, // 1 s
    .tensiometerMinPressure = 1000.0f, // 70 mBar
};
instance_data_t hochbeet_data = { &hochbeet_config, 0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, true, true, false };
static byte payload[20];


typedef state_t state_func_t( instance_data_t *data );

state_t do_state_read_sensors( instance_data_t *data );
state_t do_state_send_data( instance_data_t *data );
state_t do_state_standby( instance_data_t *data );
state_t do_state_pump_start( instance_data_t *data );
state_t do_state_pump_run( instance_data_t *data );
state_t do_state_pump_stop( instance_data_t *data );


state_func_t* const state_table[ NUM_STATES ] = {
    do_state_read_sensors, do_state_send_data, do_state_standby, do_state_pump_start, do_state_pump_run, do_state_pump_stop
};

state_t run_state( state_t cur_state, instance_data_t *data ) {
    #ifdef DEBUG1
    Serial.println("run_state");
    #endif
    return state_table[ cur_state ]( data );
};

// reads sensors and stores measurements
state_t do_state_read_sensors(instance_data_t *data) {
    Serial.println("do_state_read_sensors");
    // BME 280
    #ifdef PiBME280

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

    #else
    data->temperature = 66.66;
    data->humidity = 66.66;
    data->airPressure = 66.66;
    #endif


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

    #ifdef TTN
    // schedule sendjob
    os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(1), do_send);
    #endif

    data->timeLastDataSent = getTime();
    return STANDBY;
}

state_t do_state_standby( instance_data_t *data ) {
    Serial.println("do_state_standby");
        // Serial.print(" "); Serial.println(digitalRead(PIN_WATER_TANK_EMPTY) == 1 ? true : false);
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

    sleepForSeconds(data->config->defaultSleepTimeSec);
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

//***************************
// LMIC Events
//***************************
bool isLogicRunning = false;

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

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
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
            Serial.println();

        }
        // Disable link check validation (automatically enabled
        // during join, but because slow data rates change max TX
        // size, we don't use it in this example.
        LMIC_setLinkCheckMode(0);

        if(!isLogicRunning) {
             // Start first logic job here
            os_setTimedCallback(&logicjob, os_getTime()+sec2osticks(1), do_logic);
            isLogicRunning = true;
        }
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;

    case EV_JOIN_TXCOMPLETE:
        Serial.println(F("EV_JOIN_TXCOMPLETE"));
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
    Serial.print("Executing function: ");
    Serial.println(__func__);
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
        Serial.println(F("Packet queued"));
    }
}

void do_logic(osjob_t *j) {
    cur_state = run_state( cur_state, &hochbeet_data );

    // Start next logic job here
    os_setTimedCallback(&logicjob, os_getTime()+sec2osticks(1), do_logic);
}

//***************************
// Setup
//***************************
void setup()
{

    // Serial.begin(115200);
    Serial.begin(9600);
    delay(5000); //Backup Delay to transfer sketch

    Serial.println(F("Starting"));
#ifdef adaRTCLIB
// Initialize RTC
     if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
    if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
 //    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
#endif

#ifdef RTClock
    // configure RTC
    rtc.begin();
    rtc.setTime(14, 4, 30);
    rtc.setDate(6, 10, (uint8_t)70);
    rtc.setYear(19);

#endif
#ifdef DEBUG
    Serial.println(getTime());
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
        #ifdef PiBME280

    if (!bme.begin(BME_ADDR))
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1) ;
    }
#endif
    // Set BME in force mode to reduce power consumption
    // force mode = measure, store results, and go into sleep mode
    // until next measurement, see
    // - http://tinkerman.cat/low-power-weather-station-bme280-moteino/
    // - https://github.com/adafruit/Adafruit_BME280_Library/blob/master/examples/advancedsettings/advancedsettings.ino
    // values taken from example of 'Weather / Climate Monitor"
        #ifdef PiBME280

    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF);
#endif
    //pinMode(PIN_SONIC_TRIG, OUTPUT);
    //pinMode(PIN_SONIC_ECHO, INPUT);
    pinMode(PIN_RELAY, OUTPUT);

    //***********************
    // VL6180X
    //***********************

    Wire.begin();
    #ifdef VL6180
        s_vlx6180.init();
    s_vlx6180.configureDefault();
    s_vlx6180.setTimeout(500);
    // stop continuous mode if already active
    s_vlx6180.stopContinuous();
    #endif
    delay(500);

    //***********************
    //Interrupts for Sensors
    //***********************
    //pinMode(PIN_RELEASE_BUTTON, INPUT);
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

 /*  
     os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    LMIC_startJoining();
*/

      // LMIC init Kai
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  //LMIC_setAdrMode(1);
  //LMIC_setLinkCheckMode(1);
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  // Start job (sending automatically starts OTAA too)
  //os_setCallback(&sendjob, do_send);
  LMIC_startJoining();
  #if OTAA == false
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  //do_send(&sendjob);

    // Start first logic job here
    // os_setCallback(&logicjob, do_logic);
#else
os_init();
os_setTimedCallback(&logicjob, os_getTime()+sec2osticks(1), do_logic);
#endif

    //Testing LED ON (remove if Pump is connected )
    //setRelay(1);
    //digitalWrite(LED_BUILTIN, LOW);
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
    #ifdef VL6180
    
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
    #else
    return 1.1;
    #endif

}

#ifdef RTClock
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
#endif

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

#ifdef RTClock
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
 * Returns time since system start. Dependend on the board's 
 * capabilities using rtc or millis.
 */
uint32_t getTime() {
    #ifdef LOW_POWER
        return millis();
    #endif

    #ifdef RTClock
        return rtc.getEpoch();
    #endif

    #ifdef adaRTCLIB
        DateTime now = rtc.now();
        return now.unixtime();
    #endif

}

/**
 * Powers down node for <milliseconds> ms.
 */
void sleepForSeconds(int seconds)
{
    delay(seconds);
    return; // TODO
    // is there any critical job within sleep time?
    if(os_queryTimeCriticalJobs(sec2osticks(seconds)) == 1) {
        // do not sleep
        return;
    }

    // Ensure all debugging messages are sent before sleep
    Serial.flush();

    // @todo kann eigentlich weg, oder?
#ifdef LOW_POWER
    // Going into sleep for more than 8 s – any better idea?
    int sleepCycles = round(milliSeconds/1000 / 8);
    for (int i = 0; i < sleepCycles; i++)
    {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    }

    // Update the millis() and micros() counters, so duty cycle
    // calculations remain correct. This is a hack, fiddling with
    // Arduino's internal variables, which is needed until
    // https://github.com/arduino/Arduino/issues/5087 is fixed.
    // taken from https://github.com/meetjestad/mjs_firmware/blob/2333164163be7b569846270ff28122637bfe8e78/mjs_firmware.ino#L232-L238
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      extern volatile unsigned long timer0_millis;
      extern volatile unsigned long timer0_overflow_count;
      timer0_millis += sleepCycles * 8 * 1000;
      // timer0 uses a /64 prescaler and overflows every 256 timer ticks
      timer0_overflow_count += microsecondsToClockCycles((uint32_t)sleepCycles * 8 * 1000 * 1000) / (64 * 256);
    }
#endif

#ifdef RTClock
    deepSleepRTC(seconds);
#endif
}

#ifdef RTClock
void deepSleepRTC(uint32_t sleepTimeInSeconds) {
    uint32_t sleepTimeInMilliSeconds = sleepTimeInSeconds * 1000;
    rtc.setAlarmEpoch(rtc.getEpoch() + sleepTimeInMilliSeconds);
    rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
    rtc.attachInterrupt(alarmMatch);
    rtc.standbyMode();
}
#endif

uint8_t read_srf02(){
     byte byteHigh, byteLow;
  int distance;

  Wire.beginTransmission(SRF02_ADDRESS); // begin communication with SFR02
  Wire.write(byte(0x00));          // sets register pointer to the command register (0x00)
  Wire.write(byte(0x51));          // command sensor to measure in centimeters
  Wire.endTransmission();          // end transmission

  // Wait for 70 milliseconds
  delay(70); 
  
  Wire.beginTransmission(SRF02_ADDRESS); // begin communication with SFR02
  Wire.write(byte(0x02));          // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();          // end transmission
   
  Wire.requestFrom(SRF02_ADDRESS, 2);    // Request 2 bytes from SFR02
  
  while(Wire.available() < 2);     // Wait for the bytes to arrive
  
  // Read the values
  byteHigh = Wire.read(); 
  byteLow = Wire.read();
 
  // Calculate full bearing
  distance = ((byteHigh<<8) + byteLow);

  // Print data to Serial Monitor window
  #ifdef DEBUG1
  Serial.print("$Distance,");
  Serial.println(distance);
  #endif
  // In the demo wait half a second
  return distance;

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