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
int sonic();
void sleepForSeconds(int seconds);
void setRelay(int state);
void pumpeStop();
void pumpeStop1();
void releaseInt();
void writeDisplay();

//***************************
// TTN und LMIC
//***************************
#include "config.h"
bool joined = false;

static osjob_t sendjob;
const unsigned TX_INTERVAL = 60 * 5;

// Adafruit Feather M0
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {3, 6, LMIC_UNUSED_PIN},
};

// Pinmapping for Dragino Arduino shield
/*const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};
*/
// Pinmapping for LSN50 ??
/* 
const lmic_pinmap lmic_pins = {
        .nss =  PA15,         //38, PA15 pin
        .rxtx = LMIC_UNUSED_PIN,    
        .rst = PB0,        // 18,PB0
        .dio = {2,21,22},  //PC13,PB10,PB11
};
*/

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

// Water Level Sensors
#define ReleaseButton 17  //Buttton to reset Error
#define LevelSensor_01 10 //Tank
#define LevelSensor_02 12 //flowers
boolean level = false;
boolean level2 = false;

#define PIN_RELAY 11
//#define PIN_SONIC_TRIG 12
//#define PIN_SONIC_ECHO 13

//using foor LOOP
unsigned long millisNow = 0;

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

        sleepForSeconds(TX_INTERVAL);

        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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
    rtc.setTime(0, 0, 0);
    rtc.setDate(1, 1, (uint8_t)1970);
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
        while (1)
            ;
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
    s_vlx6180.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
    s_vlx6180.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
    //s_vlx6180.setScaling(1);
    s_vlx6180.setTimeout(500);
    // stop continuous mode if already active
    s_vlx6180.stopContinuous();
    delay(300);
    // start interleaved continuous mode with period of 100 ms
    s_vlx6180.startInterleavedContinuous(100);

    //***********************
    //Interrupts for Sensors
    //***********************
    pinMode(ReleaseButton, INPUT);
    pinMode(LevelSensor_01, INPUT);
    pinMode(LevelSensor_02, INPUT);
 //   attachInterrupt(digitalPinToInterrupt(ReleaseButton), releaseInt, HIGH);
 //   attachInterrupt(digitalPinToInterrupt(LevelSensor_01), pumpeStop1, HIGH);
 //   attachInterrupt(digitalPinToInterrupt(LevelSensor_02), pumpeStop, HIGH);

    //***********************
    //write data to Display before TTN Join
    //***********************
#ifdef OLED
    writeDisplay();
#endif

    //***********************
    // LMIC
    //***********************
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    do_send(&sendjob);

    //Testing LED ON (remove if Pump is connected )
    setRelay(1);
}

//***************************
// Loop
//***************************

unsigned long millisWriteDisplay = 0;
unsigned long millisSentTTN = 0;
void loop()
{

    if (joined == false) // joined hin und wieder true obwohl noch gar nicht joined ??
    {
#ifdef OLED
        display.clearDisplay();
        display.setCursor(2, 0);
        display.setTextColor(WHITE);
        display.setTextSize(2);
        display.print("Join TTN");
        display.display();
#endif
        os_runloop_once();
    }
    else
    {
        //***********************
        //schreibt alle 10 Sekunden die aktuellen Werte auf das Display
        //***********************
        //Problem with millis all 50days because reset to 0. Will it working if we use  rtc ??
        if (millis() - millisWriteDisplay > 10000)
        {
            millisWriteDisplay = millis();
#ifdef OLED
            writeDisplay();
#endif
        }

        if (millis() - millisSentTTN > 1000 * 1)
        {
            millisSentTTN = millis();
#ifdef OLED
   /*         display.clearDisplay();
            display.setCursor(2, 0);
            display.setTextColor(WHITE);
            display.setTextSize(2);
            display.print("send TTN");
            display.display();
            */
#endif
          //  os_runloop_once();
           //do_send(&sendjob);
        }
    }
    //  os_runloop_once();

    //Output Values s_vlx6180
    /*
  Serial.print("Ambient: ");
  Serial.print(s_vlx6180.readAmbientContinuous());
  if (s_vlx6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.print("\tRange: ");
  Serial.print(s_vlx6180.readRangeContinuousMillimeters());
  if (s_vlx6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
*/
os_runloop_once();
}

#ifdef OLED
void writeDisplay()
{
    int range = 0;
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
    display.print(bme.readTemperature());
    display.println(" C");
    display.print("Pressure: ");
    display.print(bme.readPressure() / 100.0F);
    display.println(" p");
    display.print("Hum: ");
    display.print(bme.readHumidity());
    display.println(" p");
    display.print("Level: ");
    display.print(level);
    display.println(" ");
    display.print("Level2: ");
    display.print(level2);
    display.println(" ");
    display.print("Teniso: ");
    display.print("N/A");
    display.println(" ");
    display.display();
}
#endif
//***************************
// set Relay state = 1 Relay on ; state=0 Relay off
//***************************
void setRelay(int state)
{
    digitalWrite(PIN_RELAY, state);
    //:return state;
}
//***************************
// Read sonic distance
//***************************
/*
int sonic()
{
    delay(300);

    long duration;
    long distance;

    // Kurz anschalten ausschalten, damit es nachher gleich rauschfreier ist.
    digitalWrite(PIN_SONIC_TRIG, LOW);
    delay(5);

    // Einmal kurz für 10ms einen ton senden
    digitalWrite(PIN_SONIC_TRIG, HIGH);
    delay(10);
    digitalWrite(PIN_SONIC_TRIG, LOW);

    // per pulseIn die Zeit abfragen, bis das Signal wieder zurück kam.
    duration = pulseIn(PIN_SONIC_ECHO, HIGH);
    // Die Dauer durch 2, weil wir nur eine Strecken wollen, multipliziert mit der Schallgeschwindigkeit in cm/ms.
    // Ergibt dann den Abstand in cm.
    distance = (duration / 2) * 0.03432;

    if (distance >= 500 || distance <= 0)
    {
        Serial.print("Kein gültiger Wert: ");
        Serial.println(distance);
    }
    else
    {
        Serial.print(distance);
        Serial.println(" cm");
    }

    delay(100);

    return distance;
}
*/
/*
float getTensiometerPressure() {
    int rawValue = analogRead(TENSIOMETER_PRESSURE_PIN);  // read the input pin

    // @todo auf 3V runterbrechen
    float voltage = (float) rawValue * (5.0 / 1023.0);
    voltage = (voltage < VminTyp) ? VminTyp : voltage;
    Serial.print(rawValue);  
    Serial.print(" / ");
    Serial.print(voltage);
    Serial.print(" V");
    float pressure = 1.0 / VrangeTyp * (voltage - VminTyp) * maxPressure;
    Serial.print (" / ");
    Serial.println(" kPa");
    Serial.print(pressure);
    return pressure;
}
*/

#ifdef RTC
void alarmMatch()
{
#ifdef DEBUG
    SerialUSB.print(F("alarmMatch() Woke up.\n"));
#endif

    rtc.detachInterrupt();
}

void alarmSet(int alarmDelaySeconds)
{
    int alarmTimeSeconds = rtc.getSeconds();
    int alarmTimeMinutes = rtc.getMinutes();

    alarmTimeMinutes = (alarmTimeMinutes + (alarmDelaySeconds / 60)) % 60;
    alarmTimeSeconds = (alarmTimeSeconds + (alarmDelaySeconds % 60)) % 60;

#ifdef DEBUG
    Serial.print(F("Setting next alarm timer:\n  (rtc.getMinutes / rtc.getSeconds / alarmDelaySeconds / alarmTimeMinutes / alarmTimeSeconds)\n  "));
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
    rtc.enableAlarm(rtc.MATCH_MMSS);
    rtc.attachInterrupt(alarmMatch);

#ifdef DEBUG
    SerialUSB.println("alarmSet() complete");
#endif
}

#endif

void sleepForSeconds(int seconds)
{
    // Ensure all debugging messages are sent before sleep
    Serial.flush();

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
    rtc.standbyMode();
#endif
}

void pumpeStop()
{
    setRelay(0);
    level = true;
}
void pumpeStop1()
{
    setRelay(0);
    level2 = true;
}
void releaseInt()
{
    level = false;
    level2 = false;
#ifdef OLED
    writeDisplay();
#endif
    setRelay(1);
}
