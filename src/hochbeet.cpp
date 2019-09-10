/**
 * Code für den Hochbeet-Arduino für's Verschwörhaus.
 *
 *
 */

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#ifdef LOW_POWER
    #include "LowPower.h"
#endif

#ifdef RTC
    #include <RTCZero.h>
    RTCZero rtc;
#endif

// declaring functions
void onEvent(ev_t ev);
void do_send(osjob_t* j);
int sonic();
void sleepForSeconds(int seconds);

//***************************
// TTN und LMIC
//***************************
#include "config.h"

static osjob_t sendjob;
const unsigned TX_INTERVAL = 60 * 10;

// Pinmapping for Dragino Arduino shield
const lmic_pinmap lmic_pins = {
        .nss = 10,
        .rxtx = LMIC_UNUSED_PIN,
        .rst = 9,
        .dio = {2, 6, 7},
};

//***************************
// Pins und Sensoren
//***************************
Adafruit_BME280 bme; // I2C, depending on your BME, you have to use address 0x77 (default) or 0x76, see below
#define BME_ADDR 0x76 // use address 0x77 (default) or 0x76
#define PIN_SONIC_TRIG 3
#define PIN_SONIC_ECHO 4


//***************************
// LMIC Events
//***************************
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
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
                for (int i=0; i<sizeof(artKey); ++i) {
                    Serial.print(artKey[i], HEX);
                }
                Serial.println("");
                Serial.print("nwkKey: ");
                for (int i=0; i<sizeof(nwkKey); ++i) {
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
            if (LMIC.dataLen) {
                Serial.print(F("Received "));
                Serial.print(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
            }
            
            sleepForSeconds(TX_INTERVAL);

            // Schedule next transmission to be immediately after this
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);

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
            Serial.println((unsigned) ev);
            break;
    }
}


//***************************
// Send data callback
//***************************
void do_send(osjob_t* j){
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {

        //****************
        // BME Read
        //****************
        bme.takeForcedMeasurement();

        // temp
        int temp_int = round(bme.readTemperature() * 100);
        Serial.print("Temp: ");
        Serial.println(temp_int);

        // pressure
        int pressure_int = round(bme.readPressure()/100);
        Serial.print("Pressure: ");
        Serial.println(pressure_int);

        // humidity
        int hum_int = round(bme.readHumidity() * 100);
        Serial.print("Humidity: ");
        Serial.println(hum_int);

        //****************
        // Sonic Read
        //****************
        int distance = sonic();

        //****************
        // Payload
        //****************
        byte payload[8];
        // dht hum and temp
        payload[0] = highByte(hum_int);
        payload[1] = lowByte(hum_int);
        payload[2] = highByte(temp_int);
        payload[3] = lowByte(temp_int);
        payload[4] = lowByte(pressure_int);
        payload[5] = lowByte(pressure_int);
        // distance (could also be encoded in one byte for the hochbeet?)
        payload[6] = highByte(distance);
        payload[7] = lowByte(distance);

        LMIC_setTxData2(1, (uint8_t*)payload, sizeof(payload), 0);
        Serial.println(F("Packet queued"));
    }
}


//***************************
// Setup
//***************************
void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    #ifdef RTC
        // configure RTC
        rtc.begin();
        rtc.setTime(0, 0, 0);
        rtc.setDate(1, 1, (uint8_t)1970);
    #endif

    //***********************
    // Pins und Sensoren
    //***********************
    // Setup BME280
    if (!bme.begin(BME280_ADDRESS)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
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
                        Adafruit_BME280::FILTER_OFF   );


    pinMode(PIN_SONIC_TRIG, OUTPUT);
    pinMode(PIN_SONIC_ECHO, INPUT);

    //***********************
    // LMIC
    //***********************
    os_init();
    LMIC_reset();
    LMIC.dn2Dr = DR_SF9;
    LMIC_setDrTxpow(DR_SF7, 14);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}


//***************************
// Loop
//***************************
void loop() {
    os_runloop_once();
}


//***************************
// Read sonic distance
//***************************
int sonic() {
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
    distance = (duration/2) * 0.03432;

    if (distance >= 500 || distance <= 0) {
        Serial.print("Kein gültiger Wert: ");
        Serial.println(distance);
    } else {
        Serial.print(distance);
        Serial.println(" cm");
    }

    delay(100);

    return distance;
}

#ifdef RTC
void alarmMatch() {
    #ifdef DEBUG
      SerialUSB.print(F("alarmMatch() Woke up.\n"));
    #endif

    rtc.detachInterrupt();
}

void alarmSet(int alarmDelaySeconds) {
    int alarmTimeSeconds = rtc.getSeconds();
    int alarmTimeMinutes = rtc.getMinutes();

    alarmTimeMinutes = (alarmTimeMinutes + ( alarmDelaySeconds / 60 )) % 60;
    alarmTimeSeconds = (alarmTimeSeconds + ( alarmDelaySeconds % 60 )) % 60;

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
      SerialUSB.println("alarmSet() complete") ;
    #endif
}



#endif

void sleepForSeconds(int seconds) {
    // Ensure all debugging messages are sent before sleep
    Serial.flush();
    
    #ifdef LOW_POWER
        // Going into sleep for more than 8 s – any better idea?
        int sleepCycles = round(seconds / 8) ;
        for(int i = 0; i < sleepCycles; i++) {
              LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
        }
    #endif

    #ifdef RTC
        alarmSet(seconds);
        rtc.standbyMode();
    #endif
}