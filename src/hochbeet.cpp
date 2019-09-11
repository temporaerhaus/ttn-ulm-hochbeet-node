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
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// declaring functions
void onEvent(ev_t ev);
void do_send(osjob_t *j);
int sonic();
void setRelay(int state);
void pumpeStop();


//***************************
// TTN und LMIC
//***************************
#include "config.h"

static osjob_t sendjob;
const unsigned TX_INTERVAL = 60 * 5;


// Adafruit Feather M0
const lmic_pinmap lmic_pins = {
    .nss = 8,  
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {3,6,LMIC_UNUSED_PIN},
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
DHT dht;
Adafruit_BME280 bme;
#define PIN_DHT 5
#define LevelSensor_01 3
#define PIN_RELAY 11
#define PIN_SONIC_TRIG 12
#define PIN_SONIC_ECHO 13
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
        // DHT Read
        //****************
        Serial.println(F("DHT wait"));
      //  delay(dht.getMinimumSamplingPeriod());
        Serial.println(F("DHT read"));
        float hum =  bme.readHumidity();
        int hum_int = hum * 100;
        float temp = bme.readTemperature();
        int temp_int = (temp+50) * 100;
        Serial.print(hum);
        Serial.print("\t");
        Serial.print(temp);
        Serial.println("");

        //****************
        // Sonic Read
        //****************
        int distance = 12;//sonic();

        //****************
        // Payload
        //****************
        byte payload[6];
        // dht hum and temp
        payload[0] = highByte(hum_int);
        payload[1] = lowByte(hum_int);
        payload[2] = highByte(temp_int);
        payload[3] = lowByte(temp_int);
        // distance (could also be encoded in one byte for the hochbeet?)
        payload[4] = highByte(distance);
        payload[5] = lowByte(distance);

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
    delay(10000);
    Serial.println(F("Starting"));

    //***********************
    // Pins und Sensoren
    //***********************
    //dht.setup(PIN_DHT);

    pinMode(PIN_SONIC_TRIG, OUTPUT);
    pinMode(PIN_SONIC_ECHO, INPUT);
    pinMode(PIN_RELAY, OUTPUT);
    pinMode(LevelSensor_01,INPUT);

  //***********************
  // BME280
  //***********************
  
  if (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }


    //***********************
    // LMIC
    //***********************
    //Start Kai
     os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    //StartKai Ende
    
    //Start Mathias
    /*
    os_init();
    LMIC_reset();
    LMIC.dn2Dr = DR_SF9;
    LMIC_setDrTxpow(DR_SF7, 14);
    */
    //Start Mathias Ende

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);

    setRelay(1);
   // attachInterrupt(digitalPinToInterrupt(LevelSensor_01), pumpeStop,HIGH);

  
   
}

//***************************
// Loop
//***************************
void loop()
{
     os_runloop_once();

 /*      Serial.print("Temperature: ");
    Serial.println(bme.readTemperature());

    Serial.print("PressureAir: ");
    
    Serial.println(bme.readPressure()/100.0F);
*/
 /*   if(digitalRead(LevelSensor_01)==0)
    {
        //setRelay(0);    
         //Serial.println(analogRead(A3));
         Serial.println(digitalRead(LevelSensor_01));
         setRelay(1);

    }else{
        Serial.println(digitalRead(LevelSensor_01));
        // Serial.println(analogRead());
       // setRelay(1);
        //delay(5000);
       // setRelay(0);
    }
    sonic();
       delay(1000);
    Serial.println(F("DHT wait"));
    delay(dht.getMinimumSamplingPeriod());
    Serial.println(F("DHT read"));
    Serial.print("Hum: "); 
    Serial.println(dht.getHumidity());
  //      int hum_int = hum * 100;
    Serial.print("Temp: "); 
    Serial.println(dht.getTemperature()); 
   */ 
}

void pumpeStop(){
    setRelay(0);
}

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
