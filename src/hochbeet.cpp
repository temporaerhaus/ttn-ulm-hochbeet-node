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

// declaring functions
void onEvent(ev_t ev);
void do_send(osjob_t *j);
int sonic();
void sleepForSeconds(int seconds);
void setRelay(int state);
void pumpeStop();
void pumpeStop1();
void releaseInt();  


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
Adafruit_BME280 bme; // I2C, depending on your BME, you have to use address 0x77 (default) or 0x76, see below
#define BME_ADDR 0x77 // use address 0x77 (default) or 0x76

VL6180X s_vlx6180;  // I2C, Pololu VL6180X Time-of-Flight Distance Sensor adress 0x29

//#define PIN_SONIC_TRIG 3
//#define PIN_SONIC_ECHO 4

/*
#define TENSIOMETER_PRESSURE_PIN A0
const float VminTyp = 0.2f;
const float VmaxTyp = 4.7f;
const float VrangeTyp = VmaxTyp - VminTyp;
const float maxPressure = 500.0f;
*/

// schauen ob man die Level Sensoren auch auf digitale Pins setzen kann
#define ReleaseButton  17     //Buttton to reset Error
#define LevelSensor_01 18    //Tank
#define LevelSensor_02 19     //flowers
boolean level=false;            
boolean level2=false;  

  
#define PIN_RELAY 11
//#define PIN_SONIC_TRIG 12
//#define PIN_SONIC_ECHO 13
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
        int distance = 12;//sonic();

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
     while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
    }
    Serial.println(F("Starting before delay"));
    delay(10000);
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
    //if (!bme.begin(BME280_ADDRESS)) {
    if (!bme.begin()) {         
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


    //pinMode(PIN_SONIC_TRIG, OUTPUT);
   // pinMode(PIN_SONIC_ECHO, INPUT);
    pinMode(PIN_RELAY, OUTPUT);

// VL6180X

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
  // BME280
  //***********************
  
//  if (!bme.begin()) {  
//    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
//    while (1);
// }
//Interrupts for Sensors

pinMode(ReleaseButton,INPUT);
pinMode(LevelSensor_01,INPUT);
pinMode(LevelSensor_02,INPUT);
attachInterrupt(digitalPinToInterrupt(ReleaseButton), releaseInt ,HIGH);
attachInterrupt(digitalPinToInterrupt(LevelSensor_01), pumpeStop1,HIGH);
attachInterrupt(digitalPinToInterrupt(LevelSensor_02), pumpeStop,HIGH);

 


    //***********************
    // LMIC
    //***********************
    //Start Kai
//     os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
//    LMIC_reset();
//    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
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
   // do_send(&sendjob);

    setRelay(1);
   // attachInterrupt(digitalPinToInterrupt(LevelSensor_01), pumpeStop,HIGH);

  
   
}

//***************************
// Loop
//***************************
void loop()
{ 
   //  os_runloop_once();
   
  //Output Values s_vlx6180
  Serial.print("Ambient: ");
  Serial.print(s_vlx6180.readAmbientContinuous());
  if (s_vlx6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.print("\tRange: ");
  Serial.print(s_vlx6180.readRangeContinuousMillimeters());
  if (s_vlx6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
  #ifdef Test
      Serial.print(F("TESTPRINT"));
  #endif
  delay(2000);
  //sleepForSeconds(1) ;

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

void pumpeStop(){
    setRelay(0);
    level=true;

}
void pumpeStop1(){
    setRelay(0);
    level2=true;

}
void releaseInt(){
    level=false;
    level2=false;
    setRelay(1);
}

