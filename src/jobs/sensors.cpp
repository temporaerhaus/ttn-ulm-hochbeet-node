#include "sensors.h"
#include "config/pins.h"
#include "config/control.h"
#include <Adafruit_ADS1015.h>
#include <VL6180X.h>

void bubblesort(float *array, int length);

float read_tank_distance_sensor() {
    #ifdef TESTMODE
    return 20.0;
    #else
    int numberOfSamples = 5;

    // clear trigger pin
    digitalWrite(PIN_TANK_DISTANCE_TRIGGER, 0);
    delayMicroseconds(1000); // 10ms

    float measurements[numberOfSamples];

    unsigned long start = millis();
    float duration, distance;
    Serial.print("Reading tank distance...");
    for (uint8_t i = 0; i < numberOfSamples; i++) {

        // trigger sensor
        digitalWrite(PIN_TANK_DISTANCE_TRIGGER, 1);
        delayMicroseconds(500); // 5 ms
        digitalWrite(PIN_TANK_DISTANCE_TRIGGER, 0);

        // calculate distance via duration
        duration = pulseIn(PIN_TANK_DISTANCE_ECHO, 1);
        distance = duration * 0.032/2;
        Serial.print(distance); Serial.print(" ");
        measurements[i] = distance;
    }
    Serial.println("");
    Serial.print("[DEBUG] Distance measurements took ");
    unsigned long took = millis() - start;
    Serial.println(took);

    // calculate median
    bubblesort(measurements, numberOfSamples);

    float median = measurements[numberOfSamples / 2];
    Serial.print("Median for tank distance: ");
    Serial.println(median);
    return median;
    #endif
}

// bubble sort is O(n^2), but good enough for this and simple enough
// to know what's happening.
void bubblesort(float *array, int length) {
    int i, j, tmp;
    for (i = 1; i < length ; i++) {
        for (j = 0; j < length - i ; j++) {
            if (array[j] > array[j + 1]) {
                tmp = array[j];
                array[j] = array[j + 1];
                array[j + 1] = tmp;
            }
        }
    }
}

boolean read_water_tank_status() {
    #ifdef TESTMODE
    Serial.println("[TESTMODE] Water tank status. Returning false (not empty).");
    return false;
    #else
    return digitalRead(PIN_WATER_TANK_EMPTY) == 1 ? false : true;
    #endif

}

boolean read_flower_pot_status(){
    #ifdef TESTMODE
    Serial.println("[TESTMODE] Flower pot status. Returning false (not full).");
    return false;
    #else
    return digitalRead(PIN_FLOWER_POT_FULL) == 1 ? true : false;
    #endif
}

float read_watertank_pressure(Adafruit_ADS1115& ads) {
    // We use external  16 Bit ADC ADS1115. It has an resolution of 188uV/bit
    #ifdef TESTMODE
    Serial.println("[TESTMODE] Water tank pressure. Returning 0.0f");
    return 0.0f;
    #else
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
    #endif
}

/**
 * Reads to inner pressure of the tensiometer which
 * indicates how moist the soil is.
 */
float read_tensiometer_pressure(Adafruit_ADS1115& ads) {
    #ifdef TESTMODE
    Serial.println("[TESTMODE] Tensiometer pressure. Returning 50.0f");
    return 50.0f;
    #else
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
    #endif
}


/**
 * Reads batery voltage
 */
float read_battery(Adafruit_ADS1115& ads) {
#ifdef TESTMODE
    Serial.println("[TESTMODE] BAttery Voltage pressure. Returning 5.0f");
    return 5.0f;
#else
    // We use external  16 Bit ADC ADS1115. It has an resolution of 188uV/bit
    int rawValue = ads.readADC_SingleEnded(3);
    float voltage = rawValue * 188.0f / 1000000.0f;
    int gain = 10;
    voltage = (voltage * gain);


#ifdef DEBUG
    Serial.print(rawValue);
    Serial.print(" / ");
    Serial.print(voltage);
    Serial.print(" V");
    Serial.print(" / ");
    Serial.print(voltage);
    Serial.print(" kPa  ");
    Serial.println(" ");
#endif
    return voltage;
#endif
}


/**
 * Reads the internal water level of the
 * tensiometer
 */
float read_tensiometer_internal_water_level(VL6180X& s_vlx6180) {
    #ifdef TESTMODE
    Serial.println("[TESTMODE] Tensiometer internal water level. Returning 1.0f");
    return 1.0f;
    #else
    Serial.println("Reading internal tensiometer water level...");

    s_vlx6180.startRangeContinuous();
    uint8_t numberOfSuccesfullMeasurements = 0;
    float distance = 0.0f;

    for (uint8_t i = 0; i < 5; i++) {
        float currentDistance = s_vlx6180.readRangeContinuousMillimeters();
        if (!s_vlx6180.timeoutOccurred()) {
            distance += currentDistance;
            numberOfSuccesfullMeasurements += 1;
        }
    }

    s_vlx6180.stopContinuous();

    if (numberOfSuccesfullMeasurements > 0) {
        distance = distance/numberOfSuccesfullMeasurements;
    }

    return distance;
    #endif
}

