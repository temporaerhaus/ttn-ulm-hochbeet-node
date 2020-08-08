#include "sensors.h"
#include "config/pins.h"
#include "config/control.h"
#include <Adafruit_ADS1015.h>

void quicksort(float number[20], int first, int last);


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
    for (uint8_t i = 0; i < numberOfSamples; i++) {

        // trigger sensor
        digitalWrite(PIN_TANK_DISTANCE_TRIGGER, 1);
        delayMicroseconds(500); // 5 ms
        digitalWrite(PIN_TANK_DISTANCE_TRIGGER, 0);

        // calculate distance via duration
        duration = pulseIn(PIN_TANK_DISTANCE_ECHO, 1);
        distance = duration * 0.032/2;

        measurements[i] = distance;
    }
    Serial.print("[DEBUG] Distance measurements took ");
    unsigned long took = millis() - start;
    Serial.println(took);

    // calculate median
    quicksort(measurements, 0, numberOfSamples-1);

    float median = measurements[numberOfSamples / 2];
    Serial.print("Median for tank distance: ");
    Serial.println(median);
    return median;
    #endif
}

void quicksort(float number[], int first, int last){
    int i, j, pivot, temp;

    if(first<last){
        pivot=first;
        i=first;
        j=last;

        while(i<j){
            while(number[i]<=number[pivot]&&i<last)
                i++;
            while(number[j]>number[pivot])
                j--;
            if(i<j){
                temp=number[i];
                number[i]=number[j];
                number[j]=temp;
            }
        }

        temp=number[pivot];
        number[pivot]=number[j];
        number[j]=temp;
        quicksort(number,first,j-1);
        quicksort(number,j+1,last);

    }
}

boolean read_water_tank_status() {
    #ifdef TESTMODE
    Serial.println("Reading water tank nail sensor in TESTMODE. Returning false.");
    return false;
    #else
    return digitalRead(PIN_WATER_TANK_EMPTY) == 1 ? false : true;
    #endif

}

boolean read_flower_pot_status(){
    #ifdef TESTMODE
    Serial.println("Reading flower pot sensor in TESTMODE. Returning false (not full).");
    return false;
    #else
    return digitalRead(PIN_FLOWER_POT_FULL)  == 1 ? true : false;
    #endif
}

float read_watertank_pressure(Adafruit_ADS1115& ads) {
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
 * Reads to inner pressure of the tensiometer which
 * indicates how moist the soil is.
 */
float read_tensiometer_pressure(Adafruit_ADS1115& ads) {

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
