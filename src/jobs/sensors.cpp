#include "sensors.h"
#include "config/control.h"
#include "config/pins.h"
#include <Adafruit_ADS1015.h>

void quicksort(float number[20], int first, int last);

float read_tank_distance_sensor() {
    #ifdef TESTMODE
    return 20.0;
    #else
    int numberOfSamples = 20;

    // clear trigger pin
    digitalWrite(PIN_TANK_DISTANCE_TRIGGER, 0);
    delayMicroseconds(10);

    float measurements[numberOfSamples];

    unsigned long start = millis();
    float duration, distance;
    for (uint8_t i = 0; i < numberOfSamples; i++) {

        // trigger sensor
        digitalWrite(PIN_TANK_DISTANCE_TRIGGER, 1);
        delayMicroseconds(100); // = 1ms
        digitalWrite(PIN_TANK_DISTANCE_TRIGGER, 0);

        // calculate distance via duration
        duration = pulseIn(PIN_TANK_DISTANCE_ECHO, 1);
        distance = duration * 0.032/2;

        measurements[i] = distance;
    }
    Serial.print("[DEBUG] Distance measurements took ");
    Serial.println(start - millis());

    // calculate median
    start = millis();
    quicksort(measurements, 0, numberOfSamples-1);
    Serial.print("[DEBUG] Distance quicksort took ");
    Serial.println(start - millis());

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
