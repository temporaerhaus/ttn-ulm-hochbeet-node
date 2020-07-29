#include "sensors.h"
#include "config/pins.h"

void quicksort(float number[20], int first, int last);

bool read_water_tank_float_sensor() {
    #ifdef TESTMODE
    Serial.println("Reading water tank float sensor in TESTMODE.");
    return false;
    #else
    return digitalRead(PIN_WATER_TANK_EMPTY) == 1 ? true : false;
    #endif
}


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
    Serial.print('[DEBUG] Distance measurements took ');
    Serial.println(start - millis());

    // calculate median
    start = millis();
    quicksort(measurements, 0, numberOfSamples-1);
    Serial.print('[DEBUG] Distance quicksort took ');
    Serial.println(start - millis());

    float median = measurements[numberOfSamples / 2];
    Serial.print('Median for tank distance: ');
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