#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_ADS1015.h>

float read_tank_distance_sensor();
bool read_water_tank_status();
bool read_flower_pot_status();
float read_watertank_pressure(Adafruit_ADS1115& ads);
float read_tensiometer_pressure(Adafruit_ADS1115& ads);
bool quicksort();

#endif