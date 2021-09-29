#include <Arduino.h>
#include "arduino_lmic.h"
#include <hal/hal.h>
#include <SPI.h>

#include "config/pins.h"
#include "jobs/lora_job.h"
#include "jobs/logic_job.h"

//***************************
// Setup
//***************************
void setup() {
    Serial.begin(9600);
    delay(5000); //Backup Delay to transfer sketch
    Serial.println(F("start"));

    // init LMIC_OS
    os_init();
    LMIC_reset();

    // init jobs
    lora_job_init();
    logic_job_init();
}

//***************************
// Loop
//***************************
void loop() {
    os_runloop_once();
}