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

    // init LMIC_OS
    os_init();
    LMIC_reset();

    // init jobs
    logic_job_init();
    lora_job_init();
}

//***************************
// Loop
//***************************
void loop() {
    os_runloop_once();
}