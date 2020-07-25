#include <Arduino.h>
#include "arduino_lmic.h"
#include <hal/hal.h>
#include <SPI.h>

#include "jobs/lora_job.h"
#include "jobs/logic_job.h"

#ifdef DEBUG
  #define print(x) Serial.print(x);
  #define println(x) Serial.println(x);
#else
  #define print(x)
  #define println(x)
#endif

//***************************
// Setup
//***************************
void setup()
{
    Serial.begin(9600);
    delay(5000); //Backup Delay to transfer sketch

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