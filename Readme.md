# Hochbeet Verschwörhaus

Repository for the LoRaWAN raised garden microcontroller at the Verschwörhaus Ulm.

An self-sufficient raised garden bed with two watertanks and a soil moisture sensor that checks if the bed should
be watered or not.

Some additional sensors are deployed to check if the water tanks are still full and the bed does not overflow with water.

A BME280 sensor checks additional environment values (temperature, air humidity and pressure).

This project is based on platformio.org and our system runs on a Adafruit Feather M0, but is also or should also be compatible to 
other systems and chips. 

We provide a Fritzing sketch to outline the wiring. See `fritzing` folder.

## Operation modus
* Every x hours (configurable) the soil moisture is checked.
* If over a certain pre-configured value, the pump is started for a given time.
* A float switch protects the raised bed from overlowing with water.
* Another float switch protects the pump from running dry.
* A ultrasonic distance sensor measure the fill leven inside the tank.

The whole system runs fully self sufficient on a solar cell and a large rechargeable battery pack.

## Setup
* Copy `src/config/ttn-example.h` to `src/config/ttn.h` and fill in your TTN credentials.
* Double-check if the MCCI LMIC library has the correct regional setting (see `project_config` folder).
* Check platformio.ini to enbale/disable build flags. It's possible to switch between ABP and OTAA there, among other settings.

## History
* Autumn 2019: First ideas and prototypes. "What could we all do with this...".
* Autumn/Winter: What sensors to use, what's the best way to build this.
* Early 2020: First working code and an improved LMIC-based state machine for LoRa applications. The "PCB" looks is going strong.
* Spring 2020: Corona-break. We all remember this.
* June 2020: We are back! But where did we left off?
* Also June 2020: Things are going fast now.
* Mid-July 2020: The real raised garden is finished at the Verschwörhaus.

## Contributors
* Jakob Pietron
* Kai-Uwe Piazzi
* Matthias Schneider
* Gerhard Reisinger

## Web
* https://lora.ulm-digital.com/
* https://verschwoerhaus.de/
* https://twitter.com/ttn_ulm

## License
MIT license. See license file.

