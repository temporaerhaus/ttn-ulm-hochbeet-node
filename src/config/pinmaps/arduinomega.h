#ifndef MEGA_PIN_MAP_H
#define MEGA_PIN_MAP_H


// PIN MAP for MEGA
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

#endif