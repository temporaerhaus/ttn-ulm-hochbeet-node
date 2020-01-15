//Put in TTN Credentials

#define OTAA true

#if OTAA == true
    // lsb
    static const u1_t PROGMEM DEVEUI[8]= { };
    void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

    // lsb
    static const u1_t PROGMEM APPEUI[8]= { };
    void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

    // msb
    static const u1_t PROGMEM APPKEY[16] = { };
    void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
#else
    // msb
    static const PROGMEM u1_t NWKSKEY[16] = { };
    // msb
    static const u1_t PROGMEM APPSKEY[16] = { };
    static const u4_t DEVADDR = ;
    void os_getArtEui (u1_t* buf) { }
    void os_getDevEui (u1_t* buf) { }
    void os_getDevKey (u1_t* buf) { }
#endif



// Pin Mappings

// Adafruit Feather M0
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {3, 6, LMIC_UNUSED_PIN},
};


// Pinmapping for Dragino Arduino shield
/*const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};
*/
// Pinmapping for LSN50 ??
/* 
const lmic_pinmap lmic_pins = {
        .nss =  PA15,         //38, PA15 pin
        .rxtx = LMIC_UNUSED_PIN,    
        .rst = PB0,        // 18,PB0
        .dio = {2,21,22},  //PC13,PB10,PB11
};
*/