



// RENAME TO ttn.h

// FILL IN YOUR TTN DREDENTIALS




#ifndef TTN_CONFIG_H
#define TTN_CONFIG_H

#include <arduino_lmic.h>

const static uint8_t PAYLOAD_SIZE = 20;

#ifdef OTAA
//Put in OTAA TTN Credentials
// lsb
static const u1_t PROGMEM DEVEUI[8]= { };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// lsb
static const u1_t PROGMEM APPEUI[8]= {  };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// msb
static const u1_t PROGMEM APPKEY[16] = {  };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

#else
// PUT ABP TTN Credentials Here

// 2. Insert your "Network Session Key" in hex  format, e.g., {0x12, 0x34 ...} in msb
static const PROGMEM u1_t NWKSKEY[16] = {  };
// 3. "App Session Key" in hex msb format, e.g., {0x12, 0x34 ...} in msb
static const u1_t PROGMEM APPSKEY[16] = {  };
// 4. "Device Address" in msb, but in hex format, e.g., 0x123456AB
static const u4_t DEVADDR = 0x123456AB;

// Leave empty, only used for OTAA
static const u1_t PROGMEM DEVEUI[8]= { };
static const u1_t PROGMEM APPEUI[8]= { };
static const u1_t PROGMEM APPKEY[16] = { };

#endif
#endif