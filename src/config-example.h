//Put in TTN Credentials
// lsb
static const u1_t PROGMEM DEVEUI[8]= { };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// lsb
static const u1_t PROGMEM APPEUI[8]= { };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// msb
static const u1_t PROGMEM APPKEY[16] = { };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
