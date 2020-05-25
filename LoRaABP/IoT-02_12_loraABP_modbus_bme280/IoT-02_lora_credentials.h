#define LORA_MODE_ABP   0
#define LORA_MODE_OTAA  1

#define LORA_MODE LORA_MODE_ABP

#if LORA_MODE == LORA_MODE_ABP
  /* NWKSKEY, APPSKEY and DEVADDR should by modifed by your actual values */
  //static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  //static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  //static const u4_t DEVADDR = 0x00000000 ;

  static const PROGMEM u1_t NWKSKEY[16] = { 0x1F, 0x52, 0x8F, 0x57, 0x97, 0x1E, 0xED, 0xA0, 0xF0, 0x87, 0x8F, 0xB1, 0x59, 0xF3, 0x34, 0x7C };
  static const u1_t PROGMEM APPSKEY[16] = { 0xEF, 0x0F, 0x0C, 0xBF, 0x26, 0x61, 0x48, 0x9D, 0xC2, 0x79, 0x98, 0x08, 0x75, 0x00, 0xAB, 0x37 };
  static const u4_t DEVADDR = 0x26011A31 ;  // dispositiu-01-carlos
  
#endif

#if LORA_MODE == LORA_MODE_OTAA
    // De moment no és operatiu a aquesta versió
    const char *appEui = "";
    const char *appKey = "";
#endif