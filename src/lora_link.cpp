#include "lora_link.h"
#include <RadioLib.h>
#include "cfg.h"

static Module loraMod(LORA_CS, LORA_DIO0, LORA_RST, LORA_BUSY);
static SX1276 radio(&loraMod);

bool LoraLink::begin() {
  int state = radio.begin(LORA_FREQ_MHZ);
  if (state != 0) {
    DBG_PRINTF("lora: begin err %d\n", state);
    return false;
  }
  DBG_PRINTLN("lora: begin ok");

  // Conservative baseline; tune later
  radio.setSpreadingFactor(10);
  radio.setBandwidth(125.0);
  radio.setCodingRate(5);
  radio.setOutputPower(14);
  radio.setCRC(true);
  return true;
}

bool LoraLink::send(const uint8_t* data, size_t n) {
  int state = radio.transmit((uint8_t*)data, n);
  if (state != 0) {
    DBG_PRINTF("lora: tx err %d\n", state);
  }
  return state == 0;
}
