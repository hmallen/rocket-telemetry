#include "lora_link.h"
#include "cfg.h"

#include <RadioLib.h>

static Module lora_module(LORA_CS, LORA_DIO0, LORA_RST, LORA_BUSY);
static SX1276 radio(lora_module);

bool LoraLink::begin() {
  int state = radio.begin(LORA_FREQ_MHZ);
  if (state != RADIOLIB_ERR_NONE) return false;

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
  return state == RADIOLIB_ERR_NONE;
}
