#pragma once
#include <Arduino.h>

struct LoraStatusPacket {
  uint32_t t_ms;
  int32_t  baro_alt_cm;     // optional later
  int32_t  lat_e7;          // optional later
  int32_t  lon_e7;
  int32_t  alt_mm;
  uint16_t flags;
  uint16_t crc16;
};

class LoraLink {
public:
  bool begin();
  bool send(const uint8_t* data, size_t n);
};
