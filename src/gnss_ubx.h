#pragma once
#include <Arduino.h>
#include "byte_ring.h"

struct GnssTime {
  uint16_t week = 0;
  uint32_t tow_ms = 0;
  bool fix_ok = false;
};

// Minimal UBX stream logger + optional NAV-PVT time extraction later.
class GnssUbx {
public:
  bool begin();
  void poll(ByteRing& ring, uint32_t now_us);
  const GnssTime& time() const { return time_; }

private:
  void send_ubx(const uint8_t* msg, uint16_t n);
  void configure();

  GnssTime time_;
  uint8_t chunk_[240];
  uint16_t chunk_n_ = 0;
};
