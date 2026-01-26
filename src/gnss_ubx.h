#pragma once
#include <stdint.h>
#include <Arduino.h>
#include "byte_ring.h"

class HardwareSerial;

struct GnssTime {
  uint16_t week = 0;
  uint32_t tow_ms = 0;
  bool fix_ok = false;
  uint8_t fix_type = 0;
  int32_t lon_e7 = 0;
  int32_t lat_e7 = 0;
  int32_t height_mm = 0;
  uint32_t last_pvt_ms = 0;
};

// Minimal UBX stream logger + optional NAV-PVT time extraction later.
class GnssUbx {
public:
  explicit GnssUbx(HardwareSerial& serial) : serial_(serial) {}
  bool begin();
  void poll(ByteRing* ring, uint32_t now_us);
  const GnssTime& time() const { return time_; }
  uint32_t last_rx_us() const { return last_rx_us_; }
  uint32_t bytes_rx() const { return bytes_rx_; }
  bool fresh(uint32_t now_us, uint32_t timeout_us) const {
    return last_rx_us_ != 0 && (uint32_t)(now_us - last_rx_us_) <= timeout_us;
  }

private:
  void send_ubx(const uint8_t* msg, uint16_t n);
  void configure();
  void parse_byte(uint8_t b, uint32_t now_us);

  HardwareSerial& serial_;
  GnssTime time_;
  uint32_t last_rx_us_ = 0;
  uint32_t bytes_rx_ = 0;
  uint8_t sync_ = 0;
  uint8_t cls_ = 0;
  uint8_t id_ = 0;
  uint16_t len_ = 0;
  uint16_t pay_i_ = 0;
  uint8_t ck_a_ = 0;
  uint8_t ck_b_ = 0;
  uint8_t ck_a_recv_ = 0;
  uint8_t ck_b_recv_ = 0;
  uint8_t payload_[96];
  uint8_t chunk_[240];
  uint16_t chunk_n_ = 0;
};
