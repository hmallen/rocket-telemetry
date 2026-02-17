#pragma once

#include <Arduino.h>

// Teensy-side UART companion protocol helper.
// This is a drop-in skeleton: call init() in setup(), send_snapshot() at 10 Hz,
// and poll_cmd() in loop() to receive ESP32 command requests.

namespace companion_uart {

enum CmdType : uint8_t {
  CMD_SD_START = 0x01,
  CMD_SD_STOP = 0x02,
  CMD_BUZZER = 0x03,
};

struct Snapshot {
  uint32_t t_ms = 0;
  int32_t lat_e7 = 0;
  int32_t lon_e7 = 0;
  int32_t alt_mm = 0;
  int16_t vs_cms = 0;
  int16_t rssi_dbm = -120;
  int8_t snr_db_x4 = 0;
  uint8_t phase = 0;
  uint16_t packet_count_lsb = 0;
  uint16_t vbat_mv = 0;
  uint8_t flags = 0;
};

struct Command {
  bool available = false;
  uint8_t cmd = 0;
  uint8_t arg = 0;
};

void init(HardwareSerial& serial, uint32_t baud);
void send_snapshot(const Snapshot& s);
Command poll_cmd();
void send_cmd_ack(uint8_t cmd, bool ok, uint8_t err = 0);

}  // namespace companion_uart
