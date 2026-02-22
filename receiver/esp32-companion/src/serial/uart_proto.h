#pragma once

#include <Arduino.h>

namespace companion_proto {

static constexpr uint8_t SOF1 = 0xA5;
static constexpr uint8_t SOF2 = 0x5A;
static constexpr uint8_t VERSION = 0x01;
static constexpr size_t MAX_PAYLOAD = 128;

enum MsgType : uint8_t {
  MSG_TELEM_SNAPSHOT = 0x01,
  MSG_ALERT_EVENT = 0x02,
  MSG_HEARTBEAT = 0x03,
  MSG_CMD = 0x10,
  MSG_CMD_ACK = 0x11,
};

enum CmdType : uint8_t {
  CMD_SD_START = 0x01,
  CMD_SD_STOP = 0x02,
  CMD_BUZZER = 0x03,
  CMD_TELEM_ENABLE = 0x04,
  CMD_TELEM_DISABLE = 0x05,
};

#pragma pack(push, 1)
struct TelemetryV1 {
  uint32_t t_ms;
  int32_t lat_e7;
  int32_t lon_e7;
  int32_t alt_mm;
  int16_t vs_cms;
  int16_t rssi_dbm;
  int8_t snr_db_x4;
  uint8_t phase;
  uint16_t packet_count_lsb;
  uint16_t vbat_mv;
  uint8_t flags;
  uint16_t ground_vbat_mv;
};

struct AlertV1 {
  uint8_t code;
  uint8_t severity;
  uint32_t t_ms;
};

struct CmdV1 {
  uint8_t cmd;
  uint8_t arg;
};

struct CmdAckV1 {
  uint8_t cmd;
  uint8_t ok;
  uint8_t err;
};
#pragma pack(pop)

struct Frame {
  uint8_t version = VERSION;
  uint8_t type = 0;
  uint16_t seq = 0;
  uint16_t len = 0;
  uint8_t payload[MAX_PAYLOAD] = {0};
};

uint16_t crc16_ccitt(const uint8_t* data, size_t len);

class FrameParser {
 public:
  bool feed(uint8_t b, Frame& outFrame);

 private:
  enum State {
    WAIT_SOF1,
    WAIT_SOF2,
    READ_VER,
    READ_TYPE,
    READ_SEQ_LO,
    READ_SEQ_HI,
    READ_LEN_LO,
    READ_LEN_HI,
    READ_PAYLOAD,
    READ_CRC_LO,
    READ_CRC_HI,
  } state_ = WAIT_SOF1;

  Frame frame_{};
  uint16_t crcRx_ = 0;
  size_t payloadIdx_ = 0;
};

size_t encodeFrame(const Frame& frame, uint8_t* out, size_t outCap);

const char* phaseToText(uint8_t phase);

}  // namespace companion_proto
