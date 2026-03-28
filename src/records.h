#pragma once
#include <Arduino.h>

enum RecordType : uint8_t {
  REC_IMU_FAST     = 0x01, // accel+gyro (fixed)
  REC_BARO         = 0x02, // bmp390 (fixed)
  REC_GNSS_CHUNK   = 0x03, // raw bytes (variable)
  REC_TIME_ANCHOR  = 0x04, // PPS-to-GNSS anchor (fixed)
  REC_EVENT        = 0x05, // markers (fixed)
  REC_STATS        = 0x06, // drop counters, voltages, etc (fixed)
  REC_BARO2        = 0x07, // bmp180 (fixed)
};

constexpr uint16_t REC_EVENT_ID_LORA_CMD_RX_BASE = 0x0100;

static inline bool record_event_is_lora_cmd_rx(uint16_t event_id) {
  return event_id >= REC_EVENT_ID_LORA_CMD_RX_BASE &&
         event_id < (REC_EVENT_ID_LORA_CMD_RX_BASE + 0x0100u);
}

static inline uint8_t record_event_lora_cmd_rx_cmd(uint16_t event_id) {
  return (uint8_t)(event_id - REC_EVENT_ID_LORA_CMD_RX_BASE);
}

static inline uint16_t record_event_lora_cmd_rx_id(uint8_t cmd) {
  return (uint16_t)(REC_EVENT_ID_LORA_CMD_RX_BASE + cmd);
}

static inline const char* record_lora_cmd_name(uint8_t cmd) {
  switch (cmd) {
    case 0x01: return "sd_start";
    case 0x02: return "sd_stop";
    case 0x03: return "buzzer";
    case 0x04: return "telemetry_enable";
    case 0x05: return "telemetry_disable";
    case 0x06: return "alt_calibrate";
    case 0x07: return "imu_calibrate";
    case 0x08: return "set_tx_power";
    case 0x09: return "launch_arm";
    case 0x0A: return "sd_rotate";
    case 0x0B: return "sd_format";
    case 0x0C: return "sd_dump_sample";
    default: return nullptr;
  }
}

#pragma pack(push, 1)
struct RecHdr {
  uint8_t  type;
  uint8_t  ver;
  uint16_t len;     // bytes including header
};

struct RecImuFast {
  RecHdr   h;        // type=REC_IMU_FAST
  uint32_t t_us;
  int16_t  ax, ay, az;  // raw counts
  int16_t  gx, gy, gz;  // raw counts
  int16_t  temp;        // optional raw
  uint16_t status;
};

struct RecBaro {
  RecHdr   h;        // type=REC_BARO
  uint32_t t_us;
  int32_t  press_pa_x10;
  int16_t  temp_c_x100;
  uint16_t status;
};

struct RecBaro2 {
  RecHdr   h;        // type=REC_BARO2
  uint32_t t_us;
  int32_t  press_pa_x10;
  int16_t  temp_c_x100;
  uint16_t status;
};

constexpr uint16_t GNSS_CHUNK_MAX = 240;

struct RecGnssChunk {
  RecHdr    h;       // type=REC_GNSS_CHUNK
  uint32_t  t_us;
  uint16_t  n;       // payload bytes
  uint8_t   data[GNSS_CHUNK_MAX];
};

struct RecTimeAnchor {
  RecHdr    h;       // type=REC_TIME_ANCHOR
  uint32_t  t_us_at_pps;
  uint16_t  gps_week;     // 0 if unknown yet
  uint32_t  tow_ms;       // 0 if unknown yet
  uint8_t   fix_ok;       // 0/1
  uint8_t   reserved[3];
};

struct RecEvent {
  RecHdr    h;       // type=REC_EVENT
  uint32_t  t_us;
  uint16_t  event_id;
  int16_t   value;
};

struct RecStats {
  RecHdr    h;       // type=REC_STATS
  uint32_t  t_us;
  uint32_t  ring_drops;
  uint32_t  spool_drops;
  uint32_t  sd_write_errs;
  uint16_t  vbat_mv;
  uint8_t   bat_state;
  uint8_t   reserved;
};
#pragma pack(pop)

// Block framing (written to SD)
#pragma pack(push, 1)
struct BlockHdr {
  uint32_t magic;     // 'TLMB' = 0x424D4C54
  uint16_t ver;       // 1
  uint16_t hdr_len;   // sizeof(BlockHdr)
  uint32_t seq;
  uint32_t t_start_us;
  uint32_t payload_len;
  uint32_t crc32;     // of payload only
};
#pragma pack(pop)

constexpr uint32_t BLOCK_MAGIC = 0x424D4C54;
