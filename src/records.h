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
