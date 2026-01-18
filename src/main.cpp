#include <Arduino.h>

#include "cfg.h"
#include "records.h"
#include "byte_ring.h"
#include "psram_spool.h"
#include "block_builder.h"
#include "sd_logger.h"
#include "timebase.h"
#include "gnss_ubx.h"
#include "sensors.h"
#include "lora_link.h"

// Internal fast ring
DMAMEM static uint8_t ring_mem[RING_BYTES];
static ByteRing ring;

// PSRAM spool
#if ENABLE_PSRAM_SPOOL
EXTMEM static uint8_t spool_mem[SPOOL_BYTES];
static PsramSpool spool;
#endif

static BlockBuilder block;
static SdLogger sdlog;

static GnssUbx gnss;
static Sensors sensors;
static LoraLink lora;

static uint32_t block_seq = 0;
static uint32_t sd_errs_last = 0;

static inline void ring_write_stats() {
  RecStats r{};
  r.h.type = REC_STATS; r.h.ver = 1; r.h.len = sizeof(r);
  r.t_us = micros();
  r.ring_drops = ring.drops();
#if ENABLE_PSRAM_SPOOL
  r.spool_drops = spool.drops();
#else
  r.spool_drops = 0;
#endif
  r.sd_write_errs = sdlog.write_errs();
  ring.write(&r, sizeof(r));
}

static inline void ring_write_time_anchor(uint32_t t_us_at_pps, const GnssTime& gt) {
  RecTimeAnchor r{};
  r.h.type = REC_TIME_ANCHOR; r.h.ver = 1; r.h.len = sizeof(r);
  r.t_us_at_pps = t_us_at_pps;
  r.gps_week = gt.week;
  r.tow_ms = gt.tow_ms;
  r.fix_ok = gt.fix_ok ? 1 : 0;
  ring.write(&r, sizeof(r));
}

static inline void ring_write_baro(uint32_t t_us, const BaroSample& s) {
  RecBaro r{};
  r.h.type = REC_BARO; r.h.ver = 1; r.h.len = sizeof(r);
  r.t_us = t_us;
  r.press_pa_x10 = s.press_pa_x10;
  r.temp_c_x100 = s.temp_c_x100;
  r.status = s.status;
  ring.write(&r, sizeof(r));
}

static inline void ring_write_imu(uint32_t t_us, const ImuSample& s) {
  RecImuFast r{};
  r.h.type = REC_IMU_FAST; r.h.ver = 1; r.h.len = sizeof(r);
  r.t_us = t_us;
  r.ax = s.ax; r.ay = s.ay; r.az = s.az;
  r.gx = s.gx; r.gy = s.gy; r.gz = s.gz;
  r.temp = s.temp;
  r.status = s.status;
  ring.write(&r, sizeof(r));
}

static inline bool build_and_write_block_from_source(uint32_t now_ms) {
  // Fill a block payload from spool (preferred) or ring (direct)
  BlockHdr hdr{};
  if (block.payload_len() == 0) block.reset(block_seq, micros());

  uint8_t tmp[1024];

#if ENABLE_PSRAM_SPOOL
  while (block.payload_free() >= sizeof(tmp) && spool.available() >= sizeof(tmp)) {
    uint32_t n = spool.read(tmp, sizeof(tmp));
    if (!block.append_bytes(tmp, n)) break;
  }
  // If spool has less than tmp, pull what's there
  if (spool.available() && block.payload_free()) {
    uint32_t nmax = min<uint32_t>(spool.available(), min<uint32_t>(block.payload_free(), sizeof(tmp)));
    uint32_t n = spool.read(tmp, nmax);
    block.append_bytes(tmp, n);
  }
#else
  while (block.payload_free() >= sizeof(tmp) && ring.available() >= sizeof(tmp)) {
    uint32_t n = ring.read(tmp, sizeof(tmp));
    if (!block.append_bytes(tmp, n)) break;
  }
  if (ring.available() && block.payload_free()) {
    uint32_t nmax = min<uint32_t>(ring.available(), min<uint32_t>(block.payload_free(), sizeof(tmp)));
    uint32_t n = ring.read(tmp, nmax);
    block.append_bytes(tmp, n);
  }
#endif

  // Write when block is reasonably full or when backlog exists and SD should drain
  const bool should_write =
    (block.payload_len() >= (LOG_BLOCK_BYTES - sizeof(BlockHdr) - 512)) ||
#if ENABLE_PSRAM_SPOOL
    (spool.available() > (SPOOL_BYTES / 2)) ||
#else
    (ring.available() > (RING_BYTES / 2)) ||
#endif
    false;

  if (!should_write) return false;

  uint32_t total = block.finalize(hdr);
  (void)total;
#if ENABLE_SD_LOGGER
  bool ok = sdlog.write_block(hdr, block.payload_ptr());
  if (!ok) {
    // record SD error via stats records (done periodically)
  }
#endif
  block_seq++;
  block.reset(block_seq, micros());

#if ENABLE_SD_LOGGER
  sdlog.poll_sync(now_ms);
#endif
  return true;
}

static inline void pump_ring_to_spool() {
#if ENABLE_PSRAM_SPOOL
  uint8_t tmp[1024];
  while (ring.available()) {
    uint32_t nmax = min<uint32_t>(ring.available(), sizeof(tmp));
    uint32_t n = ring.read(tmp, nmax);
    if (!spool.write(tmp, n)) {
      // If spool full, drop by not re-queueing. Stats will show it.
      break;
    }
  }
#endif
}

void setup() {
  Serial.begin(115200);

  ring.init(ring_mem, RING_BYTES);

#if ENABLE_PSRAM_SPOOL
  spool.init(spool_mem, SPOOL_BYTES);
#endif

  timebase_init(GNSS_PPS_PIN);

#if ENABLE_SD_LOGGER
  sdlog.begin();
  sdlog.open_new_log(PREALLOC_BYTES);
#endif

#if ENABLE_GNSS
  gnss.begin();
#endif

#if ENABLE_SENSORS
  sensors.begin();
#endif

  // Warm-up: force one small write path activity early (filesystem + card)
  ring_write_stats();
  pump_ring_to_spool();
  build_and_write_block_from_source(millis());

#if ENABLE_LORA
  lora.begin();
#endif
}

void loop() {
  static uint32_t last_imu_us  = 0;
  static uint32_t last_baro_us = 0;
  static uint32_t last_gnss_us = 0;
  static uint32_t last_lora_ms = 0;
  static uint32_t last_stats_ms = 0;

  const uint32_t now_us = micros();
  const uint32_t now_ms = millis();

  // PPS anchor
  uint32_t t_pps;
  if (time_pop_pps(t_pps)) {
#if ENABLE_GNSS
    ring_write_time_anchor(t_pps, gnss.time());
#else
    GnssTime dummy{};
    ring_write_time_anchor(t_pps, dummy);
#endif
  }

#if ENABLE_GNSS
  if (GNSS_SERIAL.available() > 0) {
    gnss.poll(ring, now_us);
  }
#endif

#if ENABLE_SENSORS
  // IMU target scheduler (framework stub read)
  if ((uint32_t)(now_us - last_imu_us) >= (1000000UL / IMU_HZ)) {
    ImuSample s{};
    if (sensors.read_imu(s)) ring_write_imu(now_us, s);
    last_imu_us += (1000000UL / IMU_HZ);
  }

  // Baro
  if ((uint32_t)(now_us - last_baro_us) >= (1000000UL / BARO_HZ)) {
    BaroSample b{};
    if (sensors.read_baro(b)) ring_write_baro(now_us, b);
    last_baro_us += (1000000UL / BARO_HZ);
  }
#endif

  // Periodic stats record
  if ((uint32_t)(now_ms - last_stats_ms) >= 500) {
    ring_write_stats();
    last_stats_ms = now_ms;
  }

  // Move ring -> spool
  pump_ring_to_spool();

  // Drain to SD in blocks
  build_and_write_block_from_source(now_ms);

#if ENABLE_LORA
  // Low-rate downlink stub
  if ((uint32_t)(now_ms - last_lora_ms) >= (1000UL / LORA_HZ)) {
    struct __attribute__((packed)) TinyPkt {
      uint32_t t_ms;
      uint32_t ring_drops;
      uint32_t spool_drops;
      uint32_t sd_errs;
    } pkt;
    pkt.t_ms = now_ms;
    pkt.ring_drops = ring.drops();
#if ENABLE_PSRAM_SPOOL
    pkt.spool_drops = spool.drops();
#else
    pkt.spool_drops = 0;
#endif
    pkt.sd_errs = sdlog.write_errs();
    lora.send((const uint8_t*)&pkt, sizeof(pkt));
    last_lora_ms = now_ms;
  }
#endif

  // Controlled SD sync
#if ENABLE_SD_LOGGER
  sdlog.poll_sync(now_ms);
#endif
}
