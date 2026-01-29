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

#if ENABLE_GNSS
static GnssUbx gnss_primary(GNSS_SERIAL_PRIMARY);
static GnssUbx gnss_backup(GNSS_SERIAL_BACKUP);
static bool gnss_use_backup_as_primary = false;
#endif
static Sensors sensors;
static LoraLink lora;

static uint32_t block_seq = 0;

static inline void buzzer_set(bool on) {
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
}

struct BuzzerSeq {
  bool active = false;
  bool is_on = false;
  uint8_t remaining = 0;
  uint16_t on_ms = 0;
  uint16_t off_ms = 0;
  uint32_t next_ms = 0;
};

static BuzzerSeq buzzer_seq;

static inline bool buzzer_busy() {
  return buzzer_seq.active;
}

static void buzzer_start_seq(uint16_t on_ms, uint16_t off_ms, uint8_t n, uint32_t now_ms) {
  buzzer_seq.active = (n != 0);
  buzzer_seq.is_on = false;
  buzzer_seq.remaining = n;
  buzzer_seq.on_ms = on_ms;
  buzzer_seq.off_ms = off_ms;
  buzzer_seq.next_ms = now_ms;
}

static void buzzer_poll(uint32_t now_ms) {
  if (!buzzer_seq.active) return;
  if ((int32_t)(now_ms - buzzer_seq.next_ms) < 0) return;

  if (!buzzer_seq.is_on) {
    buzzer_set(true);
    buzzer_seq.is_on = true;
    buzzer_seq.next_ms = now_ms + buzzer_seq.on_ms;
    return;
  }

  buzzer_set(false);
  buzzer_seq.is_on = false;
  if (buzzer_seq.remaining) buzzer_seq.remaining--;
  if (buzzer_seq.remaining == 0) {
    buzzer_seq.active = false;
    return;
  }
  buzzer_seq.next_ms = now_ms + buzzer_seq.off_ms;
}

static void buzzer_pulse(uint16_t on_ms, uint16_t off_ms, uint8_t n) {
  for (uint8_t i = 0; i < n; ++i) {
    buzzer_set(true);
    delay(on_ms);
    buzzer_set(false);
    if (i + 1 < n) delay(off_ms);
  }
}

static inline void buzzer_ok() {
  buzzer_pulse(200, 100, 1);
}

static inline void buzzer_fail() {
  buzzer_pulse(250, 150, 3);
}

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

static inline void ring_write_baro2(uint32_t t_us, const BaroSample& s) {
  RecBaro2 r{};
  r.h.type = REC_BARO2; r.h.ver = 1; r.h.len = sizeof(r);
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
  DBG_INIT();
  DBG_PRINTLN("boot");

  pinMode(BUZZER_PIN, OUTPUT);
  buzzer_set(false);
  buzzer_ok();

  ring.init(ring_mem, RING_BYTES);

#if ENABLE_PSRAM_SPOOL
  spool.init(spool_mem, SPOOL_BYTES);
#endif

  timebase_init(GNSS_PPS_PIN);

#if ENABLE_SD_LOGGER
  {
    const bool sd_ok = sdlog.begin() && sdlog.open_new_log(PREALLOC_BYTES);
    if (sd_ok) {
      buzzer_ok();
    } else {
      buzzer_fail();
    }
  }
#endif

#if ENABLE_GNSS
  {
    const bool g1_ok = gnss_primary.begin();
    const bool g2_ok = gnss_backup.begin();
    if (g1_ok && g2_ok) {
      buzzer_ok();
    } else {
      buzzer_fail();
    }
  }
#endif

#if ENABLE_SENSORS
  if (!sensors.begin()) {
    DBG_PRINTLN("sensors.begin failed");
    buzzer_fail();
  } else {
    DBG_PRINTLN("sensors.begin ok");
    buzzer_ok();
  }
#endif

  // Warm-up: force one small write path activity early (filesystem + card)
  ring_write_stats();
  pump_ring_to_spool();
  build_and_write_block_from_source(millis());

#if ENABLE_LORA
  {
    const bool lora_ok = lora.begin();
    if (lora_ok) {
      buzzer_ok();
    } else {
      buzzer_fail();
    }
  }
#endif
}

void loop() {
  static uint32_t last_imu_us  = 0;
  static uint32_t last_baro_us = 0;
  static uint32_t last_baro2_us = 0;
  static int32_t last_press_pa_x10 = 0;
  static int16_t last_temp_c_x100 = 0;
  static uint32_t last_stats_ms = 0;
  static bool primary_3d_beeped = false;

#if DEBUG_MODE
  static uint32_t last_dbg_ms = 0;
  static ImuSample last_imu{};
  static uint32_t last_imu_dbg_us = 0;
#endif

  const uint32_t now_us = micros();
  const uint32_t now_ms = millis();

  buzzer_poll(now_ms);

#if DEBUG_MODE
  if ((uint32_t)(now_ms - last_dbg_ms) >= 1000) {
    last_dbg_ms = now_ms;
    DBG_PRINTF("t=%lu ms baro_pa=%.1f temp_c=%.2f ring_avail=%lu ring_drops=%lu",
               (unsigned long)now_ms,
               (double)last_press_pa_x10 / 10.0,
               (double)last_temp_c_x100 / 100.0,
               (unsigned long)ring.available(), (unsigned long)ring.drops());
    if (last_imu_dbg_us != 0) {
      DBG_PRINTF(" imu_ax=%d ay=%d az=%d gx=%d gy=%d gz=%d ist=0x%04x",
                 (int)last_imu.ax, (int)last_imu.ay, (int)last_imu.az,
                 (int)last_imu.gx, (int)last_imu.gy, (int)last_imu.gz,
                 (unsigned)last_imu.status);
    } else {
      DBG_PRINT(" imu=none");
    }
#if ENABLE_PSRAM_SPOOL
    DBG_PRINTF(" spool_avail=%lu spool_drops=%lu", (unsigned long)spool.available(),
               (unsigned long)spool.drops());
#endif
#if ENABLE_GNSS
    const bool primary_fresh_dbg = gnss_primary.fresh(now_us, GNSS_FAILOVER_TIMEOUT_US);
    const bool backup_fresh_dbg  = gnss_backup.fresh(now_us, GNSS_FAILOVER_TIMEOUT_US);
    const GnssTime& pt = gnss_primary.time();
    const GnssTime& bt = gnss_backup.time();

    DBG_PRINTF(" gnss_sel=%c", gnss_use_backup_as_primary ? 'B' : 'P');
    DBG_PRINTF(" gnss_p_fresh=%u fix_ok=%u fix=%u lat=%.7f lon=%.7f alt_m=%.3f tow_ms=%lu week=%u bytes=%lu",
               (unsigned)primary_fresh_dbg,
               (unsigned)pt.fix_ok, (unsigned)pt.fix_type,
               (double)pt.lat_e7 / 1e7,
               (double)pt.lon_e7 / 1e7,
               (double)pt.height_mm / 1000.0,
               (unsigned long)pt.tow_ms, (unsigned)pt.week,
               (unsigned long)gnss_primary.bytes_rx());

    DBG_PRINTF(" gnss_b_fresh=%u fix_ok=%u fix=%u lat=%.7f lon=%.7f alt_m=%.3f tow_ms=%lu week=%u bytes=%lu",
               (unsigned)backup_fresh_dbg,
               (unsigned)bt.fix_ok, (unsigned)bt.fix_type,
               (double)bt.lat_e7 / 1e7,
               (double)bt.lon_e7 / 1e7,
               (double)bt.height_mm / 1000.0,
               (unsigned long)bt.tow_ms, (unsigned)bt.week,
               (unsigned long)gnss_backup.bytes_rx());

    const GnssTime& gt = gnss_use_backup_as_primary ? bt : pt;
    if (gt.last_sat_ms != 0) {
      DBG_PRINTF(" sat=%u/%u", (unsigned)gt.navsat_n, (unsigned)gt.navsat_num_svs);
      const uint8_t show = (gt.navsat_n < 4) ? gt.navsat_n : 4;
      for (uint8_t i = 0; i < show; ++i) {
        const auto& s = gt.navsat[i];
        DBG_PRINTF(" [%u:%u cno=%u el=%d]", (unsigned)s.gnss_id, (unsigned)s.sv_id,
                   (unsigned)s.cno, (int)s.elev_deg);
      }
    } else {
      DBG_PRINT(" sat=none");
    }
#endif
    DBG_PRINTF(" sd_errs=%lu\n", (unsigned long)sdlog.write_errs());
  }
#endif

  // PPS anchor
  uint32_t t_pps;
  if (time_pop_pps(t_pps)) {
#if ENABLE_GNSS
    const GnssUbx& anchor = gnss_primary.fresh(now_us, GNSS_FAILOVER_TIMEOUT_US)
                              ? gnss_primary
                              : gnss_backup;
    ring_write_time_anchor(t_pps, anchor.time());
#else
    GnssTime dummy{};
    ring_write_time_anchor(t_pps, dummy);
#endif
  }

#if ENABLE_GNSS
  if (!primary_3d_beeped) {
    const GnssTime& pt = gnss_primary.time();
    const bool primary_fresh = gnss_primary.fresh(now_us, GNSS_FAILOVER_TIMEOUT_US);
    const bool primary_3d = primary_fresh && pt.fix_ok && pt.fix_type >= 3;
    if (primary_3d && !buzzer_busy()) {
      buzzer_start_seq(150, 150, 2, now_ms);
      primary_3d_beeped = true;
    }
  }

  const bool primary_fresh = gnss_primary.fresh(now_us, GNSS_FAILOVER_TIMEOUT_US);
  const bool backup_fresh  = gnss_backup.fresh(now_us, GNSS_FAILOVER_TIMEOUT_US);

  if (!gnss_use_backup_as_primary) {
    if (!primary_fresh && backup_fresh) gnss_use_backup_as_primary = true;
  } else {
    if (primary_fresh) gnss_use_backup_as_primary = false;
  }

  ByteRing* ring_out_primary = gnss_use_backup_as_primary ? nullptr : &ring;
  ByteRing* ring_out_backup  = gnss_use_backup_as_primary ? &ring : nullptr;

  if (GNSS_SERIAL_PRIMARY.available() > 0) {
    gnss_primary.poll(ring_out_primary, now_us);
  }
  if (GNSS_SERIAL_BACKUP.available() > 0) {
    gnss_backup.poll(ring_out_backup, now_us);
  }
#endif

#if ENABLE_SENSORS
  // IMU target scheduler (framework stub read)
  const uint32_t imu_period_us = (IMU_HZ == 0) ? 0 : (1000000UL / IMU_HZ);
  if (imu_period_us) {
    if (last_imu_us == 0) last_imu_us = now_us;
    uint8_t imu_iters = 0;
    while ((uint32_t)(now_us - last_imu_us) >= imu_period_us) {
      ImuSample s{};
      if (sensors.read_imu(s)) {
        ring_write_imu(last_imu_us, s);
#if DEBUG_MODE
        last_imu = s;
        last_imu_dbg_us = last_imu_us;
#endif
      }
      last_imu_us += imu_period_us;
      if (++imu_iters >= 4) {
        last_imu_us = now_us;
        break;
      }
    }
  }

  // Baro
  const uint32_t baro_period_us = (BARO_HZ == 0) ? 0 : (1000000UL / BARO_HZ);
  if (baro_period_us) {
    if (last_baro_us == 0) last_baro_us = now_us;
    uint8_t baro_iters = 0;
    while ((uint32_t)(now_us - last_baro_us) >= baro_period_us) {
      BaroSample b{};
      if (sensors.read_baro(b)) {
        ring_write_baro(last_baro_us, b);
        last_press_pa_x10 = b.press_pa_x10;
        last_temp_c_x100 = b.temp_c_x100;
      }
      last_baro_us += baro_period_us;
      if (++baro_iters >= 2) {
        last_baro_us = now_us;
        break;
      }
    }
  }

  // Baro2
  const uint32_t baro2_period_us = (BARO2_HZ == 0) ? 0 : (1000000UL / BARO2_HZ);
  if (baro2_period_us) {
    if (last_baro2_us == 0) last_baro2_us = now_us;
    uint8_t baro2_iters = 0;
    while ((uint32_t)(now_us - last_baro2_us) >= baro2_period_us) {
      BaroSample b2{};
      if (sensors.read_baro2(b2)) {
        ring_write_baro2(last_baro2_us, b2);
      }
      last_baro2_us += baro2_period_us;
      if (++baro2_iters >= 1) {
        last_baro2_us = now_us;
        break;
      }
    }
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
  // Part 97 telemetry downlink (cleartext callsign embedded by LoraLink).
  uint32_t spool_drops = 0;
#if ENABLE_PSRAM_SPOOL
  spool_drops = spool.drops();
#endif
  lora.poll_telem(now_ms,
                  last_press_pa_x10,
                  last_temp_c_x100,
                  ring.drops(),
                  spool_drops,
                  sdlog.write_errs());
#endif

  // Controlled SD sync
#if ENABLE_SD_LOGGER
  sdlog.poll_sync(now_ms);
#endif
}
