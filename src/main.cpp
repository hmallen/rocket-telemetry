#include <Arduino.h>

#include "cfg.h"
#include "records.h"
#include "byte_ring.h"
#include "psram_spool.h"
#include "block_builder.h"
#include "crc32.h"
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
static uint32_t block_seq = 0;
static bool sd_logging_enabled = false;

#if ENABLE_SD_DUMP
static const char* sd_dump_record_name(uint8_t type) {
  switch (type) {
    case REC_IMU_FAST: return "REC_IMU_FAST";
    case REC_BARO: return "REC_BARO";
    case REC_GNSS_CHUNK: return "REC_GNSS_CHUNK";
    case REC_TIME_ANCHOR: return "REC_TIME_ANCHOR";
    case REC_EVENT: return "REC_EVENT";
    case REC_STATS: return "REC_STATS";
    case REC_BARO2: return "REC_BARO2";
    default: return "REC_UNKNOWN";
  }
}

static void sd_dump_print_ascii(const uint8_t* data, uint16_t n, uint16_t preview) {
  Serial.print("    ascii: ");
  for (uint16_t i = 0; i < preview; ++i) {
    char c = (data[i] >= 32 && data[i] <= 126) ? (char)data[i] : '.';
    Serial.write(c);
  }
  if (n > preview) {
    Serial.print(" ...");
  }
  Serial.println();
}

static void sd_dump_print_hex(const uint8_t* data, uint16_t n, uint16_t preview) {
  Serial.print("    hex:   ");
  for (uint16_t i = 0; i < preview; ++i) {
    if (i && (i % 16 == 0)) {
      Serial.print("\n          ");
    }
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
  if (n > preview) {
    Serial.print("... (");
    Serial.print(n);
    Serial.println(" bytes)");
  } else {
    Serial.println();
  }
}

static void sd_dump_print_record(uint32_t idx,
                                 uint8_t type,
                                 uint8_t ver,
                                 uint16_t len,
                                 const uint8_t* payload,
                                 uint16_t payload_len) {
  const char* name = sd_dump_record_name(type);
  Serial.printf("  [%lu] %s ver=%u len=%u\n",
                (unsigned long)idx, name, (unsigned)ver, (unsigned)len);
  const uint16_t preview = min(payload_len, (uint16_t)64);

  if (type == REC_IMU_FAST && payload_len >= (sizeof(RecImuFast) - sizeof(RecHdr))) {
    RecImuFast r{};
    memcpy(&r.t_us, payload, sizeof(RecImuFast) - sizeof(RecHdr));
    Serial.printf("    t_us=%lu ax=%d ay=%d az=%d gx=%d gy=%d gz=%d temp=%d status=0x%04x\n",
                  (unsigned long)r.t_us, (int)r.ax, (int)r.ay, (int)r.az,
                  (int)r.gx, (int)r.gy, (int)r.gz, (int)r.temp, (unsigned)r.status);
    return;
  }
  if ((type == REC_BARO || type == REC_BARO2) && payload_len >= (sizeof(RecBaro) - sizeof(RecHdr))) {
    RecBaro r{};
    memcpy(&r.t_us, payload, sizeof(RecBaro) - sizeof(RecHdr));
    Serial.printf("    t_us=%lu press_pa_x10=%ld temp_c_x100=%d status=0x%04x\n",
                  (unsigned long)r.t_us, (long)r.press_pa_x10, (int)r.temp_c_x100,
                  (unsigned)r.status);
    return;
  }
  if (type == REC_GNSS_CHUNK && payload_len >= 6) {
    uint32_t t_us = 0;
    uint16_t n = 0;
    memcpy(&t_us, payload, sizeof(t_us));
    memcpy(&n, payload + sizeof(t_us), sizeof(n));
    uint16_t avail = payload_len > 6 ? (payload_len - 6) : 0;
    uint16_t n_use = min(n, avail);
    Serial.printf("    t_us=%lu n=%u\n", (unsigned long)t_us, (unsigned)n);
    if (n_use) {
      const uint8_t* data = payload + 6;
      sd_dump_print_ascii(data, n_use, min(n_use, preview));
      sd_dump_print_hex(data, n_use, min(n_use, preview));
    }
    return;
  }
  if (type == REC_TIME_ANCHOR && payload_len >= (sizeof(RecTimeAnchor) - sizeof(RecHdr))) {
    RecTimeAnchor r{};
    memcpy(&r.t_us_at_pps, payload, sizeof(RecTimeAnchor) - sizeof(RecHdr));
    Serial.printf("    t_us_at_pps=%lu gps_week=%u tow_ms=%lu fix_ok=%u\n",
                  (unsigned long)r.t_us_at_pps, (unsigned)r.gps_week,
                  (unsigned long)r.tow_ms, (unsigned)r.fix_ok);
    return;
  }
  if (type == REC_EVENT && payload_len >= (sizeof(RecEvent) - sizeof(RecHdr))) {
    RecEvent r{};
    memcpy(&r.t_us, payload, sizeof(RecEvent) - sizeof(RecHdr));
    Serial.printf("    t_us=%lu event_id=%u value=%d\n",
                  (unsigned long)r.t_us, (unsigned)r.event_id, (int)r.value);
    return;
  }
  if (type == REC_STATS && payload_len >= (sizeof(RecStats) - sizeof(RecHdr))) {
    RecStats r{};
    memcpy(&r.t_us, payload, sizeof(RecStats) - sizeof(RecHdr));
    Serial.printf("    t_us=%lu ring_drops=%lu spool_drops=%lu sd_write_errs=%lu vbat_mv=%u bat_state=%u\n",
                  (unsigned long)r.t_us, (unsigned long)r.ring_drops,
                  (unsigned long)r.spool_drops, (unsigned long)r.sd_write_errs,
                  (unsigned)r.vbat_mv, (unsigned)r.bat_state);
    return;
  }

  if (payload_len) {
    sd_dump_print_hex(payload, payload_len, preview);
  }
}

static bool sd_dump_skip_bytes(FsFile& file, uint32_t n, uint32_t* crc) {
  uint8_t scratch[64];
  while (n) {
    uint32_t chunk = min(n, (uint32_t)sizeof(scratch));
    int32_t r = file.read(scratch, chunk);
    if (r <= 0) return false;
    if (crc) {
      *crc = crc32_update(*crc, scratch, (size_t)r);
    }
    n -= (uint32_t)r;
  }
  return true;
}

static void sd_dump_log() {
  Serial.println("\nsd: dump requested");
#if !ENABLE_SD_LOGGER
  Serial.println("sd: logging disabled");
  return;
#endif
  sdlog.force_sync();
  const char* log_name = sdlog.log_name();
  if (!log_name) {
    Serial.println("sd: no log file name");
    return;
  }

  FsFile file = sdlog.open_log_read();
  if (!file) {
    Serial.println("sd: open log for read failed");
    return;
  }

  if (!file.seekSet(0)) {
    Serial.println("sd: seek failed");
    file.close();
    return;
  }

  Serial.print("sd: dumping ");
  Serial.println(log_name);

  uint32_t block_idx = 0;
  while (true) {
    BlockHdr hdr{};
    int32_t r = file.read((uint8_t*)&hdr, sizeof(hdr));
    if (r == 0) break;
    if (r < (int32_t)sizeof(hdr)) {
      Serial.println("sd: truncated block header");
      break;
    }
    if (hdr.hdr_len < sizeof(hdr)) {
      Serial.println("sd: invalid block header length");
      break;
    }
    if (hdr.hdr_len > sizeof(hdr)) {
      if (!sd_dump_skip_bytes(file, hdr.hdr_len - sizeof(hdr), nullptr)) {
        Serial.println("sd: truncated block header padding");
        break;
      }
    }

    if (hdr.magic != BLOCK_MAGIC) {
      if (hdr.magic == 0) {
        Serial.println("sd: reached preallocated empty region");
      } else {
        Serial.printf("sd: invalid block magic 0x%08lx\n", (unsigned long)hdr.magic);
      }
      break;
    }

    Serial.printf("BLOCK seq=%lu t_start_us=%lu payload_len=%lu\n",
                  (unsigned long)hdr.seq,
                  (unsigned long)hdr.t_start_us,
                  (unsigned long)hdr.payload_len);

    uint32_t remaining = hdr.payload_len;
    uint32_t crc = 0u;
    uint32_t rec_idx = 0;

    while (remaining >= sizeof(RecHdr)) {
      uint8_t rec_hdr[sizeof(RecHdr)];
      int32_t rh = file.read(rec_hdr, sizeof(rec_hdr));
      if (rh < (int32_t)sizeof(rec_hdr)) {
        Serial.println("sd: truncated record header");
        remaining = 0;
        break;
      }
      remaining -= sizeof(rec_hdr);
      crc = crc32_update(crc, rec_hdr, sizeof(rec_hdr));

      const uint8_t rec_type = rec_hdr[0];
      const uint8_t rec_ver = rec_hdr[1];
      const uint16_t rec_len = (uint16_t)rec_hdr[2] | ((uint16_t)rec_hdr[3] << 8);
      if (rec_len < sizeof(RecHdr) || rec_len - sizeof(RecHdr) > remaining) {
        Serial.println("sd: invalid record length");
        if (!sd_dump_skip_bytes(file, remaining, &crc)) {
          remaining = 0;
        }
        break;
      }

      const uint16_t payload_len = rec_len - sizeof(RecHdr);
      uint8_t rec_buf[GNSS_CHUNK_MAX + 16];
      uint16_t to_read = payload_len;
      if (to_read > sizeof(rec_buf)) {
        to_read = sizeof(rec_buf);
      }
      if (to_read) {
        int32_t rp = file.read(rec_buf, to_read);
        if (rp < (int32_t)to_read) {
          Serial.println("sd: truncated record payload");
          remaining = 0;
          break;
        }
        crc = crc32_update(crc, rec_buf, to_read);
      }

      if (payload_len > to_read) {
        uint32_t leftover = payload_len - to_read;
        if (!sd_dump_skip_bytes(file, leftover, &crc)) {
          Serial.println("sd: truncated record payload");
          remaining = 0;
          break;
        }
      }

      remaining -= payload_len;
      sd_dump_print_record(rec_idx, rec_type, rec_ver, rec_len, rec_buf, min(payload_len, to_read));
      rec_idx++;
    }

    if (remaining) {
      sd_dump_skip_bytes(file, remaining, &crc);
    }

    const bool crc_ok = (crc == hdr.crc32);
    Serial.printf("BLOCK CRC32 %s (0x%08lx)\n", crc_ok ? "OK" : "BAD", (unsigned long)crc);
    block_idx++;
  }

  file.close();
  Serial.println("sd: dump complete");
}

static void sd_clear_logs() {
  Serial.println("\nsd: clear requested");
#if !ENABLE_SD_LOGGER
  Serial.println("sd: logging disabled");
  return;
#endif
  const bool restart_logging = sd_logging_enabled;
  if (!sdlog.clear_logs()) {
    Serial.println("sd: clear failed");
    return;
  }

  block_seq = 0;
  block.reset(block_seq, micros());
  ring.init(ring_mem, RING_BYTES);
#if ENABLE_PSRAM_SPOOL
  spool.init(spool_mem, SPOOL_BYTES);
#endif
  sd_logging_enabled = false;

  if (restart_logging) {
    if (!sdlog.open_new_log(PREALLOC_BYTES)) {
      Serial.println("sd: open new log failed");
      return;
    }

    sd_logging_enabled = true;
    const char* log_name = sdlog.log_name();
    if (log_name) {
      Serial.print("sd: new log ");
      Serial.println(log_name);
    } else {
      Serial.println("sd: new log started");
    }
  }
}

static void sd_rotate_log() {
  Serial.println("\nsd: rotate requested");
#if !ENABLE_SD_LOGGER
  Serial.println("sd: logging disabled");
  return;
#endif
  char old_name[32];
  old_name[0] = '\0';
  const char* log_name = sdlog.log_name();
  if (log_name) {
    snprintf(old_name, sizeof(old_name), "%s", log_name);
  }

  if (!sdlog.close_log()) {
    Serial.println("sd: close failed");
  }
  if (!sdlog.open_new_log(PREALLOC_BYTES)) {
    Serial.println("sd: open new log failed");
    sd_logging_enabled = false;
    return;
  }

  sd_logging_enabled = true;
  if (old_name[0]) {
    Serial.print("sd: closed ");
    Serial.println(old_name);
  }
  log_name = sdlog.log_name();
  if (log_name) {
    Serial.print("sd: new log ");
    Serial.println(log_name);
  }
}

static void sd_dump_print_help() {
  Serial.println("sd dump commands:");
  Serial.println("  c: clear SD logs (keeps logging off unless already on)");
  Serial.println("  d: dump SD log to serial");
  Serial.println("  n: close current log + start new log");
  Serial.println("  h/? : help");
}
#endif

static void sd_logging_reset_buffers() {
  block_seq = 0;
  block.reset(block_seq, micros());
  ring.init(ring_mem, RING_BYTES);
#if ENABLE_PSRAM_SPOOL
  spool.init(spool_mem, SPOOL_BYTES);
#endif
}

static void sd_logging_stop() {
#if !ENABLE_SD_LOGGER
  return;
#endif
  if (!sd_logging_enabled) return;
  sdlog.force_sync();
  sdlog.close_log();
  sd_logging_enabled = false;
  sd_logging_reset_buffers();
  DBG_PRINTLN("sd: logging stopped");
}

static void sd_logging_start() {
#if !ENABLE_SD_LOGGER
  return;
#endif
  if (sd_logging_enabled) return;
  sd_logging_reset_buffers();
  if (sdlog.open_new_log(PREALLOC_BYTES)) {
    sd_logging_enabled = true;
    DBG_PRINTLN("sd: logging started");
  } else {
    sd_logging_enabled = false;
    DBG_PRINTLN("sd: logging start failed");
  }
}

#if ENABLE_GNSS
static GnssUbx gnss_primary(GNSS_SERIAL_PRIMARY);
static GnssUbx gnss_backup(GNSS_SERIAL_BACKUP);
static bool gnss_use_backup_as_primary = false;
#endif
static Sensors sensors;
static LoraLink lora;

enum BatState : uint8_t {
  BAT_OK = 0,
  BAT_WARN = 1,
  BAT_SHED = 2,
  BAT_CUTOFF = 3,
};

static uint16_t g_vbat_mv = 0;
static uint8_t g_bat_state = BAT_OK;

static inline void buzzer_set(bool on) {
 #if ENABLE_BUZZER
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
 #else
  (void)on;
 #endif
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
 #if !ENABLE_BUZZER
  (void)on_ms;
  (void)off_ms;
  (void)n;
  (void)now_ms;
  return;
 #endif
  buzzer_seq.active = (n != 0);
  buzzer_seq.is_on = false;
  buzzer_seq.remaining = n;
  buzzer_seq.on_ms = on_ms;
  buzzer_seq.off_ms = off_ms;
  buzzer_seq.next_ms = now_ms;
}

static void buzzer_poll(uint32_t now_ms) {
 #if !ENABLE_BUZZER
  (void)now_ms;
  return;
 #endif
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
 #if !ENABLE_BUZZER
  (void)on_ms;
  (void)off_ms;
  (void)n;
  return;
 #endif
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
  if (!sd_logging_enabled) return;
  RecStats r{};
  r.h.type = REC_STATS; r.h.ver = 2; r.h.len = sizeof(r);
  r.t_us = micros();
  r.ring_drops = ring.drops();
#if ENABLE_PSRAM_SPOOL
  r.spool_drops = spool.drops();
#else
  r.spool_drops = 0;
#endif
  r.sd_write_errs = sdlog.write_errs();
  r.vbat_mv = g_vbat_mv;
  r.bat_state = g_bat_state;
  ring.write(&r, sizeof(r));
}

static inline void ring_write_time_anchor(uint32_t t_us_at_pps, const GnssTime& gt) {
  if (!sd_logging_enabled) return;
  RecTimeAnchor r{};
  r.h.type = REC_TIME_ANCHOR; r.h.ver = 1; r.h.len = sizeof(r);
  r.t_us_at_pps = t_us_at_pps;
  r.gps_week = gt.week;
  r.tow_ms = gt.tow_ms;
  r.fix_ok = gt.fix_ok ? 1 : 0;
  ring.write(&r, sizeof(r));
}

static inline void ring_write_baro(uint32_t t_us, const BaroSample& s) {
  if (!sd_logging_enabled) return;
  RecBaro r{};
  r.h.type = REC_BARO; r.h.ver = 1; r.h.len = sizeof(r);
  r.t_us = t_us;
  r.press_pa_x10 = s.press_pa_x10;
  r.temp_c_x100 = s.temp_c_x100;
  r.status = s.status;
  ring.write(&r, sizeof(r));
}

static inline void ring_write_baro2(uint32_t t_us, const BaroSample& s) {
  if (!sd_logging_enabled) return;
  RecBaro2 r{};
  r.h.type = REC_BARO2; r.h.ver = 1; r.h.len = sizeof(r);
  r.t_us = t_us;
  r.press_pa_x10 = s.press_pa_x10;
  r.temp_c_x100 = s.temp_c_x100;
  r.status = s.status;
  ring.write(&r, sizeof(r));
}

static inline void ring_write_imu(uint32_t t_us, const ImuSample& s) {
  if (!sd_logging_enabled) return;
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
  if (!sd_logging_enabled) return false;
  bool ok = sdlog.write_block(hdr, block.payload_ptr());
  if (!ok) {
    // record SD error via stats records (done periodically)
  }
#endif
  block_seq++;
  block.reset(block_seq, micros());

#if ENABLE_SD_LOGGER
  if (sd_logging_enabled) {
    sdlog.poll_sync(now_ms);
  }
#endif
  return true;
}

static inline void pump_ring_to_spool() {
#if ENABLE_SD_LOGGER
  if (!sd_logging_enabled) return;
#endif
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

#if ENABLE_SD_DUMP && !DEBUG_MODE
  Serial.begin(115200);
  delay(10);
  uint32_t t0 = millis();
  while (!Serial && (uint32_t)(millis() - t0) < 2000) { }
#endif
#if ENABLE_SD_DUMP
  sd_dump_print_help();
#endif

  analogReadResolution(12);
  pinMode(VBAT_PIN, INPUT);

  if (SENSOR_RAIL_EN_PIN != 255) {
    pinMode(SENSOR_RAIL_EN_PIN, OUTPUT);
    digitalWrite(SENSOR_RAIL_EN_PIN, HIGH);
  }

 #if ENABLE_BUZZER
  pinMode(BUZZER_PIN, OUTPUT);
  buzzer_set(false);
  buzzer_ok();
 #endif

  ring.init(ring_mem, RING_BYTES);

#if ENABLE_PSRAM_SPOOL
  spool.init(spool_mem, SPOOL_BYTES);
#endif

  timebase_init(GNSS_PPS_PIN);

#if ENABLE_SD_LOGGER
  {
    const bool sd_ok = sdlog.begin();
    sd_logging_enabled = false;
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
  if (sd_logging_enabled) {
    ring_write_stats();
    pump_ring_to_spool();
    build_and_write_block_from_source(millis());
  }

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
  static ImuSample last_imu_sample{};
  static bool have_imu_sample = false;

  static float vbat_filt_v = 0.0f;
  static uint32_t vbat_last_sample_ms = 0;
  static uint32_t shed_below_start_ms = 0;
  static uint32_t cutoff_below_start_ms = 0;

#if DEBUG_MODE
  static uint32_t last_dbg_ms = 0;
  static ImuSample last_imu{};
  static uint32_t last_imu_dbg_us = 0;
#endif

  const uint32_t now_us = micros();
  const uint32_t now_ms = millis();

#if ENABLE_SD_DUMP
  while (Serial.available()) {
    const int c = Serial.read();
    if (c < 0) break;
    if (c == 'c' || c == 'C') {
      sd_clear_logs();
    } else if (c == 'd' || c == 'D') {
      sd_dump_log();
    } else if (c == 'n' || c == 'N') {
      sd_rotate_log();
    } else if (c == 'h' || c == '?') {
      sd_dump_print_help();
    }
  }
#endif

  if (vbat_last_sample_ms == 0 || (uint32_t)(now_ms - vbat_last_sample_ms) >= 50) {
    vbat_last_sample_ms = now_ms;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < VBAT_AVG_SAMPLES; ++i) {
      sum += (uint32_t)analogRead(VBAT_PIN);
    }
    const float counts = (float)sum / (float)VBAT_AVG_SAMPLES;
    const float vadc = counts * (3.3f / 4095.0f);
    const float vbat = vadc * VBAT_DIVIDER_SCALE * VBAT_CAL_SCALE;

    if (vbat_filt_v == 0.0f) vbat_filt_v = vbat;
    else vbat_filt_v = vbat_filt_v + VBAT_LP_ALPHA * (vbat - vbat_filt_v);

    const float vf = vbat_filt_v;

    if (g_bat_state == BAT_OK) {
      if (vf <= VBAT_WARN_V) g_bat_state = BAT_WARN;
    } else if (g_bat_state == BAT_WARN) {
      if (vf <= VBAT_SHED_V) {
        if (shed_below_start_ms == 0) shed_below_start_ms = now_ms;
        if ((uint32_t)(now_ms - shed_below_start_ms) >= VBAT_DWELL_MS) {
          g_bat_state = BAT_SHED;
          shed_below_start_ms = 0;
        }
      } else {
        shed_below_start_ms = 0;
      }
      if (vf >= (VBAT_WARN_V + VBAT_HYST_V)) g_bat_state = BAT_OK;
    } else if (g_bat_state == BAT_SHED) {
      if (vf <= VBAT_CUTOFF_V) {
        if (cutoff_below_start_ms == 0) cutoff_below_start_ms = now_ms;
        if ((uint32_t)(now_ms - cutoff_below_start_ms) >= VBAT_DWELL_MS) {
          g_bat_state = BAT_CUTOFF;
          cutoff_below_start_ms = 0;
        }
      } else {
        cutoff_below_start_ms = 0;
      }
      if (vf >= (VBAT_SHED_V + VBAT_HYST_V)) g_bat_state = BAT_WARN;
    } else {
      if (vf >= (VBAT_CUTOFF_V + VBAT_HYST_V)) g_bat_state = BAT_SHED;
    }

    if (SENSOR_RAIL_EN_PIN != 255) {
      const bool rail_on = (g_bat_state == BAT_OK) || (g_bat_state == BAT_WARN);
      digitalWrite(SENSOR_RAIL_EN_PIN, rail_on ? HIGH : LOW);
    }

    g_vbat_mv = (uint16_t)max(0, (int)lrintf(vf * 1000.0f));
  }

  buzzer_poll(now_ms);

#if DEBUG_MODE
  if ((uint32_t)(now_ms - last_dbg_ms) >= 1000) {
    last_dbg_ms = now_ms;
    DBG_PRINTF("t=%lu ms baro_pa=%.1f temp_c=%.2f vbat=%.3f bat_state=%u ring_avail=%lu ring_drops=%lu",
               (unsigned long)now_ms,
               (double)last_press_pa_x10 / 10.0,
               (double)last_temp_c_x100 / 100.0,
               (double)g_vbat_mv / 1000.0,
               (unsigned)g_bat_state,
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

  if (!sd_logging_enabled) {
    ring_out_primary = nullptr;
    ring_out_backup = nullptr;
  }

  gnss_primary.poll(ring_out_primary, now_us);
  gnss_backup.poll(ring_out_backup, now_us);
#endif

#if ENABLE_SENSORS
  const bool use_imu_ready_pins = (GRDY_PIN != 255) || (LRDY_PIN != 255);
  if (use_imu_ready_pins) {
    ImuSample s{};
    if (sensors.read_imu(s)) {
      ring_write_imu(now_us, s);
      last_imu_sample = s;
      have_imu_sample = true;
#if DEBUG_MODE
      last_imu = s;
      last_imu_dbg_us = now_us;
#endif
    }
  } else {
    // IMU target scheduler (framework stub read)
    const uint32_t imu_period_us = (IMU_HZ == 0) ? 0 : (1000000UL / IMU_HZ);
    if (imu_period_us) {
      if (last_imu_us == 0) last_imu_us = now_us;
      uint8_t imu_iters = 0;
      while ((uint32_t)(now_us - last_imu_us) >= imu_period_us) {
        ImuSample s{};
        if (sensors.read_imu(s)) {
          ring_write_imu(last_imu_us, s);
          last_imu_sample = s;
          have_imu_sample = true;
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

  const GnssTime* lora_gps = nullptr;
#if ENABLE_GNSS
  {
    const bool primary_fresh = gnss_primary.fresh(now_us, GNSS_FAILOVER_TIMEOUT_US);
    const bool backup_fresh  = gnss_backup.fresh(now_us, GNSS_FAILOVER_TIMEOUT_US);
    if (!gnss_use_backup_as_primary) {
      lora_gps = primary_fresh ? &gnss_primary.time() : (backup_fresh ? &gnss_backup.time() : nullptr);
    } else {
      lora_gps = backup_fresh ? &gnss_backup.time() : (primary_fresh ? &gnss_primary.time() : nullptr);
    }
  }
#endif

  // Periodic stats record
  if (sd_logging_enabled && (uint32_t)(now_ms - last_stats_ms) >= 500) {
    ring_write_stats();
    last_stats_ms = now_ms;
  }

  // Move ring -> spool
  pump_ring_to_spool();

  // Drain to SD in blocks
  if (sd_logging_enabled) {
    build_and_write_block_from_source(now_ms);
  }

#if ENABLE_LORA
  lora.poll_telem(now_ms,
                  last_press_pa_x10,
                  last_temp_c_x100,
                  g_vbat_mv,
                  g_bat_state,
                  lora_gps,
                  have_imu_sample ? &last_imu_sample : nullptr);

  {
    LoraCommand cmd = LoraCommand::kNone;
    uint8_t cmd_arg = 0;
    if (lora.pop_command(cmd, &cmd_arg)) {
      if (cmd == LoraCommand::kSdStart) {
        if (!buzzer_busy()) {
          buzzer_start_seq(60, 0, 1, now_ms);
        }
        sd_logging_start();
      } else if (cmd == LoraCommand::kSdStop) {
        if (!buzzer_busy()) {
          buzzer_start_seq(60, 0, 1, now_ms);
        }
        sd_logging_stop();
      } else if (cmd == LoraCommand::kBuzzer) {
        uint16_t duration_ms = 0;
        switch (cmd_arg) {
          case 1: duration_ms = 1000; break;
          case 5: duration_ms = 5000; break;
          case 10: duration_ms = 10000; break;
          case 30: duration_ms = 30000; break;
          default: duration_ms = 0; break;
        }
        if (duration_ms != 0) {
          buzzer_start_seq(duration_ms, 0, 1, now_ms);
        }
      }
      lora.enable_tx(sd_logging_enabled);
      lora.queue_command_ack(cmd, sd_logging_enabled);
    }
  }
#endif

  // Controlled SD sync
#if ENABLE_SD_LOGGER
  if (sd_logging_enabled) {
    sdlog.poll_sync(now_ms);
  }
#endif
}
