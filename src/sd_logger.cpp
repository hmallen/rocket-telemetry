#include "sd_logger.h"
#include "cfg.h"
#include <cstdio>
#include <cstring>

SdLogger::SdLogger() : last_sync_ms_(0), write_errs_(0) {
  log_name_[0] = '\0';
}

bool SdLogger::begin() {
  // SDIO on Teensy 4.1
  if (!sd_.begin(SdioConfig(FIFO_SDIO))) {
    DBG_PRINTLN("sd: begin failed");
    return false;
  }
  DBG_PRINTLN("sd: begin ok");
  return true;
}

bool SdLogger::open_new_log(uint32_t prealloc_bytes) {
  if (f_) {
    (void)f_.sync();
    (void)f_.close();
  }
  char name[32];
  // Simple monotonic naming: LOG00000.BIN ... LOG99999.BIN
  for (uint32_t i = 0; i < 100000; i++) {
    snprintf(name, sizeof(name), "LOG%05lu.BIN", (unsigned long)i);
    if (!sd_.exists(name)) {
      const uint8_t fat_type = sd_.fatType();
      if (fat_type == FAT_TYPE_FAT16 || fat_type == FAT_TYPE_FAT32) {
        FatFile contiguous;
        if (!contiguous.createContiguous(name, prealloc_bytes)) {
          DBG_PRINTLN("sd: createContiguous failed");
          return false;
        }
        (void)contiguous.close();
        f_ = sd_.open(name, O_RDWR);
      } else {
        f_ = sd_.open(name, O_CREAT | O_EXCL | O_RDWR);
        if (!f_) {
          DBG_PRINTLN("sd: open failed");
          return false;
        }
        if (!f_.preAllocate(prealloc_bytes)) {
          DBG_PRINTLN("sd: prealloc failed");
          (void)f_.close();
          (void)sd_.remove(name);
          return false;
        }
      }

      if (!f_) {
        DBG_PRINTLN("sd: open contiguous failed");
        (void)sd_.remove(name);
        return false;
      }

      if (!f_.seekSet(0)) {
        DBG_PRINTLN("sd: seek failed");
        (void)f_.close();
        (void)sd_.remove(name);
        return false;
      }
      last_sync_ms_ = millis();
      snprintf(log_name_, sizeof(log_name_), "%s", name);
      DBG_PRINTF("sd: log %s\n", name);
      return true;
    }
  }
  return false;
}

bool SdLogger::write_block(const BlockHdr& hdr, const uint8_t* payload) {
  if (!f_) return false;
  int32_t w1 = f_.write((const uint8_t*)&hdr, sizeof(hdr));
  if (w1 != (int32_t)sizeof(hdr)) {
    write_errs_++;
    DBG_PRINTLN("sd: write hdr failed");
    return false;
  }
  int32_t w2 = f_.write(payload, hdr.payload_len);
  if (w2 != (int32_t)hdr.payload_len) {
    write_errs_++;
    DBG_PRINTLN("sd: write payload failed");
    return false;
  }
  return true;
}

void SdLogger::poll_sync(uint32_t now_ms) {
  if (!f_) return;
  if ((uint32_t)(now_ms - last_sync_ms_) >= SD_SYNC_MS) {
    f_.sync();
    last_sync_ms_ = now_ms;
  }
}

void SdLogger::force_sync() {
  if (f_) f_.sync();
}

bool SdLogger::close_log() {
  bool ok = true;
  if (f_) {
    if (!f_.sync()) ok = false;
    if (!f_.close()) ok = false;
  }
  log_name_[0] = '\0';
  last_sync_ms_ = 0;
  return ok;
}

FsFile SdLogger::open_log_read() {
  if (!log_name_[0]) return FsFile();
  return sd_.open(log_name_, O_RDONLY);
}

bool SdLogger::clear_logs() {
  if (f_) {
    f_.sync();
    f_.close();
  }
  log_name_[0] = '\0';
  write_errs_ = 0;
  last_sync_ms_ = 0;

  FsFile dir = sd_.open("/", O_RDONLY);
  if (!dir || !dir.isDir()) {
    DBG_PRINTLN("sd: open root failed");
    return false;
  }
  dir.rewind();

  FsFile entry;
  bool ok = true;
  while (entry.openNext(&dir, O_RDONLY)) {
    if (entry.isDir()) {
      entry.close();
      continue;
    }
    char name[32];
    name[0] = '\0';
    entry.getName(name, sizeof(name));
    entry.close();

    const size_t len = strlen(name);
    if (len >= 8 && strncmp(name, "LOG", 3) == 0 && len >= 4 && strcmp(name + len - 4, ".BIN") == 0) {
      if (!sd_.remove(name)) {
        DBG_PRINTF("sd: remove failed %s\n", name);
        ok = false;
      }
    }
  }
  dir.close();
  return ok;
}

bool SdLogger::format_card() {
  if (f_) {
    (void)f_.sync();
    (void)f_.close();
  }
  log_name_[0] = '\0';
  write_errs_ = 0;
  last_sync_ms_ = 0;
  if (!sd_.format()) {
    DBG_PRINTLN("sd: format failed");
    return false;
  }
  DBG_PRINTLN("sd: format complete");
  return true;
}

bool SdLogger::parse_log_index(const char* name, uint32_t* out_index) {
  if (name == nullptr || out_index == nullptr) {
    return false;
  }
  const size_t len = strlen(name);
  if (len != 12 || strncmp(name, "LOG", 3) != 0 || strcmp(name + 8, ".BIN") != 0) {
    return false;
  }
  uint32_t value = 0;
  for (size_t i = 3; i < 8; ++i) {
    const char c = name[i];
    if (c < '0' || c > '9') {
      return false;
    }
    value = value * 10u + static_cast<uint32_t>(c - '0');
  }
  *out_index = value;
  return true;
}

bool SdLogger::find_latest_log_name(char* out, size_t out_len) {
  if (out == nullptr || out_len == 0) {
    return false;
  }
  out[0] = '\0';

  if (log_name_[0] != '\0') {
    snprintf(out, out_len, "%s", log_name_);
    return true;
  }

  FsFile dir = sd_.open("/", O_RDONLY);
  if (!dir || !dir.isDir()) {
    return false;
  }
  dir.rewind();

  uint32_t best_index = 0;
  bool have_best = false;
  FsFile entry;
  while (entry.openNext(&dir, O_RDONLY)) {
    if (entry.isDir()) {
      entry.close();
      continue;
    }
    char name[32];
    name[0] = '\0';
    entry.getName(name, sizeof(name));
    entry.close();

    uint32_t index = 0;
    if (!parse_log_index(name, &index)) {
      continue;
    }
    if (!have_best || index > best_index) {
      best_index = index;
      have_best = true;
      snprintf(out, out_len, "%s", name);
    }
  }
  dir.close();
  return have_best;
}

void SdLogger::append_text(char* out, size_t out_len, const char* text) {
  if (out == nullptr || out_len == 0 || text == nullptr || text[0] == '\0') {
    return;
  }
  const size_t used = strnlen(out, out_len);
  if (used >= (out_len - 1)) {
    return;
  }
  snprintf(out + used, out_len - used, "%s", text);
}

void SdLogger::append_record_summary(char* out,
                                     size_t out_len,
                                     uint8_t type,
                                     const uint8_t* payload,
                                     uint16_t payload_len) {
  if (out == nullptr || out_len == 0) {
    return;
  }
  char line[128];
  line[0] = '\0';

  if (type == REC_IMU_FAST && payload_len >= (sizeof(RecImuFast) - sizeof(RecHdr))) {
    RecImuFast r{};
    memcpy(&r.t_us, payload, sizeof(RecImuFast) - sizeof(RecHdr));
    snprintf(line,
             sizeof(line),
             "IMU t=%lu ax=%d ay=%d az=%d gx=%d gy=%d gz=%d",
             (unsigned long)r.t_us,
             (int)r.ax,
             (int)r.ay,
             (int)r.az,
             (int)r.gx,
             (int)r.gy,
             (int)r.gz);
  } else if ((type == REC_BARO || type == REC_BARO2) && payload_len >= (sizeof(RecBaro) - sizeof(RecHdr))) {
    RecBaro r{};
    memcpy(&r.t_us, payload, sizeof(RecBaro) - sizeof(RecHdr));
    snprintf(line,
             sizeof(line),
             "%s t=%lu p=%.1fPa temp=%.2fC",
             (type == REC_BARO2) ? "BARO2" : "BARO",
             (unsigned long)r.t_us,
             (double)r.press_pa_x10 / 10.0,
             (double)r.temp_c_x100 / 100.0);
  } else if (type == REC_GNSS_CHUNK && payload_len >= 6) {
    uint32_t t_us = 0;
    uint16_t n = 0;
    memcpy(&t_us, payload, sizeof(t_us));
    memcpy(&n, payload + sizeof(t_us), sizeof(n));
    snprintf(line, sizeof(line), "GNSS t=%lu bytes=%u", (unsigned long)t_us, (unsigned)n);
  } else if (type == REC_TIME_ANCHOR && payload_len >= (sizeof(RecTimeAnchor) - sizeof(RecHdr))) {
    RecTimeAnchor r{};
    memcpy(&r.t_us_at_pps, payload, sizeof(RecTimeAnchor) - sizeof(RecHdr));
    snprintf(line,
             sizeof(line),
             "TIME t_pps=%lu week=%u tow=%lu fix=%u",
             (unsigned long)r.t_us_at_pps,
             (unsigned)r.gps_week,
             (unsigned long)r.tow_ms,
             (unsigned)r.fix_ok);
  } else if (type == REC_EVENT && payload_len >= (sizeof(RecEvent) - sizeof(RecHdr))) {
    RecEvent r{};
    memcpy(&r.t_us, payload, sizeof(RecEvent) - sizeof(RecHdr));
    snprintf(line,
             sizeof(line),
             "EVENT t=%lu id=%u val=%d",
             (unsigned long)r.t_us,
             (unsigned)r.event_id,
             (int)r.value);
  } else if (type == REC_STATS && payload_len >= (sizeof(RecStats) - sizeof(RecHdr))) {
    RecStats r{};
    memcpy(&r.t_us, payload, sizeof(RecStats) - sizeof(RecHdr));
    snprintf(line,
             sizeof(line),
             "STATS t=%lu ring=%lu spool=%lu sd_err=%lu vb=%u",
             (unsigned long)r.t_us,
             (unsigned long)r.ring_drops,
             (unsigned long)r.spool_drops,
             (unsigned long)r.sd_write_errs,
             (unsigned)r.vbat_mv);
  } else {
    snprintf(line, sizeof(line), "REC type=0x%02X len=%u", (unsigned)type, (unsigned)payload_len);
  }

  append_text(out, out_len, line);
}

bool SdLogger::dump_latest_sample(char* out, size_t out_len) {
  if (out == nullptr || out_len == 0) {
    return false;
  }
  out[0] = '\0';

  char target_name[32];
  if (!find_latest_log_name(target_name, sizeof(target_name))) {
    append_text(out, out_len, "No log file found");
    return false;
  }

  force_sync();
  FsFile file = sd_.open(target_name, O_RDONLY);
  if (!file) {
    append_text(out, out_len, "Open log failed");
    return false;
  }

  BlockHdr last_hdr{};
  uint64_t last_payload_pos = 0;
  bool have_last = false;

  while (true) {
    BlockHdr hdr{};
    const int32_t r = file.read((uint8_t*)&hdr, sizeof(hdr));
    if (r == 0) {
      break;
    }
    if (r < (int32_t)sizeof(hdr) || hdr.hdr_len < sizeof(hdr)) {
      break;
    }
    if (hdr.hdr_len > sizeof(hdr)) {
      if (!file.seekCur(static_cast<int64_t>(hdr.hdr_len - sizeof(hdr)))) {
        break;
      }
    }
    if (hdr.magic != BLOCK_MAGIC) {
      break;
    }

    const uint64_t payload_pos = file.curPosition();
    uint32_t remaining = hdr.payload_len;
    uint32_t crc = 0;
    uint8_t scratch[96];
    bool payload_ok = true;
    while (remaining > 0) {
      const uint16_t chunk = (remaining > sizeof(scratch)) ? sizeof(scratch) : (uint16_t)remaining;
      const int32_t got = file.read(scratch, chunk);
      if (got != (int32_t)chunk) {
        payload_ok = false;
        break;
      }
      crc = crc32_update(crc, scratch, (size_t)chunk);
      remaining -= chunk;
    }
    if (!payload_ok || crc != hdr.crc32) {
      break;
    }

    have_last = true;
    last_hdr = hdr;
    last_payload_pos = payload_pos;
  }

  if (!have_last) {
    file.close();
    append_text(out, out_len, "No valid log blocks");
    return false;
  }

  if (!file.seekSet(last_payload_pos)) {
    file.close();
    append_text(out, out_len, "Seek latest block failed");
    return false;
  }

  static constexpr uint8_t kSummaryKeep = 3;
  char summaries[kSummaryKeep][128];
  for (uint8_t i = 0; i < kSummaryKeep; ++i) {
    summaries[i][0] = '\0';
  }
  uint8_t summary_count = 0;
  uint32_t rec_count = 0;

  uint32_t remaining = last_hdr.payload_len;
  while (remaining >= sizeof(RecHdr)) {
    uint8_t rec_hdr[sizeof(RecHdr)] = {0};
    if (file.read(rec_hdr, sizeof(rec_hdr)) != (int32_t)sizeof(rec_hdr)) {
      break;
    }
    remaining -= sizeof(rec_hdr);

    const uint8_t rec_type = rec_hdr[0];
    const uint16_t rec_len = (uint16_t)rec_hdr[2] | ((uint16_t)rec_hdr[3] << 8);
    if (rec_len < sizeof(RecHdr)) {
      break;
    }
    const uint16_t payload_len = rec_len - sizeof(RecHdr);
    if (payload_len > remaining) {
      break;
    }

    uint8_t payload_buf[256];
    const uint16_t read_len = (payload_len > sizeof(payload_buf)) ? sizeof(payload_buf) : payload_len;
    if (read_len > 0 && file.read(payload_buf, read_len) != (int32_t)read_len) {
      break;
    }
    if (payload_len > read_len) {
      if (!file.seekCur(static_cast<int64_t>(payload_len - read_len))) {
        break;
      }
    }
    remaining -= payload_len;
    rec_count++;

    char line[128];
    line[0] = '\0';
    append_record_summary(line, sizeof(line), rec_type, payload_buf, read_len);
    if (summary_count < kSummaryKeep) {
      snprintf(summaries[summary_count], sizeof(summaries[summary_count]), "%s", line);
      summary_count++;
    } else {
      for (uint8_t i = 1; i < kSummaryKeep; ++i) {
        snprintf(summaries[i - 1], sizeof(summaries[i - 1]), "%s", summaries[i]);
      }
      snprintf(summaries[kSummaryKeep - 1], sizeof(summaries[kSummaryKeep - 1]), "%s", line);
    }
  }

  file.close();

  snprintf(out,
           out_len,
           "Log %s seq=%lu records=%lu",
           target_name,
           (unsigned long)last_hdr.seq,
           (unsigned long)rec_count);
  for (uint8_t i = 0; i < summary_count; ++i) {
    append_text(out, out_len, "\n");
    append_text(out, out_len, summaries[i]);
  }

  return true;
}
