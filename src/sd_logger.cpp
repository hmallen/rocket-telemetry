#include "sd_logger.h"
#include "cfg.h"

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
  char name[32];
  // Simple monotonic naming: LOG00000.BIN ... LOG99999.BIN
  for (uint32_t i = 0; i < 100000; i++) {
    snprintf(name, sizeof(name), "LOG%05lu.BIN", (unsigned long)i);
    if (!sd_.exists(name)) {
      f_ = sd_.open(name, O_CREAT | O_EXCL | O_RDWR);
      if (!f_) {
        DBG_PRINTLN("sd: open failed");
        return false;
      }
      if (!f_.preAllocate(prealloc_bytes)) {
        DBG_PRINTLN("sd: prealloc failed");
        return false;
      }
      if (!f_.seekSet(0)) {
        DBG_PRINTLN("sd: seek failed");
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

FsFile SdLogger::open_log_read() const {
  if (!log_name_[0]) return FsFile();
  return sd_.open(log_name_, O_RDONLY);
}
