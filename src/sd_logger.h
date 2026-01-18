#pragma once
#include <Arduino.h>
#include <SdFat.h>
#include "records.h"

class SdLogger {
public:
  SdLogger();

  bool begin();
  bool open_new_log(uint32_t prealloc_bytes);
  bool write_block(const BlockHdr& hdr, const uint8_t* payload);
  void poll_sync(uint32_t now_ms);
  void force_sync();

  uint32_t write_errs() const { return write_errs_; }

private:
  SdFs sd_;
  FsFile f_;
  uint32_t last_sync_ms_;
  uint32_t write_errs_;
};
