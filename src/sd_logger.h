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
  bool close_log();
  bool clear_logs();
  bool format_card();
  bool dump_latest_sample(char* out, size_t out_len);

  const char* log_name() const { return log_name_[0] ? log_name_ : nullptr; }
  FsFile open_log_read();

  uint32_t write_errs() const { return write_errs_; }

private:
  bool find_latest_log_name(char* out, size_t out_len);
  static bool parse_log_index(const char* name, uint32_t* out_index);
  static void append_text(char* out, size_t out_len, const char* text);
  static void append_record_summary(char* out,
                                    size_t out_len,
                                    uint8_t type,
                                    const uint8_t* payload,
                                    uint16_t payload_len);

  SdFs sd_;
  FsFile f_;
  uint32_t last_sync_ms_;
  uint32_t write_errs_;
  char log_name_[32];
};
