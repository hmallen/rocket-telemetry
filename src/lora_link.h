#pragma once
#include <stdint.h>
#include <Arduino.h>

class LoraLink {
public:
  bool begin();

  void enable_tx(bool enable);
  void shutdown();
  void set_faulted(bool faulted);

  bool ready() const;
  bool tx_enabled() const;

  // Cleartext telemetry payload (ASCII, human-decodable):
  //   ID=<CALLSIGN>;t_ms=<uint32>;
  //   press_pa_x10=<i32>;temp_c_x100=<i16>;
  //   ring_drops=<uint32>;spool_drops=<uint32>;sd_errs=<uint32>;
  //   freq_mhz=<float>;sf=<u8>;bw_khz=<float>;cr=<u8>;pwr_dbm=<i8>;id_int_ms=<uint32>\n
  // Callsign is always present as required station identification. No encryption or obfuscation.
  void poll_telem(uint32_t now_ms,
                  int32_t press_pa_x10,
                  int16_t temp_c_x100,
                  uint32_t ring_drops,
                  uint32_t spool_drops,
                  uint32_t sd_errs);

private:
  bool validate_config_() const;
  bool allow_tx_() const;
  void log_params_() const;
  void enter_silence_();
  void on_tx_done_();
  bool start_tx_(uint32_t now_ms,
                 int32_t press_pa_x10,
                 int16_t temp_c_x100,
                 uint32_t ring_drops,
                 uint32_t spool_drops,
                 uint32_t sd_errs);
  void schedule_retry_(uint32_t now_ms);

  bool began_ = false;
  bool config_ok_ = false;
  bool tx_enabled_ = false;
  bool tx_active_ = false;
  bool faulted_ = false;
  bool shutdown_ = false;

  uint32_t mission_start_ms_ = 0;
  uint32_t last_tx_ms_ = 0;
  uint32_t next_tx_ms_ = 0;
  uint32_t tx_start_ms_ = 0;
  uint32_t retry_after_ms_ = 0;
  uint8_t retries_left_ = 0;
  uint8_t consec_fail_ = 0;

  uint32_t last_ring_drops_ = 0;
  uint32_t last_spool_drops_ = 0;
  uint32_t last_sd_errs_ = 0;
  int32_t  last_press_pa_x10_ = 0;
  int16_t  last_temp_c_x100_ = 0;
  bool have_last_ = false;

  uint32_t pending_ring_drops_ = 0;
  uint32_t pending_spool_drops_ = 0;
  uint32_t pending_sd_errs_ = 0;
  int32_t  pending_press_pa_x10_ = 0;
  int16_t  pending_temp_c_x100_ = 0;
  bool pending_valid_ = false;

  char tx_buf_[240] = {0};
};
