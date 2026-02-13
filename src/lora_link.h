#pragma once
#include <stdint.h>
#include <Arduino.h>
 #include "gnss_ubx.h"
 #include "sensors.h"

enum class LoraCommand : uint8_t {
  kNone = 0,
  kSdStart = 0x01,
  kSdStop = 0x02,
  kBuzzer = 0x03,
};

class LoraLink {
public:
  bool begin();

  void enable_tx(bool enable);
  void shutdown();
  void set_faulted(bool faulted);

  bool ready() const;
  bool tx_enabled() const;
  bool pop_command(LoraCommand& cmd, uint8_t* arg = nullptr);
  void queue_command_ack(LoraCommand cmd, bool logging_enabled);

  // Cleartext telemetry payload (ASCII, human-decodable):
  //   ID=<CALLSIGN>;t_ms=<uint32>;
  //   press_pa_x10=<i32>;temp_c_x100=<i16>;
  //   ring_drops=<uint32>;spool_drops=<uint32>;sd_errs=<uint32>;
  //   freq_mhz=<float>;sf=<u8>;bw_khz=<float>;cr=<u8>;pwr_dbm=<i8>;id_int_ms=<uint32>\n
  // Callsign is always present as required station identification. No encryption or obfuscation.
  void poll_telem(uint32_t now_ms,
                  int32_t press_pa_x10,
                  int16_t temp_c_x100,
                  uint16_t vbat_mv,
                  uint8_t bat_state,
                  const GnssTime* gps,
                  const ImuSample* imu);

private:
  bool validate_config_() const;
  bool allow_tx_() const;
  void log_params_() const;
  void enter_silence_();
  void on_tx_done_();
  bool start_rx_();
  void poll_rx_(uint32_t now_ms);
  bool handle_command_(const uint8_t* data, size_t len);
  bool start_id_tx_(uint32_t now_ms);
  bool start_alt_tx_(uint32_t now_ms,
                     int32_t press_pa_x10,
                     int16_t temp_c_x100);
  bool start_gps_tx_(uint32_t now_ms,
                     const GnssTime& gps);
  bool start_navsat_tx_(uint32_t now_ms,
                        const GnssTime& gps);
  bool start_imu_tx_(uint32_t now_ms,
                     const ImuSample& imu);
  bool start_recovery_tx_(uint32_t now_ms);
  void schedule_retry_(uint32_t now_ms);
  void schedule_ack_retry_(uint32_t now_ms);
  void consume_pending_(uint32_t now_ms);

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
  bool tx_is_id_ = false;
  bool tx_is_ack_ = false;

  uint32_t last_id_tx_ms_ = 0;
  uint32_t id_retry_after_ms_ = 0;
  uint8_t id_retries_left_ = 0;

  uint32_t last_gps_tx_ms_ = 0;
  uint32_t last_alt_tx_ms_ = 0;
  uint32_t last_imu_tx_ms_ = 0;
  uint32_t last_bat_tx_ms_ = 0;
  uint32_t last_navsat_tx_ms_ = 0;
  uint32_t last_recovery_tx_ms_ = 0;

  int32_t  last_press_pa_x10_ = 0;
  int16_t  last_temp_c_x100_ = 0;
  bool have_last_alt_ = false;

  bool recovery_initialized_ = false;
  int32_t recovery_launch_alt_mm_ = 0;
  int32_t recovery_last_alt_mm_ = 0;
  uint32_t recovery_last_t_ms_ = 0;
  int32_t recovery_agl_mm_ = 0;
  int32_t recovery_max_agl_mm_ = 0;
  int16_t recovery_vspeed_cms_ = 0;
  uint8_t recovery_phase_ = 0;
  bool recovery_drogue_deployed_ = false;
  bool recovery_main_deployed_ = false;
  int32_t recovery_drogue_deploy_agl_mm_ = -1;
  int32_t recovery_main_deploy_agl_mm_ = -1;

  bool pending_valid_ = false;
  uint8_t pending_type_ = 0;
  size_t pending_len_ = 0;

  uint8_t tx_type_ = 0;
  size_t tx_len_ = 0;

  uint8_t tx_buf_[32] = {0};
  bool rx_active_ = false;
  uint8_t pending_cmd_ = 0;
  uint8_t pending_cmd_arg_ = 0;
  bool ack_pending_ = false;
  uint32_t ack_retry_after_ms_ = 0;
  uint8_t ack_retries_left_ = 0;
  size_t ack_len_ = 0;
  uint8_t ack_buf_[8] = {0};
};
