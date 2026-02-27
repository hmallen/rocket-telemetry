#include "lora_link.h"
#include <RadioLib.h>
#include "cfg.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static Module loraMod(LORA_CS, LORA_DIO0, LORA_RST, LORA_BUSY);
static SX1276 radio(&loraMod);

static volatile bool lora_tx_done_isr = false;
static void lora_on_packet_sent_isr() {
  lora_tx_done_isr = true;
}

static constexpr uint8_t RECOVERY_PHASE_IDLE = 0;
static constexpr uint8_t RECOVERY_PHASE_ASCENT = 1;
static constexpr uint8_t RECOVERY_PHASE_DESCENT = 2;
static constexpr uint8_t RECOVERY_PHASE_LANDED = 3;

bool LoraLink::set_tx_power_dbm(uint8_t power_dbm) {
  if (power_dbm < 2 || power_dbm > 17) {
    return false;
  }

  const uint8_t prev_power = tx_power_dbm_;
  tx_power_dbm_ = power_dbm;

  if (!began_ || !config_ok_) {
    return true;
  }

  const int state = radio.setOutputPower(tx_power_dbm_);
  if (state != 0) {
    tx_power_dbm_ = prev_power;
    (void)radio.setOutputPower(tx_power_dbm_);
    return false;
  }

  DBG_PRINTF("lora: tx power set to %udBm\n", (unsigned)tx_power_dbm_);
  return true;
}

bool LoraLink::arm_launch_detect_mode(bool allow_without_gps_fix) {
  if (!recovery_initialized_) {
    return false;
  }
  if (!allow_without_gps_fix && !recovery_gps_fix_3d_) {
    return false;
  }
  if (recovery_phase_ != RECOVERY_PHASE_IDLE) {
    return false;
  }
  if (recovery_liftoff_detected_) {
    return false;
  }
  recovery_launch_armed_ = true;
  return true;
}

static bool nearly_equal(float a, float b, float eps = 0.15f) {
  return fabsf(a - b) <= eps;
}

static bool lora_bw_valid(float bw_khz) {
  return
    nearly_equal(bw_khz, 7.8f) ||
    nearly_equal(bw_khz, 10.4f) ||
    nearly_equal(bw_khz, 15.6f) ||
    nearly_equal(bw_khz, 20.8f) ||
    nearly_equal(bw_khz, 31.25f) ||
    nearly_equal(bw_khz, 41.7f) ||
    nearly_equal(bw_khz, 62.5f) ||
    nearly_equal(bw_khz, 125.0f) ||
    nearly_equal(bw_khz, 250.0f) ||
    nearly_equal(bw_khz, 500.0f);
}

static int32_t abs_i32(int32_t x) {
  return (x < 0) ? -x : x;
}

static void write_u16_le(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

 static void write_i16_le(uint8_t* p, int16_t v) {
   write_u16_le(p, (uint16_t)v);
 }

static void write_u32_le(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static constexpr uint8_t LORA_CMD_MAGIC = 0xB1;
static constexpr uint8_t LORA_CMD_ACK = 0x10;
static constexpr uint8_t LORA_CMD_SD_START = 0x01;
static constexpr uint8_t LORA_CMD_SD_STOP = 0x02;
static constexpr uint8_t LORA_CMD_BUZZER = 0x03;
static constexpr uint8_t LORA_CMD_TELEM_ENABLE = 0x04;
static constexpr uint8_t LORA_CMD_TELEM_DISABLE = 0x05;
static constexpr uint8_t LORA_CMD_ALT_CALIBRATE = 0x06;
static constexpr uint8_t LORA_CMD_IMU_CALIBRATE = 0x07;
static constexpr uint8_t LORA_CMD_SET_TX_POWER = 0x08;
static constexpr uint8_t LORA_CMD_LAUNCH_ARM = 0x09;
static constexpr uint8_t LORA_CMD_SD_ROTATE = 0x0A;
static constexpr uint8_t LORA_CMD_SD_FORMAT = 0x0B;
static constexpr uint8_t LORA_CMD_SD_DUMP_SAMPLE = 0x0C;
static constexpr uint16_t LORA_RX_DONE_FLAG = 0x40;
static constexpr uint8_t LORA_ACK_REPEAT_COUNT = 3;
static constexpr uint32_t LORA_ACK_REPEAT_MS = 400;

static constexpr uint8_t RECOVERY_REASON_NONE = 0;
static constexpr uint8_t RECOVERY_DROGUE_REASON_APOGEE_VOTE = 1;
static constexpr uint8_t RECOVERY_MAIN_REASON_PRIMARY_ALTITUDE = 1;
static constexpr uint8_t RECOVERY_MAIN_REASON_BACKUP_ALTITUDE = 2;
static constexpr uint8_t RECOVERY_MAIN_REASON_FAST_DESCENT = 3;
static constexpr uint8_t RECOVERY_MAIN_REASON_BACKUP_NO_DROGUE = 4;

bool LoraLink::begin() {
  began_ = true;
  mission_start_ms_ = millis();
  request_recovery_calibration();

  config_ok_ = validate_config_();

  int state = radio.begin(LORA_FREQ_MHZ);
  if (state != 0) {
    DBG_PRINTF("lora: begin err %d\n", state);
    config_ok_ = false;
    tx_enabled_ = false;
    return false;
  }

  radio.setPacketSentAction(lora_on_packet_sent_isr);

  (void)radio.setSpreadingFactor(LORA_SF);
  (void)radio.setBandwidth(LORA_BW_KHZ);
  (void)radio.setCodingRate(LORA_CR);
  (void)radio.setOutputPower(tx_power_dbm_);
  (void)radio.setCRC(true);
  // Match receiver scripts (Pi/Pico) which configure LoRa public sync word (0x34)
  // and preamble length 8.
  (void)radio.setSyncWord(0x34);
  (void)radio.setPreambleLength(8);

  enter_silence_();
  log_params_();

#if LORA_TX_ENABLE_AT_BOOT
  enable_tx(true);
#else
  enable_tx(false);
#endif

  DBG_PRINTLN("lora: begin ok");
  return config_ok_;
}

void LoraLink::enable_tx(bool enable) {
  if (!began_) return;

  if (enable) {
    if (!allow_tx_()) {
      tx_enabled_ = false;
      return;
    }
    tx_enabled_ = true;
    next_tx_ms_ = millis();
  } else {
    tx_enabled_ = false;
    enter_silence_();
  }
}

void LoraLink::shutdown() {
  shutdown_ = true;
  tx_enabled_ = false;
  enter_silence_();
}

void LoraLink::set_faulted(bool faulted) {
  faulted_ = faulted;
  if (faulted_) {
    tx_enabled_ = false;
    enter_silence_();
  }
}

bool LoraLink::ready() const {
  return began_ && config_ok_;
}

bool LoraLink::tx_enabled() const {
  return tx_enabled_;
}

void LoraLink::request_recovery_calibration() {
  reset_recovery_state_();

  recovery_calibration_pending_ = true;
  recovery_baro_cal_start_ms_ = 0;
  recovery_baro_cal_done_ = false;
  recovery_baro_cal_sum_pa_x10_ = 0;
  recovery_baro_cal_count_ = 0;
  recovery_baro_zero_pa_x10_ = 0;

  recovery_gps_cal_done_ = false;
  recovery_gps_cal_sum_mm_ = 0;
  recovery_gps_cal_count_ = 0;
  recovery_gps_zero_mm_ = 0;

  recovery_baro_agl_mm_ = 0;
  recovery_gps_agl_mm_ = 0;
}

void LoraLink::reset_recovery_state_() {
  recovery_initialized_ = false;
  recovery_launch_alt_mm_ = 0;
  recovery_last_alt_mm_ = 0;
  recovery_last_t_ms_ = 0;
  recovery_agl_mm_ = 0;
  recovery_max_agl_mm_ = 0;
  recovery_vspeed_cms_ = 0;
  recovery_phase_ = RECOVERY_PHASE_IDLE;
  recovery_launch_armed_ = false;
  recovery_gps_fix_3d_ = false;
  recovery_gps_fix_3d_latched_ = false;
  recovery_liftoff_detected_ = false;
  recovery_apogee_detected_ = false;
  recovery_have_min_press_ = false;
  recovery_min_press_pa_x10_ = 0;
  recovery_drogue_deployed_ = false;
  recovery_main_deployed_ = false;
  recovery_landing_detected_ = false;
  recovery_drogue_reason_ = RECOVERY_REASON_NONE;
  recovery_main_reason_ = RECOVERY_REASON_NONE;
  recovery_drogue_deploy_agl_mm_ = -1;
  recovery_main_deploy_agl_mm_ = -1;
}

int32_t LoraLink::agl_from_press_mm_(int32_t press_pa_x10, int32_t ref_press_pa_x10) {
  if (press_pa_x10 <= 0 || ref_press_pa_x10 <= 0) {
    return 0;
  }

  const float press_pa = static_cast<float>(press_pa_x10) / 10.0f;
  const float ref_pa = static_cast<float>(ref_press_pa_x10) / 10.0f;
  if (press_pa <= 0.0f || ref_pa <= 0.0f) {
    return 0;
  }

  const float ratio = press_pa / ref_pa;
  const float altitude_m = 44330.0f * (1.0f - powf(ratio, 0.19029495f));
  if (!isfinite(altitude_m) || altitude_m <= 0.0f) {
    return 0;
  }
  return static_cast<int32_t>(altitude_m * 1000.0f);
}

bool LoraLink::pop_command(LoraCommand& cmd, uint8_t* arg) {
  if (pending_cmd_ == 0) return false;
  cmd = static_cast<LoraCommand>(pending_cmd_);
  if (arg != nullptr) {
    *arg = pending_cmd_arg_;
  }
  pending_cmd_ = 0;
  pending_cmd_arg_ = 0;
  return true;
}

bool LoraLink::start_recovery_tx_(uint32_t now_ms) {
  if (!tx_enabled_) return false;
  if (!recovery_initialized_) return false;

  // PROTO A1 type 6 (recovery), 28 bytes total:
  // [0]    u8   0xA1                protocol marker
  // [1]    u8   6                   packet type (recovery)
  // [2:6]  u32  t_ms                telemetry timestamp (ms)
  // [6]    u8   phase               0=idle,1=ascent,2=descent,3=landed
  // [7]    u8   event_flags         bit0=sensors calibrated, bit1=gps 3d fix,
  //                                  bit2=armed, bit3=launch detected,
  //                                  bit4=apogee, bit5=drogue deployed,
  //                                  bit6=main deployed, bit7=landing detected
  // [8:12] i32  agl_mm              current altitude AGL (mm)
  // [12:16]i32  max_agl_mm          max altitude AGL reached (mm)
  // [16:18]i16  vspeed_cms          vertical speed (cm/s)
  // [18:22]i32  drogue_deploy_agl_mm deployment AGL for drogue, -1 if not deployed
  // [22:26]i32  main_deploy_agl_mm   deployment AGL for main, -1 if not deployed
  // [26]   u8   drogue_reason        0=none,1=apogee_vote
  // [27]   u8   main_reason          0=none,1=primary_alt,2=backup_alt,3=fast_descent,4=backup_no_drogue

  tx_buf_[0] = 0xA1;
  tx_buf_[1] = 6;
  write_u32_le(&tx_buf_[2], (uint32_t)now_ms);
  tx_buf_[6] = recovery_phase_;
  const bool sensors_calibrated = !recovery_calibration_pending_;
  tx_buf_[7] = (sensors_calibrated ? 0x01u : 0x00u) |
               (recovery_gps_fix_3d_latched_ ? 0x02u : 0x00u) |
               (recovery_launch_armed_ ? 0x04u : 0x00u) |
               (recovery_liftoff_detected_ ? 0x08u : 0x00u) |
               (recovery_apogee_detected_ ? 0x10u : 0x00u) |
               (recovery_drogue_deployed_ ? 0x20u : 0x00u) |
               (recovery_main_deployed_ ? 0x40u : 0x00u) |
               (recovery_landing_detected_ ? 0x80u : 0x00u);
  write_u32_le(&tx_buf_[8], (uint32_t)recovery_agl_mm_);
  write_u32_le(&tx_buf_[12], (uint32_t)recovery_max_agl_mm_);
  write_i16_le(&tx_buf_[16], recovery_vspeed_cms_);
  write_u32_le(&tx_buf_[18], (uint32_t)recovery_drogue_deploy_agl_mm_);
  write_u32_le(&tx_buf_[22], (uint32_t)recovery_main_deploy_agl_mm_);
  tx_buf_[26] = recovery_drogue_reason_;
  tx_buf_[27] = recovery_main_reason_;
  const size_t n = 28;

  pending_valid_ = true;
  pending_type_ = 6;
  pending_len_ = n;

  lora_tx_done_isr = false;
  tx_start_ms_ = now_ms;
  tx_active_ = true;
  rx_active_ = false;
  tx_is_id_ = false;
  tx_is_ack_ = false;
  tx_type_ = 6;
  tx_len_ = n;

  int state = radio.startTransmit(tx_buf_, n);
  if (state != 0) {
    tx_active_ = false;
    DBG_PRINTF("lora: startTransmit err %d\n", state);
    return false;
  }

  return true;
}

void LoraLink::queue_command_ack(LoraCommand cmd, bool enabled_state, const char* detail) {
  uint8_t cmd_byte = 0;
  if (cmd == LoraCommand::kSdStart) {
    cmd_byte = LORA_CMD_SD_START;
  } else if (cmd == LoraCommand::kSdStop) {
    cmd_byte = LORA_CMD_SD_STOP;
  } else if (cmd == LoraCommand::kTelemEnable) {
    cmd_byte = LORA_CMD_TELEM_ENABLE;
  } else if (cmd == LoraCommand::kTelemDisable) {
    cmd_byte = LORA_CMD_TELEM_DISABLE;
  } else if (cmd == LoraCommand::kAltCalibrate) {
    cmd_byte = LORA_CMD_ALT_CALIBRATE;
  } else if (cmd == LoraCommand::kImuCalibrate) {
    cmd_byte = LORA_CMD_IMU_CALIBRATE;
  } else if (cmd == LoraCommand::kSetTxPower) {
    cmd_byte = LORA_CMD_SET_TX_POWER;
  } else if (cmd == LoraCommand::kLaunchArm) {
    cmd_byte = LORA_CMD_LAUNCH_ARM;
  } else if (cmd == LoraCommand::kSdRotate) {
    cmd_byte = LORA_CMD_SD_ROTATE;
  } else if (cmd == LoraCommand::kSdFormat) {
    cmd_byte = LORA_CMD_SD_FORMAT;
  } else if (cmd == LoraCommand::kSdDumpSample) {
    cmd_byte = LORA_CMD_SD_DUMP_SAMPLE;
  } else {
    return;
  }

  ack_buf_[0] = LORA_CMD_MAGIC;
  ack_buf_[1] = LORA_CMD_ACK;
  ack_buf_[2] = cmd_byte;
  ack_buf_[3] = enabled_state ? 1u : 0u;
  if (cmd == LoraCommand::kSetTxPower) {
    ack_buf_[4] = tx_power_dbm_;
    ack_len_ = 5;
  } else {
    ack_len_ = 4;
  }

  if (detail != nullptr && detail[0] != '\0' && ack_len_ < sizeof(ack_buf_)) {
    size_t detail_len = strlen(detail);
    const size_t detail_cap = sizeof(ack_buf_) - ack_len_;
    if (detail_len > detail_cap) {
      detail_len = detail_cap;
    }
    if (detail_len > 0) {
      memcpy(&ack_buf_[ack_len_], detail, detail_len);
      ack_len_ += detail_len;
    }
  }

  ack_pending_ = true;
  ack_retry_after_ms_ = millis() + LORA_ACK_REPEAT_MS;
  ack_retries_left_ = LORA_ACK_REPEAT_COUNT;
}

void LoraLink::poll_telem(uint32_t now_ms,
                          int32_t press_pa_x10,
                          int16_t temp_c_x100,
                          uint16_t vbat_mv,
                          uint8_t bat_state,
                          const GnssTime* gps,
                          const ImuSample* imu) {
  if (!began_) return;

  poll_rx_(now_ms);

  // Hard failsafe: maximum mission transmit duration.
  if ((uint32_t)(now_ms - mission_start_ms_) >= LORA_MAX_MISSION_TX_MS) {
#if LORA_HEARTBEAT_ENABLE
    if (tx_enabled_) {
      DBG_PRINTLN("lora: mission tx timeout -> heartbeat-only");
      tx_enabled_ = false;
      enter_silence_();
    }
#else
    shutdown();
    return;
#endif
  }

  if (!allow_tx_()) {
    enter_silence_();
    return;
  }

  if (tx_active_) {
    if (lora_tx_done_isr) {
      lora_tx_done_isr = false;
      (void)radio.finishTransmit();
#if DEBUG_MODE
      DBG_PRINTF("lora: tx done dt_ms=%lu\n", (unsigned long)(now_ms - tx_start_ms_));
#endif
      on_tx_done_();
    } else if ((uint32_t)(now_ms - tx_start_ms_) >= LORA_TX_WATCHDOG_MS) {
      (void)radio.finishTransmit();
      if (tx_is_ack_) {
        DBG_PRINTLN("lora: ack tx watchdog -> retry");
        tx_active_ = false;
        tx_is_ack_ = false;
        schedule_ack_retry_(now_ms);
      } else if (tx_is_id_) {
        DBG_PRINTLN("lora: id tx watchdog -> retry");
        tx_active_ = false;
        tx_is_id_ = false;
        if (id_retries_left_ == 0) id_retries_left_ = LORA_RETRY_LIMIT;
        if (id_retries_left_ != 0) {
          const uint8_t attempt = (uint8_t)(LORA_RETRY_LIMIT - id_retries_left_ + 1);
          const uint32_t backoff = LORA_RETRY_BASE_MS * (uint32_t)attempt;
          id_retries_left_--;
          id_retry_after_ms_ = now_ms + backoff;
        }
      } else {
#if LORA_HEARTBEAT_ENABLE
        DBG_PRINTLN("lora: tx watchdog -> heartbeat-only");
        tx_active_ = false;
        tx_enabled_ = false;
        enter_silence_();
#else
        DBG_PRINTLN("lora: tx watchdog -> shutdown");
        shutdown();
#endif
      }
    }
    return;
  }

  const bool ack_due = ack_pending_ &&
    (ack_retry_after_ms_ == 0 || (int32_t)(now_ms - ack_retry_after_ms_) >= 0);
  if (ack_due) {
    if ((uint32_t)(now_ms - last_tx_ms_) < LORA_MIN_TX_INTERVAL_MS) return;
    if (next_tx_ms_ != 0 && (int32_t)(now_ms - next_tx_ms_) < 0) return;

    lora_tx_done_isr = false;
    tx_start_ms_ = now_ms;
    tx_active_ = true;
    rx_active_ = false;
    tx_is_id_ = false;
    tx_is_ack_ = true;
    tx_type_ = 0;
    tx_len_ = ack_len_;

    const int state = radio.startTransmit(ack_buf_, ack_len_);
    if (state != 0) {
      tx_active_ = false;
      tx_is_ack_ = false;
      DBG_PRINTF("lora: ack startTransmit err %d\n", state);
      schedule_ack_retry_(now_ms);
    }
    return;
  }

  // Automatic transmitter shutdown after landing: detect stable pressure after flight.
  // Flight heuristic: a large pressure delta vs initial baseline indicates ascent/descent.
  static bool flight_detected = false;
  static bool landing_window_open = false;
  static int32_t flight_baseline_press = 0;
  static int32_t landing_ref_press = 0;
  static uint32_t landing_stable_start_ms = 0;

  if (!flight_baseline_press && press_pa_x10 != 0) {
    flight_baseline_press = press_pa_x10;
  }

  if (!flight_detected && flight_baseline_press) {
    if (abs_i32(press_pa_x10 - flight_baseline_press) >= LORA_FLIGHT_PRESS_DELTA_PA_X10) {
      flight_detected = true;
      landing_window_open = true;
      landing_stable_start_ms = 0;
      landing_ref_press = press_pa_x10;
    }
  }

  if (flight_detected && landing_window_open) {
    if (landing_stable_start_ms == 0) {
      landing_ref_press = press_pa_x10;
      landing_stable_start_ms = now_ms;
    }

    if (abs_i32(press_pa_x10 - landing_ref_press) > LORA_LANDING_PRESS_STABLE_DELTA_PA_X10) {
      landing_ref_press = press_pa_x10;
      landing_stable_start_ms = now_ms;
    }

    if ((uint32_t)(now_ms - landing_stable_start_ms) >= LORA_LANDING_STABLE_MS) {
      DBG_PRINTLN("lora: landing detected");
#if LORA_HEARTBEAT_ENABLE
      if (tx_enabled_) {
        DBG_PRINTLN("lora: landing -> heartbeat-only");
        tx_enabled_ = false;
        enter_silence_();
      }
#else
      shutdown();
      return;
#endif
    }
  }

  if ((uint32_t)(now_ms - last_tx_ms_) < LORA_MIN_TX_INTERVAL_MS) return;
  if (next_tx_ms_ != 0 && (int32_t)(now_ms - next_tx_ms_) < 0) return;

  #if LORA_HEARTBEAT_ENABLE
  const bool id_due = (LORA_HEARTBEAT_MS != 0) &&
    ((last_id_tx_ms_ == 0) || ((uint32_t)(now_ms - last_id_tx_ms_) >= LORA_HEARTBEAT_MS));
  const bool id_retry_ready = (id_retry_after_ms_ != 0) && ((int32_t)(now_ms - id_retry_after_ms_) >= 0);
  const bool want_id = id_due || id_retry_ready;

  if (want_id && !(id_retry_after_ms_ != 0 && (int32_t)(now_ms - id_retry_after_ms_) < 0)) {
    if (!start_id_tx_(now_ms, vbat_mv)) {
      if (id_retries_left_ == 0) id_retries_left_ = LORA_RETRY_LIMIT;
      if (id_retries_left_ != 0) {
        const uint8_t attempt = (uint8_t)(LORA_RETRY_LIMIT - id_retries_left_ + 1);
        const uint32_t backoff = LORA_RETRY_BASE_MS * (uint32_t)attempt;
        id_retries_left_--;
        id_retry_after_ms_ = now_ms + backoff;
      }
    }
    return;
  }
  #endif

  if (!tx_enabled_) return;

  if (pending_valid_) {
    if (retry_after_ms_ != 0 && (int32_t)(now_ms - retry_after_ms_) < 0) return;

    lora_tx_done_isr = false;
    tx_start_ms_ = now_ms;
    tx_active_ = true;
    rx_active_ = false;
    tx_is_id_ = false;
    tx_is_ack_ = false;
    tx_type_ = pending_type_;
    tx_len_ = pending_len_;

    const int state = radio.startTransmit(tx_buf_, pending_len_);
    if (state != 0) {
      tx_active_ = false;
      DBG_PRINTF("lora: startTransmit err %d\n", state);
      schedule_retry_(now_ms);
    }
    return;
  }

  const uint32_t gps_int_ms = LORA_GPS_INTERVAL_MS;
  const uint32_t alt_int_ms = LORA_ALT_INTERVAL_MS;
  const uint32_t imu_int_ms = LORA_IMU_INTERVAL_MS;
  const uint32_t bat_int_ms = LORA_BAT_INTERVAL_MS;
  const uint32_t navsat_int_ms = LORA_NAVSAT_INTERVAL_MS;
  const uint32_t recovery_int_ms = LORA_RECOVERY_INTERVAL_MS;

  const bool gps_valid = (gps != nullptr) && (gps->last_pvt_ms != 0) && gps->fix_ok && (gps->fix_type >= 3);
  recovery_gps_fix_3d_ = gps_valid;
  if (gps_valid) {
    recovery_gps_fix_3d_latched_ = true;
  }

  if (recovery_calibration_pending_) {
    if (!recovery_baro_cal_done_) {
      if (press_pa_x10 > 0) {
        if (recovery_baro_cal_start_ms_ == 0) {
          recovery_baro_cal_start_ms_ = now_ms;
        }
        if ((uint32_t)(now_ms - recovery_baro_cal_start_ms_) >= RECOVERY_BARO_CAL_DELAY_MS) {
          recovery_baro_cal_sum_pa_x10_ += press_pa_x10;
          recovery_baro_cal_count_++;
          if (recovery_baro_cal_count_ >= RECOVERY_BARO_CAL_SAMPLES) {
            recovery_baro_zero_pa_x10_ =
                static_cast<int32_t>(recovery_baro_cal_sum_pa_x10_ / (int64_t)recovery_baro_cal_count_);
            recovery_baro_cal_done_ = (recovery_baro_zero_pa_x10_ > 0);
          }
        }
      }
    }

    if (!recovery_gps_cal_done_ && gps_valid) {
      recovery_gps_cal_sum_mm_ += gps->height_mm;
      recovery_gps_cal_count_++;
      if (recovery_gps_cal_count_ >= RECOVERY_GPS_CAL_SAMPLES) {
        recovery_gps_zero_mm_ = static_cast<int32_t>(recovery_gps_cal_sum_mm_ / (int64_t)recovery_gps_cal_count_);
        recovery_gps_cal_done_ = true;
      }
    }

    if (recovery_baro_cal_done_ && recovery_gps_cal_done_) {
      recovery_calibration_pending_ = false;
    }
  }

  if (recovery_baro_cal_done_ && press_pa_x10 > 0) {
    recovery_baro_agl_mm_ = agl_from_press_mm_(press_pa_x10, recovery_baro_zero_pa_x10_);
  }

  if (recovery_gps_cal_done_ && gps_valid) {
    const int32_t gps_agl_raw = gps->height_mm - recovery_gps_zero_mm_;
    recovery_gps_agl_mm_ = (gps_agl_raw > 0) ? gps_agl_raw : 0;
  }

  bool have_altitude = false;
  int32_t selected_agl_mm = 0;
  if (recovery_baro_cal_done_ && press_pa_x10 > 0) {
    selected_agl_mm = recovery_baro_agl_mm_;
    have_altitude = true;
  } else if (recovery_gps_cal_done_ && gps_valid) {
    selected_agl_mm = recovery_gps_agl_mm_;
    have_altitude = true;
  }

  if (have_altitude) {
    if (!recovery_initialized_) {
      recovery_initialized_ = true;
      recovery_launch_alt_mm_ = 0;
      recovery_last_alt_mm_ = selected_agl_mm;
      recovery_last_t_ms_ = now_ms;
      recovery_agl_mm_ = selected_agl_mm;
      recovery_max_agl_mm_ = selected_agl_mm;
      recovery_vspeed_cms_ = 0;
      recovery_phase_ = RECOVERY_PHASE_IDLE;
      recovery_launch_armed_ = false;
      recovery_gps_fix_3d_ = gps_valid;
      recovery_gps_fix_3d_latched_ = gps_valid;
      recovery_liftoff_detected_ = false;
      recovery_apogee_detected_ = false;
      recovery_have_min_press_ = false;
      recovery_min_press_pa_x10_ = 0;
      recovery_drogue_deployed_ = false;
      recovery_main_deployed_ = false;
      recovery_landing_detected_ = false;
      recovery_drogue_reason_ = RECOVERY_REASON_NONE;
      recovery_main_reason_ = RECOVERY_REASON_NONE;
      recovery_drogue_deploy_agl_mm_ = -1;
      recovery_main_deploy_agl_mm_ = -1;
    } else {
      recovery_agl_mm_ = (selected_agl_mm > 0) ? selected_agl_mm : 0;

      if (recovery_agl_mm_ > recovery_max_agl_mm_) {
        recovery_max_agl_mm_ = recovery_agl_mm_;
      }

      recovery_vspeed_cms_ = 0;
      const uint32_t dt_ms = now_ms - recovery_last_t_ms_;
      if (dt_ms > 0 && dt_ms <= 10000) {
        const int32_t delta_mm = recovery_agl_mm_ - recovery_last_alt_mm_;
        const int32_t vspeed_cms = (delta_mm * 100) / static_cast<int32_t>(dt_ms);
        if (vspeed_cms > 32767) {
          recovery_vspeed_cms_ = 32767;
        } else if (vspeed_cms < -32768) {
          recovery_vspeed_cms_ = -32768;
        } else {
          recovery_vspeed_cms_ = static_cast<int16_t>(vspeed_cms);
        }
      }

      // Require explicit altitude gain to declare liftoff. This avoids
      // pre-launch jitter/noise from tripping the entire recovery sequence.
      if (recovery_launch_armed_ && !recovery_liftoff_detected_ &&
          recovery_agl_mm_ >= RECOVERY_LIFTOFF_CONFIRM_AGL_MM) {
        recovery_liftoff_detected_ = true;
      }

      if (press_pa_x10 > 0) {
        if (!recovery_have_min_press_) {
          recovery_have_min_press_ = true;
          recovery_min_press_pa_x10_ = press_pa_x10;
        } else if (!recovery_drogue_deployed_ && press_pa_x10 < recovery_min_press_pa_x10_) {
          recovery_min_press_pa_x10_ = press_pa_x10;
        }
      }

      const bool flight_enabled = recovery_launch_armed_ && recovery_liftoff_detected_;
      const bool altitude_drop = recovery_agl_mm_ <= (recovery_max_agl_mm_ - RECOVERY_APOGEE_DROP_MM);
      const bool descending = recovery_agl_mm_ <= (recovery_max_agl_mm_ - RECOVERY_DESCENT_CONFIRM_MM)
        || recovery_vspeed_cms_ < 0;
      if (flight_enabled) {
        recovery_phase_ = descending ? RECOVERY_PHASE_DESCENT : RECOVERY_PHASE_ASCENT;
      } else {
        recovery_phase_ = RECOVERY_PHASE_IDLE;
      }

      if (flight_enabled && !recovery_drogue_deployed_) {
        const bool neg_vspeed = recovery_vspeed_cms_ <= RECOVERY_APOGEE_NEG_VSPEED_CMS;
        const bool pressure_rise = recovery_have_min_press_
          && press_pa_x10 >= (recovery_min_press_pa_x10_ + RECOVERY_APOGEE_PRESS_RISE_PA_X10);
        uint8_t votes = 0;
        if (altitude_drop) votes++;
        if (neg_vspeed) votes++;
        if (pressure_rise) votes++;

        if (votes >= RECOVERY_APOGEE_VOTE_MIN) {
          recovery_apogee_detected_ = true;
          recovery_drogue_deployed_ = true;
          recovery_drogue_reason_ = RECOVERY_DROGUE_REASON_APOGEE_VOTE;
          recovery_drogue_deploy_agl_mm_ = recovery_agl_mm_;
        }
      }

      if (!recovery_main_deployed_) {
        const bool fast_descent = recovery_vspeed_cms_ <= RECOVERY_MAIN_FAST_DESCENT_CMS;
        const bool low_enough_for_fast_backup = recovery_agl_mm_ <= RECOVERY_MAIN_FAST_DESCENT_MAX_AGL_MM;
        const bool crossed_backup_altitude = recovery_max_agl_mm_ > RECOVERY_MAIN_BACKUP_AGL_MM;
        const bool backup_alt_trigger = crossed_backup_altitude
          && recovery_agl_mm_ <= RECOVERY_MAIN_BACKUP_AGL_MM;

        if (recovery_drogue_deployed_) {
          const bool enough_drop = recovery_drogue_deploy_agl_mm_ < 0
            || recovery_agl_mm_ <= (recovery_drogue_deploy_agl_mm_ - RECOVERY_MAIN_DROP_AFTER_DROGUE_MM);
          const bool primary_main_trigger = descending && enough_drop
            && recovery_agl_mm_ <= RECOVERY_MAIN_DEPLOY_AGL_MM;
          const bool backup_fast_descent = descending && fast_descent && low_enough_for_fast_backup;
          uint8_t main_reason = RECOVERY_REASON_NONE;
          if (primary_main_trigger) {
            main_reason = RECOVERY_MAIN_REASON_PRIMARY_ALTITUDE;
          } else if (backup_fast_descent) {
            main_reason = RECOVERY_MAIN_REASON_FAST_DESCENT;
          } else if (backup_alt_trigger) {
            main_reason = RECOVERY_MAIN_REASON_BACKUP_ALTITUDE;
          }
          if (main_reason != RECOVERY_REASON_NONE) {
            recovery_main_deployed_ = true;
            recovery_main_reason_ = main_reason;
            recovery_main_deploy_agl_mm_ = recovery_agl_mm_;
          }
        } else if (flight_enabled) {
          // Backup path if apogee/drogue logic underperforms: still prevent ballistic descent.
          if (descending && backup_alt_trigger) {
            recovery_main_deployed_ = true;
            recovery_main_reason_ = RECOVERY_MAIN_REASON_BACKUP_NO_DROGUE;
            recovery_main_deploy_agl_mm_ = recovery_agl_mm_;
          }
        }
      }

      if (recovery_main_deployed_ && recovery_agl_mm_ <= RECOVERY_LANDED_AGL_MM) {
        recovery_landing_detected_ = true;
      }

      if (recovery_landing_detected_) {
        recovery_phase_ = RECOVERY_PHASE_LANDED;
      }

      recovery_last_alt_mm_ = recovery_agl_mm_;
      recovery_last_t_ms_ = now_ms;
    }
  }

  int32_t gps_late = -2147483647;
  int32_t alt_late = -2147483647;
  int32_t imu_late = -2147483647;
  int32_t bat_late = -2147483647;
  int32_t navsat_late = -2147483647;
  int32_t recovery_late = -2147483647;

  if (gps != nullptr && gps_int_ms != 0 && gps->last_pvt_ms != 0) {
    if (last_gps_tx_ms_ == 0) gps_late = (int32_t)now_ms;
    else gps_late = (int32_t)((uint32_t)(now_ms - last_gps_tx_ms_) - gps_int_ms);
  }
  if (alt_int_ms != 0) {
    if (last_alt_tx_ms_ == 0) alt_late = (int32_t)now_ms;
    else alt_late = (int32_t)((uint32_t)(now_ms - last_alt_tx_ms_) - alt_int_ms);
  }
  if (imu != nullptr && imu_int_ms != 0) {
    if (last_imu_tx_ms_ == 0) imu_late = (int32_t)now_ms;
    else imu_late = (int32_t)((uint32_t)(now_ms - last_imu_tx_ms_) - imu_int_ms);
  }
  if (bat_int_ms != 0) {
    if (last_bat_tx_ms_ == 0) bat_late = (int32_t)now_ms;
    else bat_late = (int32_t)((uint32_t)(now_ms - last_bat_tx_ms_) - bat_int_ms);
  }
  if (gps != nullptr && navsat_int_ms != 0 && gps->last_sat_ms != 0) {
    if (last_navsat_tx_ms_ == 0) navsat_late = (int32_t)now_ms;
    else navsat_late = (int32_t)((uint32_t)(now_ms - last_navsat_tx_ms_) - navsat_int_ms);
  }
  if (recovery_initialized_ && recovery_int_ms != 0) {
    if (last_recovery_tx_ms_ == 0) recovery_late = (int32_t)now_ms;
    else recovery_late = (int32_t)((uint32_t)(now_ms - last_recovery_tx_ms_) - recovery_int_ms);
  }

  auto pick_most_overdue = [](int32_t best_late, int32_t candidate_late) {
    if (candidate_late < 0) return false;
    if (best_late < 0) return true;
    return candidate_late > best_late;
  };

  uint8_t pick = 0xFF;
  int32_t best_late = -2147483647;

  if (pick_most_overdue(best_late, gps_late)) {
    pick = 2;
    best_late = gps_late;
  }
  if (pick_most_overdue(best_late, navsat_late)) {
    pick = 5;
    best_late = navsat_late;
  }
  if (pick_most_overdue(best_late, recovery_late)) {
    pick = 6;
    best_late = recovery_late;
  }
  if (pick_most_overdue(best_late, alt_late)) {
    pick = 0;
    best_late = alt_late;
  }
  if (pick_most_overdue(best_late, imu_late)) {
    pick = 3;
    best_late = imu_late;
  }
  if (pick_most_overdue(best_late, bat_late)) {
    pick = 4;
    best_late = bat_late;
  }

  if (pick == 0xFF) return;

  pending_valid_ = true;
  pending_type_ = pick;
  retries_left_ = LORA_RETRY_LIMIT;
  retry_after_ms_ = 0;

  bool ok = false;
  if (pick == 0) {
    ok = start_alt_tx_(now_ms, press_pa_x10, temp_c_x100);
  } else if (pick == 2) {
    ok = start_gps_tx_(now_ms, *gps);
  } else if (pick == 5) {
    ok = start_navsat_tx_(now_ms, *gps);
  } else if (pick == 6) {
    ok = start_recovery_tx_(now_ms);
  } else if (pick == 3) {
    ok = start_imu_tx_(now_ms, *imu);
  } else if (pick == 4) {
    tx_buf_[0] = 0xA1;
    tx_buf_[1] = 4;
    write_u32_le(&tx_buf_[2], (uint32_t)now_ms);
    write_u16_le(&tx_buf_[6], vbat_mv);
    tx_buf_[8] = bat_state;
    const size_t n = 9;

    pending_valid_ = true;
    pending_type_ = 4;
    pending_len_ = n;

    lora_tx_done_isr = false;
    tx_start_ms_ = now_ms;
    tx_active_ = true;
    rx_active_ = false;
    tx_is_id_ = false;
    tx_is_ack_ = false;
    tx_type_ = 4;
    tx_len_ = n;

    int state = radio.startTransmit(tx_buf_, n);
    if (state != 0) {
      tx_active_ = false;
      DBG_PRINTF("lora: startTransmit err %d\n", state);
      ok = false;
    } else {
      ok = true;
    }
  }

  if (!ok) {
    schedule_retry_(now_ms);
  }
  return;
}

bool LoraLink::validate_config_() const {
  const float f = LORA_FREQ_MHZ;
  if (!(f >= 420.0f && f <= 450.0f)) return false;

  if (!(LORA_SF >= 6 && LORA_SF <= 12)) return false;
  if (!(LORA_CR >= 5 && LORA_CR <= 8)) return false;
  if (!lora_bw_valid(LORA_BW_KHZ)) return false;

  if (!(LORA_TX_POWER_DBM >= 2 && LORA_TX_POWER_DBM <= 17)) return false;

  const size_t cs_len = strlen(LORA_CALLSIGN);
  if (cs_len < 3) return false;
  if (strcmp(LORA_CALLSIGN, "N0CALL") == 0) return false;

  if (LORA_MIN_TX_INTERVAL_MS < 100) return false;
#if LORA_HEARTBEAT_ENABLE
  if (LORA_HEARTBEAT_MS != 0 && LORA_HEARTBEAT_MS < LORA_MIN_TX_INTERVAL_MS) return false;
#endif
  if (LORA_RETRY_LIMIT > 5) return false;
  return true;
}

bool LoraLink::start_navsat_tx_(uint32_t now_ms,
                               const GnssTime& gps) {
  if (!tx_enabled_) return false;

  uint8_t cno_max = 0;
  uint32_t cno_sum = 0;
  const uint8_t count = gps.navsat_n;
  for (uint8_t i = 0; i < count; ++i) {
    const uint8_t cno = gps.navsat[i].cno;
    cno_sum += cno;
    if (cno > cno_max) cno_max = cno;
  }
  const uint8_t cno_avg = count ? (uint8_t)((cno_sum + (count / 2)) / count) : 0;

  tx_buf_[0] = 0xA1;
  tx_buf_[1] = 5;
  write_u32_le(&tx_buf_[2], (uint32_t)now_ms);
  tx_buf_[6] = gps.navsat_num_svs;
  tx_buf_[7] = gps.navsat_n;
  tx_buf_[8] = cno_max;
  tx_buf_[9] = cno_avg;
  write_u16_le(&tx_buf_[10], gps.hdop_x100);
  const size_t n = 12;

  pending_valid_ = true;
  pending_type_ = 5;
  pending_len_ = n;

  lora_tx_done_isr = false;
  tx_start_ms_ = now_ms;
  tx_active_ = true;
  rx_active_ = false;
  tx_is_id_ = false;
  tx_is_ack_ = false;
  tx_type_ = 5;
  tx_len_ = n;

  int state = radio.startTransmit(tx_buf_, n);
  if (state != 0) {
    tx_active_ = false;
    DBG_PRINTF("lora: startTransmit err %d\n", state);
    return false;
  }

  return true;
}

bool LoraLink::allow_tx_() const {
  if (!config_ok_) return false;
  if (!began_) return false;
  if (shutdown_) return false;
  if (faulted_) return false;
#if !LORA_CONTROL_OPERATOR_OK
  return false;
#else
  return true;
#endif
}

void LoraLink::log_params_() const {
  DBG_PRINTF(
    "lora: part97 freq=%.3fMHz bw=%.1fkHz sf=%u cr=%u pwr=%ddBm callsign=%s min_int=%lums retry=%u wdog=%lums mission=%lums\n",
    (double)LORA_FREQ_MHZ,
    (double)LORA_BW_KHZ,
    (unsigned)LORA_SF,
    (unsigned)LORA_CR,
    (int)tx_power_dbm_,
    LORA_CALLSIGN,
    (unsigned long)LORA_MIN_TX_INTERVAL_MS,
    (unsigned)LORA_RETRY_LIMIT,
    (unsigned long)LORA_TX_WATCHDOG_MS,
    (unsigned long)LORA_MAX_MISSION_TX_MS
  );
}

void LoraLink::enter_silence_() {
  tx_active_ = false;
  retry_after_ms_ = 0;
  retries_left_ = 0;
  pending_valid_ = false;
  pending_type_ = 0;
  pending_len_ = 0;
  tx_is_id_ = false;
  tx_is_ack_ = false;
  id_retry_after_ms_ = 0;
  id_retries_left_ = 0;
  ack_pending_ = false;
  ack_retry_after_ms_ = 0;
  ack_retries_left_ = 0;
  start_rx_();
}

void LoraLink::on_tx_done_() {
  tx_active_ = false;
  last_tx_ms_ = millis();
  next_tx_ms_ = last_tx_ms_ + LORA_MIN_TX_INTERVAL_MS;
  retry_after_ms_ = 0;
  retries_left_ = 0;
  consec_fail_ = 0;

  if (tx_is_ack_) {
    schedule_ack_retry_(last_tx_ms_);
  } else if (tx_is_id_) {
    last_id_tx_ms_ = last_tx_ms_;
    id_retry_after_ms_ = 0;
    id_retries_left_ = 0;
  } else {
    if (tx_type_ == 0) {
      last_alt_tx_ms_ = last_tx_ms_;
    } else if (tx_type_ == 2) {
      last_gps_tx_ms_ = last_tx_ms_;
    } else if (tx_type_ == 3) {
      last_imu_tx_ms_ = last_tx_ms_;
    } else if (tx_type_ == 4) {
      last_bat_tx_ms_ = last_tx_ms_;
    } else if (tx_type_ == 5) {
      last_navsat_tx_ms_ = last_tx_ms_;
    } else if (tx_type_ == 6) {
      last_recovery_tx_ms_ = last_tx_ms_;
    }
    pending_valid_ = false;
  }

  tx_is_id_ = false;
  tx_is_ack_ = false;
  tx_type_ = 0;
  tx_len_ = 0;
  start_rx_();
}

bool LoraLink::start_rx_() {
  if (!began_ || !config_ok_) {
    rx_active_ = false;
    return false;
  }
  const int state = radio.startReceive();
  if (state != 0) {
    rx_active_ = false;
    return false;
  }
  rx_active_ = true;
  return true;
}

void LoraLink::poll_rx_(uint32_t now_ms) {
  (void)now_ms;
  if (!began_ || !config_ok_) return;
  if (tx_active_) return;
  if (!rx_active_) start_rx_();

  const uint16_t irq = radio.getIRQFlags();
  if (!(irq & LORA_RX_DONE_FLAG)) return;

  uint8_t buf[16];
  size_t len = radio.getPacketLength();
  if (len == 0) {
    len = sizeof(buf);
  }
  if (len > sizeof(buf)) {
    len = sizeof(buf);
  }

  const int state = radio.readData(buf, len);
  if (state == 0) {
    handle_command_(buf, len);
  }
  start_rx_();
}

bool LoraLink::handle_command_(const uint8_t* data, size_t len) {
  if (len < 2) return false;
  if (data[0] != LORA_CMD_MAGIC) return false;
  const uint8_t cmd = data[1];
  if (cmd == LORA_CMD_BUZZER) {
    if (len < 3) return false;
  } else if (cmd == LORA_CMD_SET_TX_POWER) {
    if (len < 3) return false;
  } else if (cmd != LORA_CMD_SD_START &&
             cmd != LORA_CMD_SD_STOP &&
             cmd != LORA_CMD_TELEM_ENABLE &&
             cmd != LORA_CMD_TELEM_DISABLE &&
             cmd != LORA_CMD_ALT_CALIBRATE &&
             cmd != LORA_CMD_IMU_CALIBRATE &&
             cmd != LORA_CMD_LAUNCH_ARM &&
             cmd != LORA_CMD_SD_ROTATE &&
             cmd != LORA_CMD_SD_FORMAT &&
             cmd != LORA_CMD_SD_DUMP_SAMPLE) {
    return false;
  }
#if DEBUG_MODE
  DBG_PRINTF("lora: cmd rx 0x%02X\n", cmd);
#endif
  pending_cmd_ = cmd;
  pending_cmd_arg_ =
      ((cmd == LORA_CMD_BUZZER || cmd == LORA_CMD_SET_TX_POWER || cmd == LORA_CMD_LAUNCH_ARM) && len >= 3)
          ? data[2]
          : 0;
  return true;
}

bool LoraLink::start_id_tx_(uint32_t now_ms, uint16_t vbat_mv) {
  if (!allow_tx_()) return false;

  const size_t cs_len = strlen(LORA_CALLSIGN);
  if (cs_len == 0) return false;
  if (cs_len > (sizeof(tx_buf_) - 6)) return false;

  tx_buf_[0] = 0xA1;
  tx_buf_[1] = 1;
  tx_buf_[2] = (uint8_t)cs_len;
  memcpy(&tx_buf_[3], LORA_CALLSIGN, cs_len);
  write_u16_le(&tx_buf_[3 + cs_len], vbat_mv);
  tx_buf_[5 + cs_len] = tx_power_dbm_;
  const size_t n = 6 + cs_len;

  lora_tx_done_isr = false;
  tx_start_ms_ = now_ms;
  tx_active_ = true;
  rx_active_ = false;
  tx_is_id_ = true;
  tx_is_ack_ = false;

  int state = radio.startTransmit(tx_buf_, n);
  if (state != 0) {
    tx_active_ = false;
    tx_is_id_ = false;
    DBG_PRINTF("lora: startTransmit err %d\n", state);
    return false;
  }

  return true;
}

bool LoraLink::start_alt_tx_(uint32_t now_ms,
                             int32_t press_pa_x10,
                             int16_t temp_c_x100) {
  if (!tx_enabled_) return false;

  tx_buf_[0] = 0xA1;
  tx_buf_[1] = 0;
  write_u32_le(&tx_buf_[2], (uint32_t)now_ms);
  write_u32_le(&tx_buf_[6], (uint32_t)press_pa_x10);
  write_u16_le(&tx_buf_[10], (uint16_t)temp_c_x100);
  const size_t n = 12;

  pending_valid_ = true;
  pending_type_ = 0;
  pending_len_ = n;

  lora_tx_done_isr = false;
  tx_start_ms_ = now_ms;
  tx_active_ = true;
  rx_active_ = false;
  tx_is_id_ = false;
  tx_is_ack_ = false;
  tx_type_ = 0;
  tx_len_ = n;

  int state = radio.startTransmit(tx_buf_, n);
  if (state != 0) {
    tx_active_ = false;
    DBG_PRINTF("lora: startTransmit err %d\n", state);
    return false;
  }

  last_press_pa_x10_ = press_pa_x10;
  last_temp_c_x100_ = temp_c_x100;
  have_last_alt_ = true;
  return true;
}

bool LoraLink::start_gps_tx_(uint32_t now_ms,
                            const GnssTime& gps) {
  if (!tx_enabled_) return false;

  tx_buf_[0] = 0xA1;
  tx_buf_[1] = 2;
  write_u32_le(&tx_buf_[2], (uint32_t)now_ms);
  write_u32_le(&tx_buf_[6], (uint32_t)gps.lat_e7);
  write_u32_le(&tx_buf_[10], (uint32_t)gps.lon_e7);
  write_u32_le(&tx_buf_[14], (uint32_t)gps.height_mm);
  const size_t n = 18;

  pending_valid_ = true;
  pending_type_ = 2;
  pending_len_ = n;

  lora_tx_done_isr = false;
  tx_start_ms_ = now_ms;
  tx_active_ = true;
  rx_active_ = false;
  tx_is_id_ = false;
  tx_is_ack_ = false;
  tx_type_ = 2;
  tx_len_ = n;

  int state = radio.startTransmit(tx_buf_, n);
  if (state != 0) {
    tx_active_ = false;
    DBG_PRINTF("lora: startTransmit err %d\n", state);
    return false;
  }

  return true;
}

bool LoraLink::start_imu_tx_(uint32_t now_ms,
                            const ImuSample& imu) {
  if (!tx_enabled_) return false;

  tx_buf_[0] = 0xA1;
  tx_buf_[1] = 3;
  write_u32_le(&tx_buf_[2], (uint32_t)now_ms);
  write_i16_le(&tx_buf_[6], imu.gx);
  write_i16_le(&tx_buf_[8], imu.gy);
  write_i16_le(&tx_buf_[10], imu.gz);
  write_i16_le(&tx_buf_[12], imu.ax);
  write_i16_le(&tx_buf_[14], imu.ay);
  write_i16_le(&tx_buf_[16], imu.az);
  const size_t n = 18;

  pending_valid_ = true;
  pending_type_ = 3;
  pending_len_ = n;

  lora_tx_done_isr = false;
  tx_start_ms_ = now_ms;
  tx_active_ = true;
  rx_active_ = false;
  tx_is_id_ = false;
  tx_is_ack_ = false;
  tx_type_ = 3;
  tx_len_ = n;

  int state = radio.startTransmit(tx_buf_, n);
  if (state != 0) {
    tx_active_ = false;
    DBG_PRINTF("lora: startTransmit err %d\n", state);
    return false;
  }

  return true;
}

void LoraLink::consume_pending_(uint32_t now_ms) {
  if (!pending_valid_) return;

  if (pending_type_ == 0) {
    last_alt_tx_ms_ = now_ms;
  } else if (pending_type_ == 2) {
    last_gps_tx_ms_ = now_ms;
  } else if (pending_type_ == 5) {
    last_navsat_tx_ms_ = now_ms;
  } else if (pending_type_ == 6) {
    last_recovery_tx_ms_ = now_ms;
  } else if (pending_type_ == 3) {
    last_imu_tx_ms_ = now_ms;
  } else if (pending_type_ == 4) {
    last_bat_tx_ms_ = now_ms;
  }
}

void LoraLink::schedule_retry_(uint32_t now_ms) {
  consec_fail_++;
  if (consec_fail_ >= LORA_MAX_CONSEC_TX_FAILS) {
    DBG_PRINTLN("lora: too many tx failures");
#if LORA_HEARTBEAT_ENABLE
    tx_enabled_ = false;
    consec_fail_ = 0;
    enter_silence_();
#else
    DBG_PRINTLN("lora: too many tx failures -> shutdown");
    shutdown();
#endif
    return;
  }

  if (retries_left_ == 0) {
    consume_pending_(now_ms);
    last_tx_ms_ = now_ms;
    pending_valid_ = false;
    enter_silence_();
    return;
  }

  const uint8_t attempt = (uint8_t)(LORA_RETRY_LIMIT - retries_left_ + 1);
  const uint32_t backoff = LORA_RETRY_BASE_MS * (uint32_t)attempt;
  retries_left_--;
  retry_after_ms_ = now_ms + backoff;
}

void LoraLink::schedule_ack_retry_(uint32_t now_ms) {
  if (ack_retries_left_ == 0) {
    ack_pending_ = false;
    ack_retry_after_ms_ = 0;
    return;
  }

  ack_retries_left_--;
  if (ack_retries_left_ == 0) {
    ack_pending_ = false;
    ack_retry_after_ms_ = 0;
    return;
  }

  ack_retry_after_ms_ = now_ms + LORA_ACK_REPEAT_MS;
}
