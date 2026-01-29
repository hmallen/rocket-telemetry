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

static int16_t abs_i16(int16_t x) {
  return (x < 0) ? (int16_t)-x : x;
}

static void write_u16_le(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void write_u32_le(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

bool LoraLink::begin() {
  began_ = true;
  mission_start_ms_ = millis();

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
  (void)radio.setOutputPower(LORA_TX_POWER_DBM);
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

void LoraLink::poll_telem(uint32_t now_ms,
                          int32_t press_pa_x10,
                          int16_t temp_c_x100) {
  if (!began_) return;

  // Hard failsafe: maximum mission transmit duration.
  if ((uint32_t)(now_ms - mission_start_ms_) >= LORA_MAX_MISSION_TX_MS) {
    shutdown();
    return;
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
      DBG_PRINTLN("lora: tx watchdog -> shutdown");
      (void)radio.finishTransmit();
      shutdown();
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
      DBG_PRINTLN("lora: landing detected -> shutdown");
      shutdown();
      return;
    }
  }

  if (!tx_enabled_) return;

  const bool changed = (!have_last_) ||
    (abs_i32(press_pa_x10 - last_press_pa_x10_) >= LORA_MEANINGFUL_PRESS_DELTA_PA_X10) ||
    (abs_i16((int16_t)(temp_c_x100 - last_temp_c_x100_)) >= LORA_MEANINGFUL_TEMP_DELTA_C_X100);

  if (changed) {
    const bool pending_differs = (!pending_valid_) ||
      (press_pa_x10 != pending_press_pa_x10_) ||
      (temp_c_x100 != pending_temp_c_x100_);
    if (pending_differs) {
      pending_press_pa_x10_ = press_pa_x10;
      pending_temp_c_x100_ = temp_c_x100;
      pending_valid_ = true;
      retries_left_ = LORA_RETRY_LIMIT;
    }
  }

  const bool id_due = (LORA_HEARTBEAT_MS != 0) &&
    ((last_id_tx_ms_ == 0) || ((uint32_t)(now_ms - last_id_tx_ms_) >= LORA_HEARTBEAT_MS));
  const bool id_retry_ready = (id_retry_after_ms_ != 0) && ((int32_t)(now_ms - id_retry_after_ms_) >= 0);
  const bool want_id = id_due || id_retry_ready;

  if ((uint32_t)(now_ms - last_tx_ms_) < LORA_MIN_TX_INTERVAL_MS) return;
  if (next_tx_ms_ != 0 && (int32_t)(now_ms - next_tx_ms_) < 0) return;

  if (want_id && !(id_retry_after_ms_ != 0 && (int32_t)(now_ms - id_retry_after_ms_) < 0)) {
    if (!start_id_tx_(now_ms)) {
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

  if (!pending_valid_) return;
  if (retry_after_ms_ != 0 && (int32_t)(now_ms - retry_after_ms_) < 0) return;

  if (!start_data_tx_(now_ms,
                      pending_press_pa_x10_,
                      pending_temp_c_x100_)) {
    schedule_retry_(now_ms);
  }
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
  if (!(LORA_HEARTBEAT_MS == 0 || LORA_HEARTBEAT_MS <= 30000)) return false;
  if (LORA_HEARTBEAT_MS != 0 && LORA_HEARTBEAT_MS < LORA_MIN_TX_INTERVAL_MS) return false;
  if (LORA_RETRY_LIMIT > 5) return false;
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
    (int)LORA_TX_POWER_DBM,
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
  tx_is_id_ = false;
  id_retry_after_ms_ = 0;
  id_retries_left_ = 0;
  (void)radio.sleep();
}

void LoraLink::on_tx_done_() {
  tx_active_ = false;
  last_tx_ms_ = millis();
  next_tx_ms_ = last_tx_ms_ + LORA_MIN_TX_INTERVAL_MS;
  retry_after_ms_ = 0;
  retries_left_ = 0;
  consec_fail_ = 0;

  if (tx_is_id_) {
    last_id_tx_ms_ = last_tx_ms_;
    id_retry_after_ms_ = 0;
    id_retries_left_ = 0;
  } else {
    pending_valid_ = false;
  }

  tx_is_id_ = false;
}

bool LoraLink::start_id_tx_(uint32_t now_ms) {
  if (!tx_enabled_) return false;

  const size_t cs_len = strlen(LORA_CALLSIGN);
  if (cs_len == 0) return false;
  if (cs_len > (sizeof(tx_buf_) - 3)) return false;

  tx_buf_[0] = 0xA1;
  tx_buf_[1] = 1;
  tx_buf_[2] = (uint8_t)cs_len;
  memcpy(&tx_buf_[3], LORA_CALLSIGN, cs_len);
  const size_t n = 3 + cs_len;

  lora_tx_done_isr = false;
  tx_start_ms_ = now_ms;
  tx_active_ = true;
  tx_is_id_ = true;

  int state = radio.startTransmit(tx_buf_, n);
  if (state != 0) {
    tx_active_ = false;
    tx_is_id_ = false;
    DBG_PRINTF("lora: startTransmit err %d\n", state);
    return false;
  }

  return true;
}

bool LoraLink::start_data_tx_(uint32_t now_ms,
                             int32_t press_pa_x10,
                             int16_t temp_c_x100) {
  if (!tx_enabled_) return false;

  tx_buf_[0] = 0xA1;
  tx_buf_[1] = 0;
  write_u32_le(&tx_buf_[2], (uint32_t)now_ms);
  write_u32_le(&tx_buf_[6], (uint32_t)press_pa_x10);
  write_u16_le(&tx_buf_[10], (uint16_t)temp_c_x100);
  const size_t n = 12;

  lora_tx_done_isr = false;
  tx_start_ms_ = now_ms;
  tx_active_ = true;
  tx_is_id_ = false;

  int state = radio.startTransmit(tx_buf_, n);
  if (state != 0) {
    tx_active_ = false;
    DBG_PRINTF("lora: startTransmit err %d\n", state);
    return false;
  }

  last_press_pa_x10_ = press_pa_x10;
  last_temp_c_x100_ = temp_c_x100;
  have_last_ = true;
  return true;
}

void LoraLink::schedule_retry_(uint32_t now_ms) {
  consec_fail_++;
  if (consec_fail_ >= LORA_MAX_CONSEC_TX_FAILS) {
    DBG_PRINTLN("lora: too many tx failures -> shutdown");
    shutdown();
    return;
  }

  if (retries_left_ == 0) {
    // Fail toward silence: do not persist on the same payload indefinitely.
    // Treat the pending payload as "consumed" so we won't immediately re-arm retries
    // until meaningful data changes again.
    last_press_pa_x10_ = pending_press_pa_x10_;
    last_temp_c_x100_ = pending_temp_c_x100_;
    have_last_ = true;
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
