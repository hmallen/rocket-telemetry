#include "controller.h"

#include "config.h"

#ifndef COMPANION_BAT_ADC_PIN
#define COMPANION_BAT_ADC_PIN 34
#endif

#ifndef COMPANION_BAT_ADC_DIVIDER_SCALE
#define COMPANION_BAT_ADC_DIVIDER_SCALE 2.0f
#endif

#ifndef COMPANION_BAT_ADC_CAL_SCALE
#define COMPANION_BAT_ADC_CAL_SCALE 1.0f
#endif

#ifndef COMPANION_BAT_ADC_SAMPLES
#define COMPANION_BAT_ADC_SAMPLES 16
#endif

#ifndef COMPANION_BAT_SAMPLE_INTERVAL_MS
#define COMPANION_BAT_SAMPLE_INTERVAL_MS 1000
#endif

namespace {
constexpr float kCompanionBatAdcRefV = 3.3f;
constexpr float kCompanionBatAdcMaxCounts = 4095.0f;
}

// ── Static member definitions ──────────────────────────────────────────────
Controller* Controller::instance_ = nullptr;

// ── Command callback (called by MainScreen LVGL button handlers) ───────────

bool Controller::sendCommandCb(const char* action, int durationS) {
  if (!instance_) return false;
  if (COMPANION_LINK_UART) {
    return instance_->uart_.sendCommand(action, durationS);
  } else {
    return instance_->api_.sendCommand(action, durationS);
  }
}

// ── Construction ───────────────────────────────────────────────────────────

Controller::Controller(TFT_eSPI& tft, const String& host, uint16_t port)
    : api_(host, port),
      uart_(Serial2, UART_BAUD, UART_RX_PIN, UART_TX_PIN),
      screen_(tft) {}

// ── begin() ────────────────────────────────────────────────────────────────

void Controller::begin() {
  instance_ = this;

  screen_.setCommandCallback(sendCommandCb);
  screen_.begin();  // inits TFT, LVGL drivers, and builds the full UI

#if COMPANION_LINK_UART
  pinMode(COMPANION_BAT_ADC_PIN, INPUT);
#endif

  if (COMPANION_LINK_UART) {
    uart_.begin();
  } else {
    if (api_.fetchInitialState(state_)) {
      lastRxMs_ = millis();
    }
  }

  lastPhase_ = state_.flight.phase;
  lastConnected_ = state_.link.connected;
  lastStale_ = state_.stale;
  lastPrimaryAlert_ = state_.primaryAlert;

  if (!COMPANION_LINK_UART) {
    sseConnected_ = api_.openEventStream();
  }
}

// ── Helpers ────────────────────────────────────────────────────────────────

bool Controller::ensureConnected() {
  if (COMPANION_LINK_UART) return true;
  if (sseConnected_) return true;

  uint32_t now = millis();
  if (now - lastReconnectAttemptMs_ < 2000) return false;
  lastReconnectAttemptMs_ = now;

  sseConnected_ = api_.openEventStream();
  return sseConnected_;
}

void Controller::updateStaleness() {
  uint32_t now = millis();
  uint32_t age = (lastRxMs_ == 0) ? UINT32_MAX : (now - lastRxMs_);
  state_.stale = (age > 3000);
  if (state_.stale) {
    state_.link.connected = false;
    state_.link.lastPacketAgeMs = static_cast<int>(age);
  }

  if (state_.link.connected != lastConnected_) {
    screen_.pushAlert(state_.link.connected ? "LINK RESTORED" : "LINK DOWN");
    lastConnected_ = state_.link.connected;
  }
  if (state_.stale != lastStale_) {
    screen_.pushAlert(state_.stale ? "LINK STALE" : "LINK LIVE");
    lastStale_ = state_.stale;
  }
}

void Controller::updateCompanionBattery() {
#if !COMPANION_LINK_UART
  return;
#endif

  uint32_t now = millis();
  if (lastCompanionBatSampleMs_ != 0 &&
      (now - lastCompanionBatSampleMs_) < COMPANION_BAT_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastCompanionBatSampleMs_ = now;

  uint32_t sum = 0;
  for (uint8_t i = 0; i < COMPANION_BAT_ADC_SAMPLES; ++i) {
    sum += static_cast<uint32_t>(analogRead(COMPANION_BAT_ADC_PIN));
  }

  float counts = static_cast<float>(sum) / static_cast<float>(COMPANION_BAT_ADC_SAMPLES);
  float vadc = counts * (kCompanionBatAdcRefV / kCompanionBatAdcMaxCounts);
  state_.battery.companionVbatV =
      vadc * COMPANION_BAT_ADC_DIVIDER_SCALE * COMPANION_BAT_ADC_CAL_SCALE;
}

// ── tick() ─────────────────────────────────────────────────────────────────

void Controller::tick() {
  ensureConnected();
  updateCompanionBattery();

  bool updated = false;
  if (COMPANION_LINK_UART) {
    updated = uart_.poll(state_);
  } else if (sseConnected_) {
    updated = api_.pollEventStream(state_);
  }

  if (updated) {
    lastRxMs_ = millis();

    if (state_.flight.phase != lastPhase_ && state_.flight.phase.length()) {
      screen_.pushAlert("PHASE -> " + state_.flight.phase);
      lastPhase_ = state_.flight.phase;
    }

    if (state_.primaryAlert.length() && state_.primaryAlert != lastPrimaryAlert_) {
      screen_.pushAlert(state_.primaryAlert);
      lastPrimaryAlert_ = state_.primaryAlert;
    }
  }

  updateStaleness();
  screen_.update(state_);
}
