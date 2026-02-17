#include "controller.h"

Controller::Controller(TFT_eSPI& tft, const String& host, uint16_t port)
    : api_(host, port), screen_(tft) {}

void Controller::begin() {
  screen_.begin();

  if (api_.fetchInitialState(state_)) {
    lastRxMs_ = millis();
  }
  sseConnected_ = api_.openEventStream();
}

bool Controller::ensureConnected() {
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
}

void Controller::tick() {
  ensureConnected();

  if (sseConnected_) {
    bool updated = api_.pollEventStream(state_);
    if (updated) {
      lastRxMs_ = millis();
    }
  }

  updateStaleness();
  screen_.render(state_);
}
