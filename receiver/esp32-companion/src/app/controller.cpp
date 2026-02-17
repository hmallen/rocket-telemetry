#include "controller.h"

#include "config.h"

Controller::Controller(TFT_eSPI& tft, const String& host, uint16_t port)
    : api_(host, port), screen_(tft), touch_(TOUCH_CS_PIN, TOUCH_IRQ_PIN) {}

void Controller::begin() {
  screen_.begin();
  touch_.begin();
  touch_.setRotation(1);

  if (api_.fetchInitialState(state_)) {
    lastRxMs_ = millis();
  }

  lastPhase_ = state_.flight.phase;
  lastConnected_ = state_.link.connected;
  lastStale_ = state_.stale;
  lastPrimaryAlert_ = state_.primaryAlert;

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

  if (state_.link.connected != lastConnected_) {
    screen_.pushAlert(state_.link.connected ? "LINK RESTORED" : "LINK DOWN");
    lastConnected_ = state_.link.connected;
  }
  if (state_.stale != lastStale_) {
    screen_.pushAlert(state_.stale ? "LINK STALE" : "LINK LIVE");
    lastStale_ = state_.stale;
  }
}

void Controller::tick() {
  ensureConnected();

  if (sseConnected_) {
    bool updated = api_.pollEventStream(state_);
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
  }

  handleTouch();
  updateStaleness();
  if (mode_ == UiMode::DASHBOARD) {
    screen_.render(state_);
  }
}

bool Controller::inside(int x, int y, int bx, int by, int bw, int bh) {
  return x >= bx && x <= (bx + bw) && y >= by && y <= (by + bh);
}

bool Controller::mapTouchToScreen(int rawX, int rawY, int& outX, int& outY) {
  if (TOUCH_X_MAX <= TOUCH_X_MIN || TOUCH_Y_MAX <= TOUCH_Y_MIN) return false;
  if (TOUCH_SWAP_XY) {
    int tmp = rawX;
    rawX = rawY;
    rawY = tmp;
  }
  long sx = map(rawX, TOUCH_X_MIN, TOUCH_X_MAX, 0, 319);
  long sy = map(rawY, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, 239);
  outX = constrain((int)sx, 0, 319);
  outY = constrain((int)sy, 0, 239);
  return true;
}

void Controller::handleTouch() {
  if (!touch_.touched()) return;
  uint32_t now = millis();
  if (now - lastTouchMs_ < 250) return;  // debounce

  TS_Point p = touch_.getPoint();
  int x = 0, y = 0;
  if (!mapTouchToScreen(p.x, p.y, x, y)) return;

  if (mode_ == UiMode::CALIBRATION) {
    screen_.renderCalibration(p.x, p.y, x, y, true);
    if (x > 280 && y < 40) {
      mode_ = UiMode::DASHBOARD;
      lastTouchMs_ = now;
    }
    return;
  }

  // Enter calibration mode from top-left corner.
  if (x < 40 && y < 40) {
    mode_ = UiMode::CALIBRATION;
    screen_.renderCalibration(p.x, p.y, x, y, true);
    lastTouchMs_ = now;
    return;
  }

  const auto& b = screen_.buttons();
  bool sent = false;

  if (inside(x, y, b.buzz1X, b.buzz1Y, b.buzz1W, b.buzz1H)) {
    sent = api_.sendCommand("buzzer", 1);
    screen_.setCommandStatus(sent ? "BUZZ 1s sent" : "BUZZ 1s failed", sent);
  } else if (inside(x, y, b.buzz5X, b.buzz5Y, b.buzz5W, b.buzz5H)) {
    sent = api_.sendCommand("buzzer", 5);
    screen_.setCommandStatus(sent ? "BUZZ 5s sent" : "BUZZ 5s failed", sent);
  } else if (inside(x, y, b.sdX, b.sdY, b.sdW, b.sdH)) {
    sent = api_.sendCommand(sdLoggingEnabled_ ? "sd_stop" : "sd_start", 0);
    if (sent) sdLoggingEnabled_ = !sdLoggingEnabled_;
    screen_.setCommandStatus(
        sent ? (sdLoggingEnabled_ ? "SD start sent" : "SD stop sent") : "SD cmd failed", sent);
  }

  if (sent) {
    lastTouchMs_ = now;
  }
}
