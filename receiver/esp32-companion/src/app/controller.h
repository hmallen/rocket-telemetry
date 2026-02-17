#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

#include "../model/telemetry_state.h"
#include "../net/api_client.h"
#include "../ui/screen_main.h"

class Controller {
 public:
  Controller(TFT_eSPI& tft, const String& host, uint16_t port);
  void begin();
  void tick();

 private:
  enum class UiMode { DASHBOARD, CALIBRATION };

  ApiClient api_;
  MainScreen screen_;
  CompanionState state_;
  XPT2046_Touchscreen touch_;
  bool sdLoggingEnabled_ = false;

  bool sseConnected_ = false;
  uint32_t lastRxMs_ = 0;
  uint32_t lastReconnectAttemptMs_ = 0;
  uint32_t lastTouchMs_ = 0;
  UiMode mode_ = UiMode::DASHBOARD;
  String lastPhase_;
  bool lastConnected_ = false;
  bool lastStale_ = true;
  String lastPrimaryAlert_;

  void updateStaleness();
  bool ensureConnected();
  void handleTouch();
  bool inside(int x, int y, int bx, int by, int bw, int bh);
  bool mapTouchToScreen(int rawX, int rawY, int& outX, int& outY);
};
