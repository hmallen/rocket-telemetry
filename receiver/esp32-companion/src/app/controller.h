#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>

#include "config.h"

#include "../model/telemetry_state.h"
#include "../net/api_client.h"
#include "../serial/uart_link.h"
#include "../ui/screen_main.h"

class Controller {
 public:
  Controller(TFT_eSPI& tft, const String& host, uint16_t port);
  void begin();
  void tick();

 private:
  ApiClient api_;
  UartLink uart_;
  MainScreen screen_;
  CompanionState state_;

  bool sseConnected_ = false;
  uint32_t lastRxMs_ = 0;
  uint32_t lastReconnectAttemptMs_ = 0;
  uint32_t lastCompanionBatSampleMs_ = 0;
  String lastPhase_;
  bool lastConnected_ = false;
  bool lastStale_ = true;
  String lastPrimaryAlert_;

  void updateStaleness();
  void updateCompanionBattery();
  bool ensureConnected();

  // Static command callback registered with MainScreen so LVGL button
  // handlers can send commands back through UART or HTTP API.
  static Controller* instance_;
  static bool sendCommandCb(const char* action, int durationS);
};
