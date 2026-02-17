#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>

#include "../model/telemetry_state.h"
#include "../net/api_client.h"
#include "../ui/screen_main.h"

class Controller {
 public:
  Controller(TFT_eSPI& tft, const String& host, uint16_t port);
  void begin();
  void tick();

 private:
  ApiClient api_;
  MainScreen screen_;
  CompanionState state_;

  bool sseConnected_ = false;
  uint32_t lastRxMs_ = 0;
  uint32_t lastReconnectAttemptMs_ = 0;

  void updateStaleness();
  bool ensureConnected();
};
