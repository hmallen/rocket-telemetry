#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "../model/telemetry_state.h"

class MainScreen {
 public:
  MainScreen(TFT_eSPI& tft);
  void begin();
  void render(const CompanionState& state);

 private:
  TFT_eSPI& tft_;
  uint32_t lastRenderMs_ = 0;
  void drawHeader(const CompanionState& state);
  void drawPrimary(const CompanionState& state);
  void drawFooter(const CompanionState& state);
};
