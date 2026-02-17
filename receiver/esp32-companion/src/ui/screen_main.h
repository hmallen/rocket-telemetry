#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "../model/telemetry_state.h"

class MainScreen {
 public:
  struct ButtonRects {
    int buzz1X = 8, buzz1Y = 182, buzz1W = 96, buzz1H = 24;
    int buzz5X = 112, buzz5Y = 182, buzz5W = 96, buzz5H = 24;
    int sdX = 216, sdY = 182, sdW = 96, sdH = 24;
  };

  MainScreen(TFT_eSPI& tft);
  void begin();
  void render(const CompanionState& state);
  const ButtonRects& buttons() const { return buttons_; }

 private:
  TFT_eSPI& tft_;
  ButtonRects buttons_;
  uint32_t lastRenderMs_ = 0;
  void drawHeader(const CompanionState& state);
  void drawPrimary(const CompanionState& state);
  void drawFooter(const CompanionState& state);
  void drawButtons(const CompanionState& state);
};
