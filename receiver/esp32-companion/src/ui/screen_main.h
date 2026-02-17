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
  void renderCalibration(int rawX, int rawY, int mappedX, int mappedY, bool hasPoint);
  void setCommandStatus(const String& msg, bool ok);
  void pushAlert(const String& msg);
  const ButtonRects& buttons() const { return buttons_; }

 private:
  TFT_eSPI& tft_;
  ButtonRects buttons_;
  uint32_t lastRenderMs_ = 0;
  bool firstRender_ = true;
  void drawHeader(const CompanionState& state);
  void drawPrimary(const CompanionState& state);
  void drawAlertStrip(const CompanionState& state);
  void drawFooter(const CompanionState& state);
  void drawButtons(const CompanionState& state);

  String cmdMsg_;
  bool cmdOk_ = true;
  uint32_t cmdTs_ = 0;
  String alerts_[4];
  int alertCount_ = 0;
};
