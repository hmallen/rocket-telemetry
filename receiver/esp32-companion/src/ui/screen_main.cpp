#include "screen_main.h"

#include <math.h>

MainScreen::MainScreen(TFT_eSPI& tft) : tft_(tft) {}

void MainScreen::begin() {
  tft_.init();
  tft_.setRotation(1);  // Landscape 320x240
  tft_.fillScreen(TFT_BLACK);
  tft_.setTextColor(TFT_WHITE, TFT_BLACK);
  tft_.setTextDatum(TL_DATUM);
}

void MainScreen::render(const CompanionState& state) {
  if (millis() - lastRenderMs_ < 100) return;  // 10 fps
  lastRenderMs_ = millis();

  tft_.fillScreen(TFT_BLACK);
  drawHeader(state);
  drawPrimary(state);
  drawFooter(state);
}

void MainScreen::drawHeader(const CompanionState& state) {
  uint16_t linkColor = state.stale ? TFT_ORANGE : (state.link.connected ? TFT_GREEN : TFT_RED);
  tft_.fillRect(0, 0, 320, 28, TFT_DARKGREY);

  tft_.setTextColor(linkColor, TFT_DARKGREY);
  tft_.setTextFont(2);
  tft_.drawString(state.link.connected ? "LINK" : "NO LINK", 6, 6);

  tft_.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft_.drawString("RSSI " + String(state.link.rssi), 80, 6);
  tft_.drawString("SNR " + String(state.link.snr, 1), 160, 6);
  tft_.drawString("AGE " + String(state.link.lastPacketAgeMs) + "ms", 225, 6);
}

void MainScreen::drawPrimary(const CompanionState& state) {
  tft_.setTextColor(TFT_CYAN, TFT_BLACK);
  tft_.setTextFont(4);
  tft_.drawString("PHASE: " + state.flight.phase, 8, 40);

  tft_.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft_.setTextFont(6);
  String altText = isnan(state.alt.altitudeAglM) ? "---" : String(state.alt.altitudeAglM, 1);
  tft_.drawString("ALT " + altText + "m", 8, 85);

  tft_.setTextColor(TFT_GREEN, TFT_BLACK);
  tft_.setTextFont(4);
  String vsText = isnan(state.alt.verticalSpeedMps) ? "---" : String(state.alt.verticalSpeedMps, 1);
  tft_.drawString("VS  " + vsText + " m/s", 8, 155);
}

void MainScreen::drawFooter(const CompanionState& state) {
  tft_.fillRect(0, 210, 320, 30, TFT_DARKGREY);
  tft_.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft_.setTextFont(2);

  String callsign = state.flight.callsign.length() ? state.flight.callsign : "(none)";
  tft_.drawString("CS " + callsign, 6, 219);

  String batt = isnan(state.battery.vbatV) ? "--.-" : String(state.battery.vbatV, 2);
  tft_.drawString("VBAT " + batt + "V", 140, 219);

  tft_.drawString("PKT " + String(state.flight.packetCount), 245, 219);
}
