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
  if (millis() - lastRenderMs_ < 250) return;  // ~4 fps, easy to see live updates.
  lastRenderMs_ = millis();

  static uint32_t renderCount = 0;
  renderCount++;

  const char spinner[] = {'|', '/', '-', '\\'};
  const char spinnerChar = spinner[renderCount & 0x03];

  String linkStatus = state.stale ? "STALE" : (state.link.connected ? "LIVE" : "NO LINK");
  String altText = isnan(state.alt.altitudeAglM) ? "---" : String(state.alt.altitudeAglM, 1);
  String vsText = isnan(state.alt.verticalSpeedMps) ? "---" : String(state.alt.verticalSpeedMps, 1);
  String battText = isnan(state.battery.vbatV) ? "--.--" : String(state.battery.vbatV, 2);
  String alertText = state.primaryAlert.length() ? state.primaryAlert : "(none)";

  tft_.fillScreen(TFT_BLACK);
  tft_.setTextFont(2);
  tft_.setTextColor(TFT_WHITE, TFT_BLACK);

  int y = 6;
  tft_.drawString("Companion UART Debug", 6, y);
  y += 18;
  tft_.drawString("Render: " + String(renderCount) + "  " + String(spinnerChar), 6, y);
  y += 18;
  tft_.drawString("Uptime(ms): " + String(millis()), 6, y);
  y += 18;
  tft_.drawString("RX seq: " + String(state.seq), 6, y);
  y += 18;
  tft_.drawString("Link: " + linkStatus, 6, y);
  y += 18;
  tft_.drawString("RSSI: " + String(state.link.rssi) + "  SNR: " + String(state.link.snr, 1), 6, y);
  y += 18;
  tft_.drawString("Age(ms): " + String(state.link.lastPacketAgeMs), 6, y);
  y += 18;
  tft_.drawString("Phase: " + state.flight.phase, 6, y);
  y += 18;
  tft_.drawString("Alt(m): " + altText + "  VS: " + vsText, 6, y);
  y += 18;
  tft_.drawString("VBAT: " + battText + "V", 6, y);
  y += 18;
  tft_.drawString("Pkt: " + String(state.flight.packetCount), 6, y);
  y += 18;
  tft_.drawString("Alert: " + alertText, 6, y);

  firstRender_ = false;
}

void MainScreen::setCommandStatus(const String& msg, bool ok) {
  cmdMsg_ = msg;
  cmdOk_ = ok;
  cmdTs_ = millis();
}

void MainScreen::pushAlert(const String& msg) {
  if (msg.length() == 0) return;
  for (int i = 3; i > 0; --i) alerts_[i] = alerts_[i - 1];
  alerts_[0] = msg;
  if (alertCount_ < 4) alertCount_++;
}

void MainScreen::renderCalibration(int rawX, int rawY, int mappedX, int mappedY, bool hasPoint) {
  tft_.fillScreen(TFT_BLACK);
  tft_.setTextColor(TFT_CYAN, TFT_BLACK);
  tft_.setTextFont(2);
  tft_.drawString("TOUCH CAL MODE", 8, 8);
  tft_.setTextColor(TFT_WHITE, TFT_BLACK);
  tft_.drawString("Tap top-right corner to exit", 8, 30);

  if (hasPoint) {
    tft_.drawString("Raw X: " + String(rawX), 8, 60);
    tft_.drawString("Raw Y: " + String(rawY), 8, 80);
    tft_.drawString("Map X: " + String(mappedX), 8, 100);
    tft_.drawString("Map Y: " + String(mappedY), 8, 120);
    tft_.fillCircle(mappedX, mappedY, 4, TFT_GREEN);
  } else {
    tft_.drawString("Touch panel to sample coordinates", 8, 80);
  }
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
  // Clear only the primary content area instead of full-screen clear each frame.
  tft_.fillRect(0, 28, 320, 142, TFT_BLACK);

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
  tft_.drawString("VS  " + vsText + " m/s", 8, 145);
}

void MainScreen::drawAlertStrip(const CompanionState& state) {
  tft_.fillRect(0, 170, 320, 12, TFT_MAROON);
  tft_.setTextFont(1);
  tft_.setTextColor(TFT_WHITE, TFT_MAROON);

  String alertText;
  if (state.primaryAlert.length()) {
    alertText = state.primaryAlert;
  } else if (alertCount_ > 0) {
    alertText = alerts_[0];
  } else {
    alertText = state.stale ? "LINK STALE" : "Nominal";
  }
  tft_.drawString(alertText, 4, 172);
}

void MainScreen::drawFooter(const CompanionState& state) {
  tft_.fillRect(0, 210, 320, 30, TFT_DARKGREY);
  tft_.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft_.setTextFont(2);

  String leftText;
  if (cmdMsg_.length() && (millis() - cmdTs_ < 3000)) {
    leftText = cmdMsg_;
    tft_.setTextColor(cmdOk_ ? TFT_GREENYELLOW : TFT_RED, TFT_DARKGREY);
  } else {
    String callsign = state.flight.callsign.length() ? state.flight.callsign : "(none)";
    leftText = "CS " + callsign;
    tft_.setTextColor(TFT_WHITE, TFT_DARKGREY);
  }
  tft_.drawString(leftText, 6, 219);

  String batt = isnan(state.battery.vbatV) ? "--.-" : String(state.battery.vbatV, 2);
  tft_.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft_.drawString("VBAT " + batt + "V", 140, 219);

  tft_.drawString("PKT " + String(state.flight.packetCount), 245, 219);
}

void MainScreen::drawButtons(const CompanionState& state) {
  (void)state;
  tft_.setTextFont(2);
  tft_.setTextColor(TFT_BLACK, TFT_ORANGE);

  tft_.fillRoundRect(buttons_.buzz1X, buttons_.buzz1Y, buttons_.buzz1W, buttons_.buzz1H, 4, TFT_ORANGE);
  tft_.drawCentreString("BUZZ 1s", buttons_.buzz1X + buttons_.buzz1W / 2, buttons_.buzz1Y + 6, 2);

  tft_.fillRoundRect(buttons_.buzz5X, buttons_.buzz5Y, buttons_.buzz5W, buttons_.buzz5H, 4, TFT_ORANGE);
  tft_.drawCentreString("BUZZ 5s", buttons_.buzz5X + buttons_.buzz5W / 2, buttons_.buzz5Y + 6, 2);

  tft_.setTextColor(TFT_BLACK, TFT_CYAN);
  tft_.fillRoundRect(buttons_.sdX, buttons_.sdY, buttons_.sdW, buttons_.sdH, 4, TFT_CYAN);
  tft_.drawCentreString("SD TOGGLE", buttons_.sdX + buttons_.sdW / 2, buttons_.sdY + 6, 2);
}
