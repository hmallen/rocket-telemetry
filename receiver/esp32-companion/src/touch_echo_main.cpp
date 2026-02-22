#if defined(TOUCH_ECHO_TEST)

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

#if __has_include("config.h")
#include "config.h"
#else
#error "Missing include/config.h. Copy include/config.h.example to include/config.h and set values."
#endif

namespace {

constexpr int kScreenW = 320;
constexpr int kScreenH = 240;
constexpr int kStatusH = 80;

constexpr int kCols = 3;
constexpr int kRows = 4;
constexpr int kButtonW = 96;
constexpr int kButtonH = 34;
constexpr int kButtonGapX = 8;
constexpr int kButtonGapY = 2;
constexpr int kButtonLeft = 8;
constexpr int kButtonTop = 88;

const char* kKeys[kRows * kCols] = {
    "1", "2", "3",
    "4", "5", "6",
    "7", "8", "9",
    "*", "0", "#",
};

TFT_eSPI tft = TFT_eSPI();
XPT2046_Touchscreen touch(TOUCH_CS_PIN, TOUCH_IRQ_PIN);

uint32_t renderCount = 0;
uint32_t keyPressCount = 0;
String lastKey = "-";
int32_t lastRawX = -1;
int32_t lastRawY = -1;
int32_t lastMapX = -1;
int32_t lastMapY = -1;

int activeKeyIndex = -1;
uint32_t activeKeyUntilMs = 0;
uint32_t lastStatusTickMs = 0;
uint32_t lastPressMs = 0;

void keyRect(int idx, int& x, int& y, int& w, int& h) {
  int row = idx / kCols;
  int col = idx % kCols;
  x = kButtonLeft + col * (kButtonW + kButtonGapX);
  y = kButtonTop + row * (kButtonH + kButtonGapY);
  w = kButtonW;
  h = kButtonH;
}

void drawKey(int idx, bool active) {
  int x = 0;
  int y = 0;
  int w = 0;
  int h = 0;
  keyRect(idx, x, y, w, h);

  uint16_t fillColor = active ? TFT_GREEN : TFT_DARKGREY;
  uint16_t textColor = active ? TFT_BLACK : TFT_WHITE;

  tft.fillRoundRect(x, y, w, h, 4, fillColor);
  tft.drawRoundRect(x, y, w, h, 4, TFT_WHITE);
  tft.setTextColor(textColor, fillColor);
  tft.drawCentreString(kKeys[idx], x + w / 2, y + 8, 2);
}

void drawKeypad() {
  for (int i = 0; i < (kRows * kCols); ++i) {
    drawKey(i, false);
  }
}

bool mapTouchToScreen(int rawX, int rawY, int& outX, int& outY) {
  if (TOUCH_X_MAX <= TOUCH_X_MIN || TOUCH_Y_MAX <= TOUCH_Y_MIN) {
    return false;
  }

  if (TOUCH_SWAP_XY) {
    int temp = rawX;
    rawX = rawY;
    rawY = temp;
  }

  long sx = map(rawX, TOUCH_X_MIN, TOUCH_X_MAX, 0, kScreenW - 1);
  long sy = map(rawY, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, kScreenH - 1);

  outX = constrain(static_cast<int>(sx), 0, kScreenW - 1);
  outY = constrain(static_cast<int>(sy), 0, kScreenH - 1);
  return true;
}

int keyAt(int x, int y) {
  for (int i = 0; i < (kRows * kCols); ++i) {
    int bx = 0;
    int by = 0;
    int bw = 0;
    int bh = 0;
    keyRect(i, bx, by, bw, bh);
    if (x >= bx && x <= (bx + bw) && y >= by && y <= (by + bh)) {
      return i;
    }
  }
  return -1;
}

void drawStatus() {
  const char spinner[] = {'|', '/', '-', '\\'};
  char spin = spinner[renderCount & 0x03];

  tft.fillRect(0, 0, kScreenW, kStatusH, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(2);

  int y = 4;
  tft.drawString("Touch Echo Test", 6, y);
  y += 16;
  tft.drawString("Render: " + String(renderCount) + "  " + String(spin), 6, y);
  y += 16;
  tft.drawString("Presses: " + String(keyPressCount) + "  Last: " + lastKey, 6, y);
  y += 16;
  tft.drawString("Raw: " + String(lastRawX) + "," + String(lastRawY), 6, y);
  y += 16;
  tft.drawString("Map: " + String(lastMapX) + "," + String(lastMapY), 6, y);
}

}  // namespace

void setup() {
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);

  touch.begin();
#if defined(ESP32)
  // XPT2046_Touchscreen::begin() calls SPI.begin() with default pins.
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TOUCH_CS_PIN);
#endif
  touch.setRotation(1);

  drawStatus();
  drawKeypad();
}

void loop() {
  uint32_t now = millis();

  if (now - lastStatusTickMs >= 150) {
    lastStatusTickMs = now;
    renderCount++;
    drawStatus();
  }

  if (touch.touched()) {
    TS_Point p = touch.getPoint();
    lastRawX = p.x;
    lastRawY = p.y;

    int mappedX = -1;
    int mappedY = -1;
    if (mapTouchToScreen(p.x, p.y, mappedX, mappedY)) {
      lastMapX = mappedX;
      lastMapY = mappedY;

      int idx = keyAt(mappedX, mappedY);
      if (idx >= 0 && (now - lastPressMs) > 220) {
        lastPressMs = now;
        keyPressCount++;
        lastKey = kKeys[idx];

        if (activeKeyIndex >= 0 && activeKeyIndex != idx) {
          drawKey(activeKeyIndex, false);
        }

        activeKeyIndex = idx;
        activeKeyUntilMs = now + 180;
        drawKey(activeKeyIndex, true);
      }
    } else {
      lastMapX = -1;
      lastMapY = -1;
    }

    drawStatus();
  }

  if (activeKeyIndex >= 0 && now >= activeKeyUntilMs) {
    drawKey(activeKeyIndex, false);
    activeKeyIndex = -1;
  }

  delay(10);
}

#endif  // TOUCH_ECHO_TEST
