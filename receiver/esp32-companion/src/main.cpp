#if !defined(TOUCH_ECHO_TEST) && !defined(TTY_ECHO_TEST) && !defined(TOUCH_PROBE_TEST)

#include <Arduino.h>
#include <WiFi.h>
#include <TFT_eSPI.h>

#if __has_include("config.h")
#include "config.h"
#else
#error "Missing include/config.h. Copy include/config.h.example to include/config.h and set values."
#endif

#include "app/lvgl_controller.h"

TFT_eSPI tft = TFT_eSPI();
LvglController* controller = nullptr;

static void runDisplaySelfTest() {
#if DISPLAY_TEST_SCREEN
  tft.init();
  tft.setRotation(1);

  const int w = tft.width();
  const int h = tft.height();
  const int barW = w / 3;

  // RGB bars for quick visual verification.
  tft.fillRect(0, 0, barW, h / 2, TFT_RED);
  tft.fillRect(barW, 0, barW, h / 2, TFT_GREEN);
  tft.fillRect(2 * barW, 0, w - (2 * barW), h / 2, TFT_BLUE);

  tft.fillRect(0, h / 2, w, h / 2, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(2);
  tft.drawString("ESP32 Companion Display Test", 8, (h / 2) + 12);
  tft.drawString("If you can read this, LCD+BL are OK", 8, (h / 2) + 34);

  delay(2000);
#endif
}

static void connectWifi() {
  if (COMPANION_LINK_UART) {
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting WiFi");
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect timeout");
  }
}

void setup() {
  // Avoid using UART0 for debug when UART companion link is on RXD0/TXD0 (IO3/IO1),
  // otherwise debug logs can corrupt binary protocol frames.
#if !(COMPANION_LINK_UART && (UART_RX_PIN == 3) && (UART_TX_PIN == 1))
  Serial.begin(115200);
  delay(100);
#endif

  // LCDWiki E32R32P/E32N32P backlight control (IO27): high = backlight on.
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);

  runDisplaySelfTest();

  if (!COMPANION_LINK_UART) {
    connectWifi();
  }

  controller = new LvglController(tft, String(GS_HOST), static_cast<uint16_t>(GS_PORT));
  controller->begin();
}

void loop() {
  if (!COMPANION_LINK_UART) {
    if (WiFi.status() != WL_CONNECTED) {
      connectWifi();
      delay(300);
      return;
    }
  }

  if (controller != nullptr) {
    controller->tick();
  }

  delay(10);
}

#endif  // !TOUCH_ECHO_TEST && !TTY_ECHO_TEST && !TOUCH_PROBE_TEST
