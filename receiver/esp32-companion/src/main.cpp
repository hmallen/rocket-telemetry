#if !defined(TOUCH_ECHO_TEST) && !defined(TTY_ECHO_TEST) && !defined(TOUCH_PROBE_TEST)

#include <Arduino.h>
#include <WiFi.h>
#include <TFT_eSPI.h>

#if __has_include("config.h")
#include "config.h"
#else
#error "Missing include/config.h. Copy include/config.h.example to include/config.h and set values."
#endif

#ifndef OTA_ENABLE
#define OTA_ENABLE 0
#endif

#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME "esp32-companion"
#endif

#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif

#ifndef OTA_PORT
#define OTA_PORT 3232
#endif

#if OTA_ENABLE
#include <ArduinoOTA.h>
#endif

#include "app/lvgl_controller.h"

TFT_eSPI tft = TFT_eSPI();
LvglController* controller = nullptr;

namespace {
#if !(COMPANION_LINK_UART && (UART_RX_PIN == 3) && (UART_TX_PIN == 1))
constexpr bool kDebugSerialAvailable = true;
#else
constexpr bool kDebugSerialAvailable = false;
#endif

constexpr uint32_t kWifiReconnectIntervalMs = 5000;
uint32_t gLastWifiConnectAttemptMs = 0;

#if OTA_ENABLE
bool gOtaStarted = false;
#endif

bool shouldUseWifi() {
  return (!COMPANION_LINK_UART) || OTA_ENABLE;
}

void startOtaIfReady() {
#if OTA_ENABLE
  if (gOtaStarted || WiFi.status() != WL_CONNECTED) {
    return;
  }

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPort(OTA_PORT);
  if (OTA_PASSWORD[0] != '\0') {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }
  ArduinoOTA.begin();
  gOtaStarted = true;
#endif
}
}  // namespace

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
  if (!shouldUseWifi()) {
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    startOtaIfReady();
    return;
  }

  uint32_t now = millis();
  if (gLastWifiConnectAttemptMs != 0 &&
      (now - gLastWifiConnectAttemptMs) < kWifiReconnectIntervalMs) {
    return;
  }

  gLastWifiConnectAttemptMs = millis();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

#if !(COMPANION_LINK_UART && (UART_RX_PIN == 3) && (UART_TX_PIN == 1))
  if (kDebugSerialAvailable) {
    Serial.print("Connecting WiFi");
  }
#endif

  if (COMPANION_LINK_UART) {
    return;
  }

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(300);
#if !(COMPANION_LINK_UART && (UART_RX_PIN == 3) && (UART_TX_PIN == 1))
    if (kDebugSerialAvailable) {
      Serial.print(".");
    }
#endif
  }

#if !(COMPANION_LINK_UART && (UART_RX_PIN == 3) && (UART_TX_PIN == 1))
  if (kDebugSerialAvailable) {
    Serial.println();
  }
#endif

  if (WiFi.status() == WL_CONNECTED) {
    startOtaIfReady();
#if !(COMPANION_LINK_UART && (UART_RX_PIN == 3) && (UART_TX_PIN == 1))
    if (kDebugSerialAvailable) {
      Serial.print("WiFi connected: ");
      Serial.println(WiFi.localIP());
    }
#endif
  } else {
#if !(COMPANION_LINK_UART && (UART_RX_PIN == 3) && (UART_TX_PIN == 1))
    if (kDebugSerialAvailable) {
      Serial.println("WiFi connect timeout");
    }
#endif
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

  if (shouldUseWifi()) {
    connectWifi();
  }

  controller = new LvglController(tft, String(GS_HOST), static_cast<uint16_t>(GS_PORT));
  controller->begin();

  startOtaIfReady();
}

void loop() {
  if (shouldUseWifi() && WiFi.status() != WL_CONNECTED) {
#if OTA_ENABLE
    gOtaStarted = false;
#endif
    connectWifi();

    if (!COMPANION_LINK_UART) {
      delay(300);
      return;
    }
  }

#if OTA_ENABLE
  startOtaIfReady();
  if (gOtaStarted) {
    ArduinoOTA.handle();
  }
#endif

  if (controller != nullptr) {
    controller->tick();
  }

  delay(10);
}

#endif  // !TOUCH_ECHO_TEST && !TTY_ECHO_TEST && !TOUCH_PROBE_TEST
