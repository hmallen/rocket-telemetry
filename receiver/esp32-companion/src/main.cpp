#if !defined(TOUCH_ECHO_TEST) && !defined(TTY_ECHO_TEST)

#include <Arduino.h>
#include <WiFi.h>
#include <TFT_eSPI.h>

#if __has_include("config.h")
#include "config.h"
#else
#error "Missing include/config.h. Copy include/config.h.example to include/config.h and set values."
#endif

#include "app/controller.h"

TFT_eSPI tft = TFT_eSPI();
Controller* controller = nullptr;

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

  if (!COMPANION_LINK_UART) {
    connectWifi();
  }

  controller = new Controller(tft, String(GS_HOST), static_cast<uint16_t>(GS_PORT));
  controller->begin();  // calls screen_.begin() → lv_init(), driver registration, UI build
}

void loop() {
  if (!COMPANION_LINK_UART) {
    if (WiFi.status() != WL_CONNECTED) {
      connectWifi();
      delay(300);
      return;
    }
  }

  // Advance LVGL tick counter with real elapsed time.
  static uint32_t lastTickMs = 0;
  uint32_t now = millis();
  lv_tick_inc(now - lastTickMs);
  lastTickMs = now;

  if (controller != nullptr) {
    controller->tick();   // poll UART/API, push new data to LVGL labels
  }

  lv_timer_handler();  // run LVGL tasks (render, animations, input)

  delay(5);
}

#endif  // !TOUCH_ECHO_TEST && !TTY_ECHO_TEST
