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
  Serial.begin(115200);
  delay(100);

  // LCDWiki E32R32P/E32N32P backlight control (IO27): high = backlight on.
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);

  connectWifi();

  controller = new Controller(tft, String(GS_HOST), static_cast<uint16_t>(GS_PORT));
  controller->begin();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
    delay(300);
    return;
  }

  if (controller != nullptr) {
    controller->tick();
  }

  delay(10);
}
