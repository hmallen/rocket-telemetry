#if defined(TTY_ECHO_TEST)

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <ctype.h>
#include <string.h>

#if __has_include("config.h")
#include "config.h"
#else
#error "Missing include/config.h. Copy include/config.h.example to include/config.h and set values."
#endif

namespace {

constexpr int kScreenW = 320;
constexpr int kScreenH = 240;
constexpr int kHeaderH = 62;
constexpr int kLineH = 16;
constexpr int kMaxLines = (kScreenH - kHeaderH) / kLineH;
constexpr bool kEchoBackToPi = true;
constexpr uint8_t kBytesPerDumpLine = 4;
constexpr uint32_t kPartialDumpFlushMs = 120;

#if defined(TTY_ECHO_UART_RX_PIN)
constexpr int kTtyEchoRxPin = TTY_ECHO_UART_RX_PIN;
#else
constexpr int kTtyEchoRxPin = UART_RX_PIN;
#endif

#if defined(TTY_ECHO_UART_TX_PIN)
constexpr int kTtyEchoTxPin = TTY_ECHO_UART_TX_PIN;
#else
constexpr int kTtyEchoTxPin = UART_TX_PIN;
#endif

TFT_eSPI tft = TFT_eSPI();
HardwareSerial piUart(2);

String lines[kMaxLines];
int currentLine = 0;
uint32_t rxBytes = 0;
uint32_t txBytes = 0;
uint32_t renderCount = 0;
uint32_t lastHeaderMs = 0;
uint32_t lastRxMs = 0;
String pendingHex;
String pendingAscii;
uint8_t pendingCount = 0;
bool bodyNeedsFullRedraw = true;
int dirtyLineIndex = -1;

void drawHeader() {
  renderCount++;

  tft.fillRect(0, 0, kScreenW, kHeaderH - 2, TFT_BLACK);
  tft.setCursor(6, 4);
  tft.print("Pi TTY Echo");

  tft.setCursor(6, 20);
  tft.print("Render: ");
  tft.print(renderCount);

  tft.setCursor(6, 36);
  tft.print("RX: ");
  tft.print(rxBytes);
  tft.print("  TX: ");
  tft.print(txBytes);

  tft.setCursor(6, 52);
  if (lastRxMs == 0) {
    tft.print("LastRX: never");
  } else {
    tft.print("LastRXms: ");
    tft.print(millis() - lastRxMs);
  }

  tft.drawFastHLine(0, kHeaderH - 2, kScreenW, TFT_DARKGREY);
}

void pushLine() {
  for (int i = 0; i < kMaxLines - 1; ++i) {
    lines[i] = lines[i + 1];
  }
  lines[kMaxLines - 1] = "";
  currentLine = kMaxLines - 1;
  bodyNeedsFullRedraw = true;
  dirtyLineIndex = -1;
}

void appendTerminalLine(const String& text) {
  if (currentLine >= kMaxLines - 1) {
    pushLine();
  } else {
    currentLine++;
  }
  lines[currentLine] = text;
  if (!bodyNeedsFullRedraw) {
    dirtyLineIndex = currentLine;
  }
}

void flushPendingDumpLine() {
  if (pendingCount == 0) {
    return;
  }

  appendTerminalLine(pendingHex + " |" + pendingAscii + "|");
  pendingHex = "";
  pendingAscii = "";
  pendingCount = 0;
}

void appendByteToDump(uint8_t value) {
  if (pendingCount > 0) {
    pendingHex += ' ';
  }

  const char kHexDigits[] = "0123456789ABCDEF";
  pendingHex += kHexDigits[(value >> 4) & 0x0F];
  pendingHex += kHexDigits[value & 0x0F];

  if (isprint(value)) {
    pendingAscii += static_cast<char>(value);
  } else {
    pendingAscii += '.';
  }

  pendingCount++;
  if (pendingCount >= kBytesPerDumpLine) {
    flushPendingDumpLine();
  }
}

void drawBodyFull() {
  tft.fillRect(0, kHeaderH, kScreenW, kScreenH - kHeaderH, TFT_BLACK);
  int y = kHeaderH;
  for (int i = 0; i < kMaxLines; ++i) {
    tft.setCursor(4, y);
    tft.print(lines[i]);
    y += kLineH;
  }

  bodyNeedsFullRedraw = false;
  dirtyLineIndex = -1;
}

void drawBodyLine(int lineIndex) {
  if (lineIndex < 0 || lineIndex >= kMaxLines) {
    return;
  }

  const int y = kHeaderH + (lineIndex * kLineH);
  tft.fillRect(0, y, kScreenW, kLineH, TFT_BLACK);
  tft.setCursor(4, y);
  tft.print(lines[lineIndex]);
}

void renderScreen() {
  drawHeader();
  if (bodyNeedsFullRedraw) {
    drawBodyFull();
  } else {
    drawBodyLine(dirtyLineIndex);
    dirtyLineIndex = -1;
  }
}

}  // namespace

void setup() {
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);

  tft.init();
  tft.setRotation(1);
  tft.setTextFont(1);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);

  piUart.begin(UART_BAUD, SERIAL_8N1, kTtyEchoRxPin, kTtyEchoTxPin);

  lines[0] = "ESP32 UART viewer ready";
  lines[1] = "UART " + String(UART_BAUD) + " RX" + String(kTtyEchoRxPin) + " TX" + String(kTtyEchoTxPin);
  lines[2] = "HEX+ASCII " + String(kBytesPerDumpLine) + "B lines";
  lines[3] = "Waiting for Pi tty data...";
  currentLine = 3;
  bodyNeedsFullRedraw = true;
  dirtyLineIndex = -1;

  if (kEchoBackToPi) {
    const char* banner = "\r\nESP32 tty echo ready\r\n";
    piUart.write(reinterpret_cast<const uint8_t*>(banner), strlen(banner));
    txBytes += strlen(banner);
  }

  renderScreen();
  lastHeaderMs = millis();
}

void loop() {
  bool changed = false;

  while (piUart.available() > 0) {
    int value = piUart.read();
    if (value < 0) {
      break;
    }

    uint8_t c = static_cast<uint8_t>(value);
    rxBytes++;
    lastRxMs = millis();
    appendByteToDump(c);
    changed = true;

    if (kEchoBackToPi) {
      piUart.write(c);
      txBytes++;
    }
  }

  if (pendingCount > 0 && (millis() - lastRxMs >= kPartialDumpFlushMs)) {
    flushPendingDumpLine();
    changed = true;
  }

  if (changed) {
    renderScreen();
    lastHeaderMs = millis();
  } else {
    const uint32_t now = millis();
    if (now - lastHeaderMs >= 250) {
      drawHeader();
      lastHeaderMs = now;
    }
  }

  delay(10);
}

#endif  // TTY_ECHO_TEST
