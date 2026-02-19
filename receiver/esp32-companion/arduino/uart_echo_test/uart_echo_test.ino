#include <Arduino.h>

// LCDWiki E32R32P/E32N32P UART header is RXD0/TXD0 (IO3/IO1).
// For this bring-up test we use UART2 mapped onto those pins.
static constexpr int UART_RX_PIN = 3;
static constexpr int UART_TX_PIN = 1;
static constexpr uint32_t UART_BAUD = 115200;

HardwareSerial Uart(2);

void setup() {
  Uart.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
}

void loop() {
  static uint8_t buf[256];

  int avail = Uart.available();
  if (avail > 0) {
    size_t toRead = static_cast<size_t>(avail);
    if (toRead > sizeof(buf)) toRead = sizeof(buf);

    size_t n = Uart.readBytes(buf, toRead);
    if (n > 0) {
      Uart.print("OK\r\n");
      Uart.write(buf, n);
    }
  }

  delay(2);
}
