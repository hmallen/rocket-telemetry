#include <Arduino.h>

// Alternate UART test that keeps UART0 (IO1/IO3) free for USB serial monitor.
// Wire your external UART to these pins for this test.
static constexpr int UART_RX_PIN = 16;
static constexpr int UART_TX_PIN = 17;
static constexpr uint32_t UART_BAUD = 115200;

HardwareSerial Uart(2);

void setup() {
  Serial.begin(115200);
  Serial.println("UART echo test (alt pins) starting...");

  Uart.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.printf("Listening on UART2 RX=%d TX=%d @ %lu\n", UART_RX_PIN, UART_TX_PIN, UART_BAUD);
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

      Serial.print("RX ");
      Serial.print(n);
      Serial.println(" bytes, echoed");
    }
  }

  delay(2);
}
