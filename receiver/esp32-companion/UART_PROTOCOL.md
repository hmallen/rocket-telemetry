# Companion UART Protocol (Teensy ↔ ESP32)

This defines the direct wired fallback/primary transport for the companion display.

## Wiring

- Teensy TX -> ESP32 RX (UART)
- Teensy RX <- ESP32 TX (UART)
- GND -> GND
- 3.3V logic levels only on UART signal lines

## Frame format

```text
SOF1  SOF2  VER  TYPE  SEQ(2)  LEN(2)  PAYLOAD(LEN)  CRC16(2)
0xA5  0x5A  0x01
```

CRC16 is CCITT over: `VER..PAYLOAD`.

## Message types

- `0x01` `MSG_TELEM_SNAPSHOT` (Teensy -> ESP32)
- `0x02` `MSG_ALERT_EVENT` (Teensy -> ESP32)
- `0x03` `MSG_HEARTBEAT` (both)
- `0x10` `MSG_CMD` (ESP32 -> Teensy)
- `0x11` `MSG_CMD_ACK` (Teensy -> ESP32)

## Transport selection

In `include/config.h`:

```cpp
#define COMPANION_LINK_UART 1
#define UART_BAUD 115200
#define UART_RX_PIN 16
#define UART_TX_PIN 17
```

Set `COMPANION_LINK_UART 0` to use Wi-Fi API/SSE instead.

## ESP32 implementation

- Parser/encoder: `src/serial/uart_proto.*`
- UART state bridge: `src/serial/uart_link.*`
- Controller auto-switches between UART and Wi-Fi using `COMPANION_LINK_UART`.

## Teensy skeleton

Drop-in helper:

- `src/companion_uart.h`
- `src/companion_uart.cpp`

Usage pattern:

```cpp
companion_uart::init(Serial4, 115200);

// 10 Hz
companion_uart::Snapshot s{};
// fill fields
companion_uart::send_snapshot(s);

// poll commands in loop
auto c = companion_uart::poll_cmd();
if (c.available) {
  // execute c.cmd/c.arg
  companion_uart::send_cmd_ack(c.cmd, true);
}
```

Note: `poll_cmd()` is a minimal skeleton parser in this commit. Replace with a full
resynchronizing state machine + CRC validation for production flight use.
