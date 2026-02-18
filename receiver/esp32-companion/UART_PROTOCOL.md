# Companion UART Protocol (Pi Ground Station ↔ ESP32)

This defines the direct wired fallback/primary transport for the companion display
when the **ground station host is the Raspberry Pi**.

## Wiring

- Pi TX -> ESP32 RX (UART)
- Pi RX <- ESP32 TX (UART)
- GND -> GND
- 3.3V logic levels only on UART signal lines

Pi note:
- Use the Pi UART on GPIO14/15 (`/dev/serial0` preferred).
- Disable Linux serial console on that UART if needed, so your app has exclusive access.

ESP32 board note (LCDWiki E32R32P/E32N32P):
- The exposed UART header is tied to ESP32 UART0 (`RXD0=IO3`, `TXD0=IO1`) per LCDWiki docs.
- GPIO numbers are ESP32 GPIO IDs (not Raspberry Pi BCM numbering).

## Frame format

```text
SOF1  SOF2  VER  TYPE  SEQ(2)  LEN(2)  PAYLOAD(LEN)  CRC16(2)
0xA5  0x5A  0x01
```

CRC16 is CCITT over: `VER..PAYLOAD`.

## Message types

- `0x01` `MSG_TELEM_SNAPSHOT` (Pi -> ESP32)
- `0x02` `MSG_ALERT_EVENT` (Pi -> ESP32)
- `0x03` `MSG_HEARTBEAT` (both)
- `0x10` `MSG_CMD` (ESP32 -> Pi)
- `0x11` `MSG_CMD_ACK` (Pi -> ESP32)

## Transport selection

In `include/config.h`:

```cpp
#define COMPANION_LINK_UART 1
#define UART_BAUD 115200
#define UART_RX_PIN 3
#define UART_TX_PIN 1
```

On LCDWiki E32R32P/E32N32P, these map to UART0 (`RXD0=IO3`, `TXD0=IO1`) on the
exposed UART header.

Set `COMPANION_LINK_UART 0` to use Wi-Fi API/SSE instead.

## ESP32 implementation

- Parser/encoder: `src/serial/uart_proto.*`
- UART state bridge: `src/serial/uart_link.*`
- Controller auto-switches between UART and Wi-Fi using `COMPANION_LINK_UART`.

## Host-side implementation guidance (Pi)

Run a small UART bridge process on the Pi that:

1. Encodes telemetry into `MSG_TELEM_SNAPSHOT` frames at ~10 Hz
2. Parses incoming `MSG_CMD` from ESP32
3. Executes command actions in ground-station code (`buzzer`, `sd_start`, `sd_stop`)
4. Replies with `MSG_CMD_ACK`

This repository now treats UART companion mode as **Pi ground station ↔ ESP32**.
