# ESP32 Companion Display (Phase 2 Scaffold)

This is a first-pass firmware scaffold for the 3.2" ESP32 touchscreen companion display.

## What this includes

- PlatformIO project for ESP32 (Arduino framework)
- Wi-Fi connection + auto-reconnect
- Initial snapshot fetch from Pi: `GET /api/companion/state`
- Live telemetry stream from Pi: `GET /api/companion/events` (SSE)
- Optional direct UART transport to the Pi ground station host (bypass Wi-Fi)
- Minimal dashboard UI on TFT:
  - Link status / RSSI / SNR / packet age
  - Flight phase
  - Altitude AGL
  - Vertical speed
  - Callsign, battery, packet count
- Stale-link watchdog (marks link stale after 3s)
- Touch command buttons (XPT2046):
  - `BUZZ 1s`
  - `BUZZ 5s`
  - `SD TOGGLE` (start/stop)
- Alert strip + recent event history cues (phase/link/alert changes)
- Command ack status text on footer
- Quick touch calibration mode (tap top-left corner; exit via top-right)

## Directory

```text
receiver/esp32-companion/
  platformio.ini
  include/config.h.example
  src/main.cpp
  src/app/controller.*
  src/model/telemetry_state.h
  src/net/api_client.*
  src/serial/uart_proto.*
  src/serial/uart_link.*
  src/ui/screen_main.*
  UART_PROTOCOL.md
```

## Setup

1. Copy config template:

```bash
cp include/config.h.example include/config.h
```

2. Edit `include/config.h` with:
- Wi-Fi SSID/password
- Pi host/IP and port (ground station)
- Touch pins and calibration bounds
- `GS_AUTH_TOKEN` (required if companion command endpoint is protected)
- `COMPANION_LINK_UART` and UART pin/baud settings if using wired serial

3. Install/build/upload with PlatformIO:

```bash
pio run
pio run -t upload
pio device monitor
```

## TFT_eSPI note

This scaffold uses `TFT_eSPI`. You must configure your panel in TFT_eSPI's setup
(User_Setup or User_Setup_Select) to match your hardware wiring (ST7789, pins, rotation).

## Touch note

Touch uses XPT2046. Raw touch calibration differs by panel orientation and wiring.
Start with the defaults in `config.h.example`, then calibrate and update:

- `TOUCH_X_MIN`, `TOUCH_X_MAX`
- `TOUCH_Y_MIN`, `TOUCH_Y_MAX`
- `TOUCH_SWAP_XY` (set to `1` if X/Y are rotated/swapped)

## UART mode

Set `COMPANION_LINK_UART` to `1` in `config.h` to use direct UART instead of Wi-Fi.

See `UART_PROTOCOL.md` for frame format and Pi-ground-station integration notes.

## Next steps (Phase 4)

- Add alert panel and event history
- Add screen brightness control + night mode
- Optional: migrate UI to LVGL for richer widgets
