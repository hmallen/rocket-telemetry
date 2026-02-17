# ESP32 Companion Display (Phase 2 Scaffold)

This is a first-pass firmware scaffold for the 3.2" ESP32 touchscreen companion display.

## What this includes

- PlatformIO project for ESP32 (Arduino framework)
- Wi-Fi connection + auto-reconnect
- Initial snapshot fetch from Pi: `GET /api/companion/state`
- Live telemetry stream from Pi: `GET /api/companion/events` (SSE)
- Minimal dashboard UI on TFT:
  - Link status / RSSI / SNR / packet age
  - Flight phase
  - Altitude AGL
  - Vertical speed
  - Callsign, battery, packet count
- Stale-link watchdog (marks link stale after 3s)

## Directory

```text
receiver/esp32-companion/
  platformio.ini
  include/config.h.example
  src/main.cpp
  src/app/controller.*
  src/model/telemetry_state.h
  src/net/api_client.*
  src/ui/screen_main.*
```

## Setup

1. Copy config template:

```bash
cp include/config.h.example include/config.h
```

2. Edit `include/config.h` with:
- Wi-Fi SSID/password
- Pi host/IP and port (ground station)

3. Install/build/upload with PlatformIO:

```bash
pio run
pio run -t upload
pio device monitor
```

## TFT_eSPI note

This scaffold uses `TFT_eSPI`. You must configure your panel in TFT_eSPI's setup
(User_Setup or User_Setup_Select) to match your hardware wiring (ST7789, pins, rotation).

## Next steps (Phase 3)

- Add touch handling (XPT2046)
- Add command buttons to call `POST /api/companion/cmd`
- Add alert panel and event history
- Add screen brightness control + night mode
- Optional: migrate UI to LVGL for richer widgets
