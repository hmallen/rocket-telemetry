# ESP32 Companion Display (LVGL / CrowPanel 3.5)

This firmware targets the **CrowPanel 3.5" ESP32 HMI (480x320 landscape)** as a
ground-station telemetry display.

## What this now includes

- Full LVGL-based UI (rebuilt from scratch)
- UART-first companion transport to Pi ground station
- Optional Wi-Fi API/SSE transport fallback
- Telemetry dashboard:
  - Link status, RSSI, SNR, packet age
  - Flight phase, altitude AGL, vertical speed
  - Packet count, callsign
  - `TX_VBAT` (rocket telemetry battery)
  - `BAT_ADC` (local display board battery ADC)
- Collapsible action panel:
  - `BUZZER 1s`
  - `BUZZER 5s`
  - `SD START`
  - `SD STOP`
  - `TX ENABLE`
  - `TX DISABLE`
- Collapsible settings menu with touch calibration flow
- Touch calibration persistence in NVS (`Preferences`)

## Build + flash

```bash
pio run -e esp32dev
pio run -e esp32dev -t upload
pio device monitor -b 115200
```

## Configuration

1. Copy config template:

```bash
cp include/config.h.example include/config.h
```

2. Edit `include/config.h`:

- `COMPANION_LINK_UART` (`1` recommended for ground station)
- UART pins/baud for your board revision
- Wi-Fi + host settings if using API/SSE mode
- auth token if `/api/companion/cmd` is protected

## CrowPanel pin notes

`platformio.ini` is configured for CrowPanel 3.5 with ILI9488 + XPT2046 defaults.

- Default profile uses: `TFT_MISO=12`, `TOUCH_CS=33`
- Elecrow v2.2 boards may require: `TFT_MISO=33`, `TOUCH_CS=12`

If your panel revision differs, override build flags in `platformio.ini`.

## Touch calibration

Use **Settings -> Calibrate Touch** from the LVGL action panel.

The calibration flow captures all four corners and stores bounds/swap config in
NVS so they persist across reboots.

## UART protocol

See `UART_PROTOCOL.md` for frame details and command IDs used between Pi and ESP32.
