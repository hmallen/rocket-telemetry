# LVGL UI Rebuild Specification

## Overview
Rebuild the ESP32 companion display UI from scratch using LVGL 8.3.x instead of raw TFT_eSPI drawing.
The display is a **CrowPanel 3.5" HMI ESP32 (480x320)** with ILI9488 driver and resistive touchscreen.
The ESP32 module is ESP32-WROVER-B.

## Hardware Details (CrowPanel 3.5")
- **Display:** 480x320 TFT-LCD, ILI9488 driver
- **Touch:** Resistive (XPT2046)
- **MCU:** ESP32-WROVER-B
- **Backlight pin:** GPIO 27 (HIGH = on)
- **Resolution:** 480x320 (landscape rotation=1)

### Pin Configuration (from CrowPanel reference)
The CrowPanel uses TFT_eSPI with ILI9488. The existing platformio.ini has ST7789 config which is WRONG for this display. Must update to ILI9488 with correct CrowPanel pins.

Reference from CrowPanel PlatformIO example (board: denky32):
```ini
lib_deps = 
  lvgl/lvgl@8.3.6
  bodmer/TFT_eSPI@2.5.31
```

## What to Change

### 1. platformio.ini
- Change board from `esp32dev` to `denky32` (or keep esp32dev but with WROVER settings, partition scheme supporting PSRAM)
- Replace TFT_eSPI build_flags: switch from ST7789 to ILI9488_DRIVER for the CrowPanel
- Add LVGL library dependency: `lvgl/lvgl@^8.3.6`
- Keep ArduinoJson dependency
- Keep XPT2046_Touchscreen or switch to TFT_eSPI built-in touch
- Add lv_conf.h build flag: `-D LV_CONF_INCLUDE_SIMPLE`
- Set LVGL tick and buffer config via build_flags or lv_conf.h

### 2. Create `include/lv_conf.h`
- Enable color depth 16
- Set LV_HOR_RES_MAX=480, LV_VER_RES_MAX=320
- Enable LV_USE_LOG for debug
- Enable widgets needed: label, btn, tabview, dropdown, slider, checkbox, switch, arc, bar, chart
- Enable LV_FONT_MONTSERRAT_14, _16, _20, _24
- Set LV_MEM_SIZE appropriately (48KB+ if PSRAM available)

### 3. Rewrite `src/ui/` — Replace screen_main.cpp/.h entirely

#### Main Layout (LVGL)
Create an LVGL-based UI with these screens/panels:

**A. Telemetry Dashboard (primary screen, fullscreen by default)**
- **Header bar (top ~30px):** Link status indicator (colored dot), RSSI, SNR, packet age, packet count
- **Main area:** 
  - Flight phase (large text, color-coded)
  - Altitude AGL (very large, prominent — this is THE primary reading)
  - Vertical speed
  - Max altitude (track and display)
- **Battery strip:** Telemetry TX battery voltage, companion battery voltage
- **Alert strip:** Scrolling/latest alert text at bottom of data area

**B. Actions Panel (collapsible, slides in from the right or bottom)**
- Toggle button/tab to expand/collapse
- When expanded, takes ~40% of screen width (right side) or ~40% height (bottom)
- When collapsed, just a small tab/handle visible at edge
- Contains:
  - BUZZ 1s button
  - BUZZ 5s button  
  - SD Toggle button (shows current state: Recording/Stopped)
  - Future: arm/disarm, launch sequence buttons
- Each button shows last command status briefly

**C. Settings Menu (collapsible, accessed via gear icon or swipe)**
- Collapsible panel or separate screen accessed from a settings icon
- Contains:
  - **Touchscreen Calibration** — interactive 3-point or 4-point calibration routine
    - Show crosshair targets at screen corners
    - User taps each target
    - Calculate and store calibration coefficients
    - Store cal data in NVS/EEPROM for persistence
  - Display brightness slider (PWM on pin 27)
  - UART baud rate display (read-only info)
  - Link mode display (UART vs WiFi)
  - About/version info

### 4. Update `src/main.cpp`
- Add LVGL initialization: `lv_init()`, display driver registration, input device registration
- Set up display flush callback using TFT_eSPI (like CrowPanel reference code)
- Set up touch input callback
- Call `lv_timer_handler()` in loop (or via timer)
- Remove direct TFT drawing — all rendering through LVGL

### 5. Update `src/app/controller.cpp/.h`
- Remove direct TFT_eSPI touch handling (XPT2046_Touchscreen) — LVGL handles input
- Keep telemetry state management, UART/API polling
- Add methods to update LVGL label objects with new telemetry data
- Keep the command sending logic (buzz, sd_start, sd_stop)
- Remove UiMode enum — LVGL handles screen switching

### 6. Keep untouched
- `src/serial/` — uart_link, uart_proto (working fine)
- `src/net/` — api_client (working fine)
- `src/model/telemetry_state.h` — data model (working fine)
- `include/config.h` — but may need to update pin defines for CrowPanel

## CrowPanel ILI9488 TFT_eSPI Build Flags
Based on CrowPanel reference, the TFT_eSPI configuration should be:
```
-D USER_SETUP_LOADED=1
-D ILI9488_DRIVER=1
-D TFT_WIDTH=320
-D TFT_HEIGHT=480
-D TFT_MISO=12
-D TFT_MOSI=13
-D TFT_SCLK=14
-D TFT_CS=15
-D TFT_DC=2
-D TFT_RST=-1
-D TOUCH_CS=33
-D SPI_FREQUENCY=27000000
-D SPI_READ_FREQUENCY=20000000
-D SPI_TOUCH_FREQUENCY=2500000
-D LOAD_GLCD=1
-D LOAD_FONT2=1
-D LOAD_FONT4=1
-D LOAD_FONT6=1
-D LOAD_FONT7=1
-D LOAD_FONT8=1
-D LOAD_GFXFF=1
-D SMOOTH_FONT=1
```

Note: CrowPanel V2.2 uses `TFT_MISO=33` and `TOUCH_CS=12` — check board version.

## LVGL Display Flush Callback Pattern
```cpp
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  lcd.startWrite();
  lcd.setAddrWindow(area->x1, area->y1, w, h);
  lcd.pushColors((uint16_t *)&color_p->full, w * h, true);
  lcd.endWrite();
  lv_disp_flush_ready(disp);
}
```

## LVGL Touch Input Callback Pattern
```cpp
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  uint16_t touchX, touchY;
  bool touched = lcd.getTouch(&touchX, &touchY, 600);
  if (!touched) {
    data->state = LV_INDEV_STATE_REL;
  } else {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchX;
    data->point.y = touchY;
  }
}
```

## Style Guide
- Dark theme (black background, colored accents)
- Altitude in large yellow/amber text
- Phase in cyan
- Link status: green=connected, red=disconnected, orange=stale
- Buttons: orange for buzzer, cyan for SD, red for dangerous actions
- Clean, readable fonts — Montserrat 14-24pt
- Consistent padding and alignment

## Build Verification
After making changes, run `pio run` to verify it compiles (no upload needed — we're building on a Pi, not connected to the ESP32 right now). Fix any compilation errors.

## Important Notes
- This is a PlatformIO project, NOT Arduino IDE
- The display buffer should be `screenWidth * screenHeight / 8` or similar — don't allocate full framebuffer (ESP32 RAM limited)
- Use `lv_disp_draw_buf_init` with a single buffer
- LVGL tick should be called from a timer or in the main loop
- Keep the existing UART protocol and API client code — only the UI layer changes
