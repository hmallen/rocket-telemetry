#pragma once

// Copy this file to include/config.h and fill values.

#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASSWORD "WIFI_PASSWORD"

// Example: "192.168.1.200"
#define GS_HOST "192.168.1.200"
#define GS_PORT 8000

// Companion transport mode:
// 0 = Wi-Fi API/SSE (default)
// 1 = direct UART link to Pi ground station
#define COMPANION_LINK_UART 1
#define COMPANION_UART_DEBUG 0

// Optional auth token for /api/companion/cmd.
// Keep empty if not configured on ground station.
#define GS_AUTH_TOKEN ""

// TLS is not used in this first-pass local-field setup.
#define GS_USE_TLS 0

// UART link settings (used when COMPANION_LINK_UART=1)
#define UART_BAUD 115200
#define UART_RX_PIN 3
#define UART_TX_PIN 1

// XPT2046 touch controller pins (adjust to your wiring)
#define TOUCH_CS_PIN 12
//#define TOUCH_CS_PIN 33
//#define TOUCH_IRQ_PIN 36
#define TOUCH_IRQ_PIN -1
#define TOUCH_SCLK_PIN TFT_SCLK
#define TOUCH_MISO_PIN TFT_MISO
#define TOUCH_MOSI_PIN TFT_MOSI

// Touch calibration bounds (raw ADC values from your panel)
// Update after calibration for accurate button taps.
#define TOUCH_X_MIN 200
#define TOUCH_X_MAX 3900
#define TOUCH_Y_MIN 200
#define TOUCH_Y_MAX 3900

// Set to 1 if touch axes are rotated (X/Y swapped)
#define TOUCH_SWAP_XY 0

// Set to 1 to show a quick LCD bring-up test (RGB + text) at boot.
#define DISPLAY_TEST_SCREEN 1