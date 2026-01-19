#pragma once
#include <Arduino.h>

// ---------- Rates (Hz) ----------
constexpr uint32_t IMU_HZ   = 400;   // accel
constexpr uint32_t GYRO_HZ  = 760;   // gyro
constexpr uint32_t BARO_HZ  = 200;
constexpr uint32_t GNSS_HZ  = 10;
constexpr uint32_t LORA_HZ  = 2;

// ---------- Logging ----------
constexpr uint32_t LOG_BLOCK_BYTES = 32768;    // SD write unit (32 KiB)
constexpr uint32_t SD_SYNC_MS      = 250;      // periodic controlled sync
constexpr uint32_t PREALLOC_BYTES  = 128UL * 1024UL * 1024UL; // 128 MiB flight budget

// Internal byte ring (fast, ISR-safe if you keep writes short)
constexpr uint32_t RING_BYTES = 256 * 1024;

// PSRAM spool (bulk shock absorber). Allocate < 8 MiB to leave headroom.
constexpr uint32_t SPOOL_BYTES = 6UL * 1024UL * 1024UL;

// ---------- GNSS ----------
constexpr uint32_t GNSS_BAUD = 460800;
#define GNSS_SERIAL Serial1
constexpr uint8_t  GNSS_PPS_PIN = 2;   // pick an interrupt-capable pin

// ---------- I2C ----------
#define I2C_BUS Wire
constexpr uint32_t I2C_HZ = 1000000; // 1 MHz if your wiring allows

// ---------- LoRa (SX127x-style via RadioLib) ----------
constexpr uint8_t LORA_CS   = 10;
constexpr uint8_t LORA_DIO0 = 9;
constexpr uint8_t LORA_RST  = 8;
constexpr uint8_t LORA_BUSY = 255; // unused on SX127x
constexpr float   LORA_FREQ_MHZ = 915.0; // set to your legal band

// ---------- Build toggles ----------
#define ENABLE_SD_LOGGER 0
#define ENABLE_PSRAM_SPOOL 1
#define ENABLE_GNSS 1
#define ENABLE_SENSORS 1
#define ENABLE_LORA 0

#ifndef DEBUG_MODE
#define DEBUG_MODE 0
#endif

#if DEBUG_MODE
#define DBG_INIT() do { Serial.begin(115200); delay(10); } while (0)
#define DBG_PRINT(x) do { Serial.print(x); } while (0)
#define DBG_PRINTLN(x) do { Serial.println(x); } while (0)
#define DBG_PRINTF(...) do { Serial.printf(__VA_ARGS__); } while (0)
#else
#define DBG_INIT() do { } while (0)
#define DBG_PRINT(x) do { } while (0)
#define DBG_PRINTLN(x) do { } while (0)
#define DBG_PRINTF(...) do { } while (0)
#endif
