#pragma once
#include <stdint.h>
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
#define GNSS_SERIAL_PRIMARY Serial1
#define GNSS_SERIAL_BACKUP  Serial2
constexpr uint32_t GNSS_FAILOVER_TIMEOUT_US = 2000000;
constexpr uint8_t  GNSS_PPS_PIN = 2;   // pick an interrupt-capable pin

// ---------- Buzzer ----------
constexpr uint8_t BUZZER_PIN = 28;

constexpr uint8_t GINT_PIN = 33;
constexpr uint8_t GRDY_PIN = 34;
constexpr uint8_t LIN1_PIN = 35;
constexpr uint8_t LIN2_PIN = 36;
constexpr uint8_t LRDY_PIN = 37;

// ---------- I2C ----------
#define I2C_BUS Wire
constexpr uint32_t I2C_HZ = 1000000; // 1 MHz if your wiring allows
constexpr uint8_t  BMP_CS = 6;
constexpr uint32_t BMP_SPI_HZ = 1000000;
#define BMP_SPI_BUS SPI

// ---------- LoRa (SX127x-style via RadioLib) ----------
// FCC Part 97 only (Amateur Radio Service). Not legal for unlicensed/uncontrolled operation.
// Firmware must be operated under the supervision of a licensed control operator.
constexpr uint8_t LORA_CS   = 10;
constexpr uint8_t LORA_DIO0 = 9;
constexpr uint8_t LORA_RST  = 8;
constexpr uint8_t LORA_BUSY = 255; // unused on SX127x
constexpr float   LORA_FREQ_MHZ = 433.0f; // 70 cm amateur band (420-450 MHz in the US)

// Operator identification: embed callsign in clear, human-decodable ASCII in every telemetry frame.
// Set these before enabling TX.
constexpr const char LORA_CALLSIGN[] = "CALLSIGN";
#define LORA_CONTROL_OPERATOR_OK 0

// RF parameter discipline: choose minimum BW/SF/power that meets link budget.
constexpr uint8_t LORA_SF = 9;            // 6..12
constexpr float   LORA_BW_KHZ = 125.0f;   // one of standard LoRa BWs; validated at startup
constexpr uint8_t LORA_CR = 5;            // 5..8 => 4/5..4/8
constexpr int8_t  LORA_TX_POWER_DBM = 10; // conservative default; avoid max/PA_BOOST unless required

// Deterministic scheduling + duty-cycle restraint.
constexpr uint32_t LORA_MIN_TX_INTERVAL_MS = (1000UL / LORA_HZ);
constexpr uint32_t LORA_HEARTBEAT_MS = 30000; // max silence between ID-bearing telemetry (0 disables)

// Retries: bounded, with backoff; fail toward silence.
constexpr uint8_t  LORA_RETRY_LIMIT = 2;
constexpr uint32_t LORA_RETRY_BASE_MS = 250;
constexpr uint8_t  LORA_MAX_CONSEC_TX_FAILS = 3;

// Meaningful-change thresholds (duty-cycle restraint).
constexpr int32_t  LORA_MEANINGFUL_PRESS_DELTA_PA_X10 = 100;   // 10 Pa
constexpr int16_t  LORA_MEANINGFUL_TEMP_DELTA_C_X100  = 50;    // 0.50 C

// Hard failsafes.
constexpr uint32_t LORA_TX_WATCHDOG_MS = 5000;
constexpr uint32_t LORA_MAX_MISSION_TX_MS = 20UL * 60UL * 1000UL;
constexpr uint32_t LORA_LANDING_STABLE_MS = 30000;
constexpr int32_t  LORA_LANDING_PRESS_STABLE_DELTA_PA_X10 = 50;
constexpr int32_t  LORA_FLIGHT_PRESS_DELTA_PA_X10 = 5000;

// TX is disabled by default on boot. Set to 1 only when intentional and supervised.
#define LORA_TX_ENABLE_AT_BOOT 0

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
