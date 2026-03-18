# rocket-telemetry

High-rate flight data logger / telemetry firmware for a **Teensy 4.1** (Arduino framework via PlatformIO).

The firmware samples sensors, timestamps data with `micros()`, optionally anchors time to GNSS PPS, buffers records through a fast in-RAM ring (and optional PSRAM “spool”), and can write framed binary blocks to SD (Teensy 4.1 SDIO). A LoRa downlink module exists (enabled by default) and includes **FCC Part 97** guardrails.

## Status / defaults

`src/cfg.h` controls feature toggles. Current defaults in this repo:

- **SD logging**: enabled (`ENABLE_SD_LOGGER 1`)
- **PSRAM spool**: enabled (`ENABLE_PSRAM_SPOOL 1`)
- **GNSS**: enabled (`ENABLE_GNSS 1`) (raw UBX byte logging; configuration is a stub)
- **Sensors**: enabled (`ENABLE_SENSORS 1`)
- **LoRa**: enabled (`ENABLE_LORA 1`, `LORA_CONTROL_OPERATOR_OK 1`)

## Hardware

### Target MCU

- **Teensy 4.1**

### Sensors / peripherals (as implemented)

- **Barometer**: BMP3XX (configured via SPI) and BMP180 (I2C) as `baro2`
- **Gyro**: Adafruit L3GD20 (I2C)
- **Accelerometer**: Adafruit LSM303 (I2C)
- **GNSS**: two UARTs (`Serial1` primary, `Serial3` backup) + PPS input
- **SD**: Teensy 4.1 on-board SDIO (when enabled)
- **LoRa**: SX1276 via RadioLib (when enabled)

### Pin / bus configuration

All pins and bus settings are defined in `src/cfg.h`:

- **GNSS**
  - `GNSS_SERIAL_PRIMARY`: `Serial1`
  - `GNSS_SERIAL_BACKUP`: `Serial3`
  - `GNSS_PPS_PIN`: `2`
  - `GNSS_BAUD`: `115200`
- **Buzzer**: `BUZZER_PIN = 28` (short beep every ~2 seconds)
- **I2C**: `Wire`, `I2C_HZ = 400,000`
- **BMP3XX SPI**
  - `BMP_SPI_BUS = SPI`
  - `BMP_CS = 38`
- **LoRa (SX127x)**
  - `LORA_CS = 10`, `LORA_DIO0 = 9`, `LORA_RST = 8`

## Building / flashing

This is a PlatformIO project.

- **Environments**
  - `teensy41`
  - `teensy41_debug` (adds `-D DEBUG_MODE=1`)

Typical workflow (PlatformIO CLI):

```sh
pio run -e teensy41
pio run -e teensy41 -t upload
pio device monitor -b 115200
```

Serial monitor speed is `115200` (see `platformio.ini`).

## Runtime behavior (high level)

`src/main.cpp` drives the system:

- **Timebase**
  - `micros()` is the primary timestamp.
  - A PPS interrupt latches `t_us_at_pps`; a `REC_TIME_ANCHOR` record is emitted on each PPS edge.
- **GNSS**
  - Raw bytes from UART(s) are packed into `REC_GNSS_CHUNK` records.
  - Basic failover selects the “fresh” receiver based on a timeout (`GNSS_FAILOVER_TIMEOUT_US`).
  - Module configuration is currently a stub in `GnssUbx::configure()`.
- **Sensors**
  - Baro is sampled at `BARO_HZ`.
  - IMU sampling is scheduled at `IMU_HZ`.
- **Buffering**
  - Records are written into an internal `ByteRing` (DMA memory).
  - If enabled, the ring is drained into a PSRAM spool (`PsramSpool`) to absorb bursts.
- **SD block writing**
  - Data is drained into a `BlockBuilder` and written in 32 KiB blocks (when `ENABLE_SD_LOGGER` is enabled).

## Data format

### Record stream

Records are packed structs defined in `src/records.h`, each beginning with:

- `RecHdr { type, ver, len }`

Record types currently used:

- `REC_IMU_FAST` (accel/gyro)
- `REC_BARO`
- `REC_BARO2` (BMP180)
- `REC_GNSS_CHUNK` (raw bytes)
- `REC_TIME_ANCHOR` (PPS-to-GNSS time anchor)
- `REC_EVENT` (markers)
- `REC_STATS` (drop counters, voltages, SD write error counter)

### SD block framing

When SD logging is enabled, data is written as a sequence of blocks:

- `BlockHdr` (see `records.h`)
  - `magic`: `0x424D4C54` (`'TLMB'`)
  - `ver`: `1`
  - `seq`: monotonic block number
  - `t_start_us`: `micros()` at block start
  - `payload_len`: bytes following the header
  - `crc32`: CRC32 of payload
- `payload`: concatenated record bytes

Log files are named `LOG00000.BIN`, `LOG00001.BIN`, ... and are preallocated (see `PREALLOC_BYTES`).

## LoRa telemetry

The LoRa downlink code (`src/lora_link.*`) is intended for **FCC Part 97** amateur telemetry and includes multiple safeguards:

- Transmit is blocked unless you explicitly set `LORA_CONTROL_OPERATOR_OK 1`.
- Transmit is disabled at boot by default (`LORA_TX_ENABLE_AT_BOOT 0`).
- Telemetry payload uses a **custom binary format** and includes the callsign as an identifier byte sequence.

If you enable LoRa, you must also set a real `LORA_CALLSIGN` and ensure you are operating legally and under appropriate supervision.

## Tuning / configuration

Edit `src/cfg.h` for:

- Sample rates (`IMU_HZ`, `BARO_HZ`, `GNSS_HZ`, `LORA_HZ`)
- Buffer sizes (`RING_BYTES`, `SPOOL_BYTES`)
- SD write/sync parameters (`LOG_BLOCK_BYTES`, `SD_SYNC_MS`, `PREALLOC_BYTES`)
- Enabling/disabling features (the `ENABLE_*` macros)


## Receiver Tools

The project includes several ground-side companion tools located in the `receiver/` directory:

- **ESP32 Companion** (`receiver/esp32-companion`): An ESP32-based application using the Arduino framework and LVGL to display live telemetry data received over LoRa.
- **Pi Ground Station** (`receiver/pi`): A Python-based ground station server designed to run on a Raspberry Pi, providing a web interface, logging, and mapping capabilities.
- **Pico Emulator** (`receiver/pico`): A LoRa flight emulator for the Raspberry Pi Pico to generate simulated telemetry data for testing ground station setups.

## License

See `LICENSE`.