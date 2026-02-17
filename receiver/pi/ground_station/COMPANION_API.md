# Companion Display API (ESP32 Touchscreen)

This API is intended for a Pi-companion touchscreen display (ESP32) and is separate from the browser UI API.

## Endpoints

### `GET /api/companion/state`
Returns a lightweight telemetry snapshot suitable for ESP32 rendering.

Response:

```json
{
  "state": {
    "ts": 1771290000.123,
    "seq": 123,
    "link": {
      "connected": true,
      "rssi": -91,
      "snr": 7.25,
      "crc_ok": true,
      "last_packet_age_ms": 84
    },
    "flight": {
      "phase": "coast",
      "packet_count": 2812,
      "callsign": "N0CALL"
    },
    "gps": { "lat_deg": 32.99, "lon_deg": -96.75, "alt_m": 512.3, "svs_used": 10, "svs_total": 18 },
    "alt": { "press_kpa": 98.7, "temp_c": 24.3 },
    "imu": { "ax_g": 0.02, "ay_g": -0.01, "az_g": 1.01, "gx_dps": 0.6, "gy_dps": -0.2, "gz_dps": 0.3 },
    "attitude": { "roll": 2.1, "pitch": -0.8, "yaw": 178.3, "filter": "madgwick" },
    "battery": { "vbat_v": 3.96, "bat_state_label": "OK" },
    "recovery": {
      "enabled": true,
      "phase": "coast",
      "altitude_agl_m": 402.1,
      "max_altitude_agl_m": 410.8,
      "vertical_speed_mps": 92.4,
      "drogue": { "deployed": false },
      "main": { "deployed": false }
    },
    "alerts": []
  }
}
```

### `GET /api/companion/events` (SSE)
SSE stream for live updates.

- `event: telemetry` for normal updates
- `event: hb` for heartbeat keepalive (1s cadence)

Example telemetry payload:

```json
{
  "event": "telemetry",
  "state": { "seq": 124, "link": { "rssi": -93 } }
}
```

### `POST /api/companion/cmd`
Sends a command through the same command path used by `/api/command`.

Auth required with either:
- `Authorization: Bearer <GS_AUTH_TOKEN>`
- `X-Auth-Token: <GS_AUTH_TOKEN>`

Request body examples:

```json
{"action": "sd_start"}
```

```json
{"action": "sd_stop"}
```

```json
{"action": "buzzer", "duration_s": 5}
```

## Notes

- The companion API is intentionally small and stable for microcontroller clients.
- `alerts` includes link staleness and low battery warnings for direct UI signaling.
- Link state is derived from telemetry timestamp freshness.
