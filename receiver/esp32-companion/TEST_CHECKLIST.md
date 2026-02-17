# ESP32 Companion Display Test Checklist

Use this checklist before field deployment.

## Bench Test (Required)

- [ ] Build firmware successfully (`pio run`)
- [ ] Flash device (`pio run -t upload`)
- [ ] Serial monitor shows Wi-Fi connect and no crash loop
- [ ] `GET /api/companion/state` returns valid JSON from Pi
- [ ] SSE stream connects (`/api/companion/events`) and updates continuously
- [ ] Link status changes correctly when Pi service is stopped/restarted
- [ ] Stale indicator appears after >3s without telemetry
- [ ] `BUZZ 1s` touch command works
- [ ] `BUZZ 5s` touch command works
- [ ] `SD TOGGLE` starts/stops logging as expected
- [ ] Command failures show error feedback on screen
- [ ] Touch calibration mode opens (top-left) and exits (top-right)
- [ ] Mapped touch coordinates align with button hitboxes

## Security & Network

- [ ] `GS_AUTH_TOKEN` is set on ESP32 when command auth is enabled on Pi
- [ ] `/api/companion/cmd` returns 401 without token (when auth enabled)
- [ ] Pi bind/access mode matches expected launch network policy

## Field Simulation (Recommended)

- [ ] Ground station runs for 30+ minutes without UI freeze
- [ ] Reconnect behavior works after Wi-Fi disruption/rejoin
- [ ] Alert strip shows phase transitions and link events
- [ ] Display remains readable in expected daylight conditions
- [ ] No accidental command triggers from noisy touch events

## Pre-Launch

- [ ] Confirm final touch calibration constants in `include/config.h`
- [ ] Confirm correct ground station host/IP and port
- [ ] Confirm buzzer + SD command behavior with live radio link
- [ ] Keep fallback control path available (laptop/web UI)
