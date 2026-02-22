# ESP32 Companion Display Test Checklist

Use this checklist before field deployment.

## Bench Test (Required)

- [ ] Build firmware successfully (`pio run`)
- [ ] Flash device (`pio run -t upload`)
- [ ] Device boots with no crash loop (`pio device monitor`)
- [ ] Stale indicator appears after >3s without telemetry
- [ ] `BUZZ 1s` touch command works
- [ ] `BUZZ 5s` touch command works
- [ ] `SD TOGGLE` starts/stops logging as expected
- [ ] Command failures show error feedback on screen
- [ ] Touch calibration mode opens (top-left) and exits (top-right)
- [ ] Mapped touch coordinates align with button hitboxes

## Bench Test: Wi-Fi Mode (`COMPANION_LINK_UART=0`)

- [ ] Serial monitor shows Wi-Fi connect and no reconnect storm
- [ ] `GET /api/companion/state` returns valid JSON from Pi
- [ ] SSE stream connects (`/api/companion/events`) and updates continuously
- [ ] Link status changes correctly when Pi service is stopped/restarted

## Bench Test: UART Mode (`COMPANION_LINK_UART=1`)

- [ ] Wiring verified: Pi TX -> ESP32 RX, Pi RX <- ESP32 TX, GND common
- [ ] Logic level verified: UART signals are 3.3V (no 5V on RX/TX)
- [ ] UART config matches both sides (baud + pins)
- [ ] ESP32 receives telemetry over UART and updates phase/altitude/packet count
- [ ] Pulling one UART wire causes stale/link-down indication within ~3s
- [ ] Reconnecting UART recovers without reboot
- [ ] UART command path works end-to-end (BUZZ/SD toggle + ACK feedback)
- [ ] No framing instability under load (no persistent garbage/lock-up)

## Security & Network

- [ ] `GS_AUTH_TOKEN` is set on ESP32 when command auth is enabled on Pi
- [ ] `/api/companion/cmd` returns 401 without token (when auth enabled)
- [ ] Pi bind/access mode matches expected launch network policy

## UART Robustness (Recommended)

- [ ] 30+ minute run with continuous UART telemetry has no parser lockup
- [ ] Power cycle ESP32 while Pi ground station runs: link recovers automatically
- [ ] Restart/power cycle Pi ground station while ESP32 runs: stale alarm then recovery
- [ ] Cable wiggle test does not trigger false commands
- [ ] Command retry/timeout behavior is acceptable for field use

## Field Simulation (Recommended)

- [ ] Ground station runs for 30+ minutes without UI freeze
- [ ] Reconnect behavior works after Wi-Fi disruption/rejoin
- [ ] Alert strip shows phase transitions and link events
- [ ] Display remains readable in expected daylight conditions
- [ ] No accidental command triggers from noisy touch events

## Pre-Launch

- [ ] Confirm final touch calibration constants in `include/config.h`
- [ ] Confirm selected transport mode (`COMPANION_LINK_UART` 0 or 1)
- [ ] If Wi-Fi mode: confirm correct ground station host/IP and port
- [ ] If UART mode: confirm final UART pins/baud and cable strain relief
- [ ] Confirm buzzer + SD command behavior with live radio link
- [ ] Keep fallback control path available (laptop/web UI)
