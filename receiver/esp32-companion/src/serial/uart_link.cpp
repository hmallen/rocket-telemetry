#include "uart_link.h"

#include <math.h>
#include <string.h>

#include "config.h"

using namespace companion_proto;

UartLink::UartLink(HardwareSerial& serial, uint32_t baud, int rxPin, int txPin)
    : serial_(serial), baud_(baud), rxPin_(rxPin), txPin_(txPin) {}

void UartLink::begin() { serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_); }

void UartLink::applyTelemetry(const TelemetryV1& t, CompanionState& ioState) {
  ioState.tsMs = millis();
  ioState.seq++;

  ioState.link.connected = (t.flags & 0x01) != 0;
  ioState.link.rssi = t.rssi_dbm;
  ioState.link.snr = static_cast<float>(t.snr_db_x4) / 4.0f;
  ioState.link.lastPacketAgeMs = 0;

  ioState.flight.phase = String(phaseToText(t.phase));
  ioState.flight.packetCount = t.packet_count_lsb;

  ioState.alt.altitudeAglM = static_cast<float>(t.alt_mm) / 1000.0f;
  ioState.alt.verticalSpeedMps = static_cast<float>(t.vs_cms) / 100.0f;

  ioState.battery.vbatV = static_cast<float>(t.vbat_mv) / 1000.0f;
  ioState.battery.label = ioState.battery.vbatV < 3.5f ? "LOW" : "OK";

  ioState.stale = false;
}

bool UartLink::poll(CompanionState& ioState) {
  bool updated = false;
  while (serial_.available() > 0) {
    uint8_t b = static_cast<uint8_t>(serial_.read());
    Frame frame;
    if (!parser_.feed(b, frame)) continue;

    if (frame.type == MSG_TELEM_SNAPSHOT && frame.len >= sizeof(TelemetryV1)) {
      TelemetryV1 t{};
      memcpy(&t, frame.payload, sizeof(TelemetryV1));
      applyTelemetry(t, ioState);
      updated = true;
      rxFrames_++;
    } else if (frame.type == MSG_ALERT_EVENT && frame.len >= sizeof(AlertV1)) {
      AlertV1 a{};
      memcpy(&a, frame.payload, sizeof(AlertV1));
      ioState.primaryAlert = "ALERT " + String(a.code);
      updated = true;
      rxFrames_++;
    } else if (frame.type == MSG_CMD_ACK && frame.len >= sizeof(CmdAckV1)) {
      CmdAckV1 ack{};
      memcpy(&ack, frame.payload, sizeof(CmdAckV1));
      ioState.primaryAlert = ack.ok ? "CMD OK" : "CMD FAIL";
      updated = true;
      rxFrames_++;
    }
  }
  debugTick();
  return updated;
}

bool UartLink::sendCommand(const String& action, int durationS) {
  CmdV1 cmd{};
  if (action == "sd_start") {
    cmd.cmd = CMD_SD_START;
    cmd.arg = 0;
  } else if (action == "sd_stop") {
    cmd.cmd = CMD_SD_STOP;
    cmd.arg = 0;
  } else if (action == "buzzer") {
    cmd.cmd = CMD_BUZZER;
    cmd.arg = static_cast<uint8_t>(durationS);
  } else {
    return false;
  }

  Frame f{};
  f.version = VERSION;
  f.type = MSG_CMD;
  f.seq = ++txSeq_;
  f.len = sizeof(CmdV1);
  memcpy(f.payload, &cmd, sizeof(CmdV1));

  uint8_t out[2 + 1 + 1 + 2 + 2 + MAX_PAYLOAD + 2];
  size_t n = encodeFrame(f, out, sizeof(out));
  if (n == 0) return false;
  bool ok = serial_.write(out, n) == n;
  if (ok) txFrames_++;
  debugTick();
  return ok;
}

void UartLink::debugTick() {
#if COMPANION_UART_DEBUG
#if (COMPANION_LINK_UART && (UART_RX_PIN == 3) && (UART_TX_PIN == 1))
  // UART0 is being used as the binary companion link; don't print debug logs here.
  return;
#endif
  uint32_t now = millis();
  if (now - lastDbgMs_ < 2000) return;
  Serial.printf("[UART] rx_frames=%lu tx_frames=%lu\n", (unsigned long)rxFrames_, (unsigned long)txFrames_);
  lastDbgMs_ = now;
#endif
}
