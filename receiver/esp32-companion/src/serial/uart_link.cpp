#include "uart_link.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "config.h"

using namespace companion_proto;

namespace {

String parseCallsignFromTail(const uint8_t* payload, size_t payloadLen, size_t baseLen) {
  if (payload == nullptr || payloadLen <= baseLen) {
    return String();
  }

  const size_t tailLen = payloadLen - baseLen;
  char buf[25];
  const size_t copyLen = (tailLen < (sizeof(buf) - 1)) ? tailLen : (sizeof(buf) - 1);
  memcpy(buf, payload + baseLen, copyLen);
  buf[copyLen] = '\0';

  String callsign(buf);
  callsign.trim();
  return callsign;
}

}  // namespace

UartLink::UartLink(HardwareSerial& serial, uint32_t baud, int rxPin, int txPin)
    : serial_(serial), baud_(baud), rxPin_(rxPin), txPin_(txPin) {}

void UartLink::updateDerivedVerticalSpeeds(uint32_t sampleTms,
                                           uint16_t packetCount,
                                           CompanionState& ioState) {
  const bool hasNewPacket = !havePacketCount_ || packetCount != lastPacketCount_;
  if (!hasNewPacket) {
    return;
  }

  havePacketCount_ = true;
  lastPacketCount_ = packetCount;
  ioState.alt.baroVerticalSpeedMps = NAN;
  ioState.alt.gpsVerticalSpeedMps = NAN;

  if (haveAltHistory_) {
    const uint32_t dtMs = sampleTms - lastAltSampleTms_;
    if (dtMs > 0 && dtMs <= 10000) {
      const float dtS = static_cast<float>(dtMs) / 1000.0f;
      if (!isnan(ioState.alt.altitudeAglM) && !isnan(lastBaroAltM_)) {
        ioState.alt.baroVerticalSpeedMps = (ioState.alt.altitudeAglM - lastBaroAltM_) / dtS;
      }
      if (!isnan(ioState.alt.gpsAltitudeM) && !isnan(lastGpsAltM_)) {
        ioState.alt.gpsVerticalSpeedMps = (ioState.alt.gpsAltitudeM - lastGpsAltM_) / dtS;
      }
    }
  }

  haveAltHistory_ = true;
  lastAltSampleTms_ = sampleTms;
  lastBaroAltM_ = ioState.alt.altitudeAglM;
  lastGpsAltM_ = ioState.alt.gpsAltitudeM;
}

void UartLink::begin() {
  if (&serial_ == &Serial) {
    // UART0 is fixed on GPIO3/GPIO1; do not pass remap pins.
    serial_.begin(baud_);
    return;
  }
  serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
}

void UartLink::applyTelemetry(const TelemetryV1& t,
                              bool hasTxPower,
                              bool hasRecoveryEvents,
                              bool hasGpsQuality,
                              CompanionState& ioState) {
  ioState.tsMs = millis();
  ioState.seq++;

  ioState.link.connected = (t.flags & 0x01) != 0;
  ioState.link.rssi = t.rssi_dbm;
  ioState.link.snr = static_cast<float>(t.snr_db_x4) / 4.0f;
  ioState.link.lastPacketAgeMs = 0;

  ioState.flight.phase = String(phaseToText(t.phase));
  ioState.flight.packetCount = t.packet_count_lsb;

  ioState.alt.altitudeAglM = static_cast<float>(t.alt_mm) / 1000.0f;
  ioState.alt.gpsAltitudeM = (t.gps_alt_mm == INT32_MIN) ? NAN : (static_cast<float>(t.gps_alt_mm) / 1000.0f);
  ioState.alt.verticalSpeedMps = static_cast<float>(t.vs_cms) / 100.0f;
  updateDerivedVerticalSpeeds(t.t_ms, t.packet_count_lsb, ioState);

  ioState.battery.telemetryVbatV = static_cast<float>(t.vbat_mv) / 1000.0f;
  ioState.battery.groundVbatV =
      (t.ground_vbat_mv == 0) ? NAN : (static_cast<float>(t.ground_vbat_mv) / 1000.0f);
  ioState.battery.label = ioState.battery.telemetryVbatV < 3.5f ? "LOW" : "OK";

  ioState.hasSdLoggingState = true;
  ioState.sdLoggingEnabled = (t.flags & 0x08) != 0;
  ioState.hasTelemetryTxState = true;
  ioState.telemetryTxEnabled = (t.flags & 0x10) != 0;
  const bool txPowerValid = hasTxPower && t.telemetry_tx_power_dbm >= 2 && t.telemetry_tx_power_dbm <= 17;
  ioState.hasTelemetryTxPowerState = txPowerValid;
  ioState.telemetryTxPowerDbm = txPowerValid ? t.telemetry_tx_power_dbm : 0;
  ioState.hasCommandLockoutState = true;
  ioState.commandLockoutActive = (t.flags & 0x20) != 0;
  ioState.hasRecoveryDeploymentState = true;
  if (hasRecoveryEvents) {
    const uint8_t events = t.recovery_event_flags;
    ioState.hasRecoveryEventState = true;
    ioState.recoverySensorsCalibrated = (events & 0x01) != 0;
    ioState.recoveryGpsFix3d = (events & 0x02) != 0;
    ioState.recoveryLaunchArmed = (events & 0x04) != 0;
    ioState.recoveryLaunchDetected = (events & 0x08) != 0;
    ioState.recoveryApogee = (events & 0x10) != 0;
    ioState.recoveryDrogueDeployed = (events & 0x20) != 0;
    ioState.recoveryMainDeployed = (events & 0x40) != 0;
    ioState.recoveryLandingDetected = (events & 0x80) != 0;
  } else {
    ioState.hasRecoveryEventState = false;
    ioState.recoverySensorsCalibrated = false;
    ioState.recoveryLaunchDetected = false;
    ioState.recoveryApogee = false;
    ioState.recoveryLandingDetected = false;
    ioState.recoveryDrogueDeployed = (t.flags & 0x02) != 0;
    ioState.recoveryMainDeployed = (t.flags & 0x04) != 0;
    ioState.recoveryLaunchArmed = (t.flags & 0x40) != 0;
    ioState.recoveryGpsFix3d = (t.flags & 0x80) != 0;
  }

  const bool gpsSvsValid = hasGpsQuality && t.gps_svs_used != 0xFF && t.gps_svs_total != 0xFF;
  ioState.hasGpsSvsState = gpsSvsValid;
  ioState.gpsSvsUsed = gpsSvsValid ? t.gps_svs_used : 0;
  ioState.gpsSvsTotal = gpsSvsValid ? t.gps_svs_total : 0;

  const bool gpsHdopValid = hasGpsQuality && t.gps_hdop_x100 != 0xFFFF;
  ioState.hasGpsHdopState = gpsHdopValid;
  ioState.gpsHdop = gpsHdopValid ? (static_cast<float>(t.gps_hdop_x100) / 100.0f) : NAN;

  ioState.stale = false;
}

bool UartLink::poll(CompanionState& ioState) {
  static constexpr size_t kTelemetryV1NoGpsQualityLen = offsetof(TelemetryV1, gps_svs_used);
  static constexpr size_t kTelemetryV1NoEventFlagsLen = offsetof(TelemetryV1, recovery_event_flags);
  static constexpr size_t kTelemetryV1NoTxPowerOrEventsLen = offsetof(TelemetryV1, telemetry_tx_power_dbm);
  bool updated = false;
  while (serial_.available() > 0) {
    uint8_t b = static_cast<uint8_t>(serial_.read());
    Frame frame;
    if (!parser_.feed(b, frame)) continue;

    if (frame.type == MSG_TELEM_SNAPSHOT) {
      if (frame.len >= sizeof(TelemetryV1)) {
        TelemetryV1 t{};
        memcpy(&t, frame.payload, sizeof(TelemetryV1));
        applyTelemetry(t, true, true, true, ioState);
        ioState.flight.callsign =
            parseCallsignFromTail(frame.payload, frame.len, sizeof(TelemetryV1));
        updated = true;
        rxFrames_++;
      } else if (frame.len >= kTelemetryV1NoGpsQualityLen) {
        TelemetryV1 t{};
        memcpy(&t, frame.payload, kTelemetryV1NoGpsQualityLen);
        applyTelemetry(t, true, true, false, ioState);
        ioState.flight.callsign =
            parseCallsignFromTail(frame.payload, frame.len, kTelemetryV1NoGpsQualityLen);
        updated = true;
        rxFrames_++;
      } else if (frame.len >= kTelemetryV1NoEventFlagsLen) {
        TelemetryV1 t{};
        memcpy(&t, frame.payload, kTelemetryV1NoEventFlagsLen);
        applyTelemetry(t, true, false, false, ioState);
        ioState.flight.callsign =
            parseCallsignFromTail(frame.payload, frame.len, kTelemetryV1NoEventFlagsLen);
        updated = true;
        rxFrames_++;
      } else if (frame.len >= kTelemetryV1NoTxPowerOrEventsLen) {
        TelemetryV1 t{};
        memcpy(&t, frame.payload, kTelemetryV1NoTxPowerOrEventsLen);
        applyTelemetry(t, false, false, false, ioState);
        ioState.flight.callsign =
            parseCallsignFromTail(frame.payload, frame.len, kTelemetryV1NoTxPowerOrEventsLen);
        updated = true;
        rxFrames_++;
      } else if (frame.len >= sizeof(TelemetryV1Legacy)) {
        TelemetryV1Legacy t{};
        memcpy(&t, frame.payload, sizeof(TelemetryV1Legacy));

        ioState.tsMs = millis();
        ioState.seq++;

        ioState.link.connected = (t.flags & 0x01) != 0;
        ioState.link.rssi = t.rssi_dbm;
        ioState.link.snr = static_cast<float>(t.snr_db_x4) / 4.0f;
        ioState.link.lastPacketAgeMs = 0;

        ioState.flight.phase = String(phaseToText(t.phase));
        ioState.flight.packetCount = t.packet_count_lsb;
        ioState.flight.callsign =
            parseCallsignFromTail(frame.payload, frame.len, sizeof(TelemetryV1Legacy));

        ioState.alt.altitudeAglM = static_cast<float>(t.alt_mm) / 1000.0f;
        ioState.alt.gpsAltitudeM = NAN;
        ioState.alt.verticalSpeedMps = static_cast<float>(t.vs_cms) / 100.0f;
        updateDerivedVerticalSpeeds(t.t_ms, t.packet_count_lsb, ioState);

        ioState.battery.telemetryVbatV = static_cast<float>(t.vbat_mv) / 1000.0f;
        ioState.battery.groundVbatV =
            (t.ground_vbat_mv == 0) ? NAN : (static_cast<float>(t.ground_vbat_mv) / 1000.0f);
        ioState.battery.label = ioState.battery.telemetryVbatV < 3.5f ? "LOW" : "OK";

        ioState.hasSdLoggingState = true;
        ioState.sdLoggingEnabled = (t.flags & 0x08) != 0;
        ioState.hasTelemetryTxState = true;
        ioState.telemetryTxEnabled = (t.flags & 0x10) != 0;
        ioState.hasTelemetryTxPowerState = false;
        ioState.telemetryTxPowerDbm = 0;
        ioState.hasCommandLockoutState = true;
        ioState.commandLockoutActive = (t.flags & 0x20) != 0;
        ioState.hasRecoveryDeploymentState = true;
        ioState.recoveryDrogueDeployed = (t.flags & 0x02) != 0;
        ioState.recoveryMainDeployed = (t.flags & 0x04) != 0;
        ioState.hasRecoveryEventState = false;
        ioState.recoverySensorsCalibrated = false;
        ioState.recoveryLaunchArmed = false;
        ioState.recoveryGpsFix3d = false;
        ioState.hasGpsSvsState = false;
        ioState.gpsSvsUsed = 0;
        ioState.gpsSvsTotal = 0;
        ioState.hasGpsHdopState = false;
        ioState.gpsHdop = NAN;
        ioState.recoveryLaunchDetected = false;
        ioState.recoveryApogee = false;
        ioState.recoveryLandingDetected = false;

        ioState.stale = false;
        updated = true;
        rxFrames_++;
      }
    } else if (frame.type == MSG_ALERT_EVENT && frame.len >= sizeof(AlertV1)) {
      AlertV1 a{};
      memcpy(&a, frame.payload, sizeof(AlertV1));
      ioState.primaryAlert = "ALERT " + String(a.code);
      updated = true;
      rxFrames_++;
    } else if (frame.type == MSG_CMD_ACK && frame.len >= sizeof(CmdAckV1)) {
      CmdAckV1 ack{};
      memcpy(&ack, frame.payload, sizeof(CmdAckV1));

      String detail;
      if (frame.len > sizeof(CmdAckV1)) {
        const size_t rawLen = static_cast<size_t>(frame.len - sizeof(CmdAckV1));
        char detailBuf[128];
        size_t copyLen = rawLen;
        if (copyLen >= sizeof(detailBuf)) {
          copyLen = sizeof(detailBuf) - 1;
        }
        memcpy(detailBuf, frame.payload + sizeof(CmdAckV1), copyLen);
        detailBuf[copyLen] = '\0';
        detail = String(detailBuf);
        detail.trim();
      }

      String cmdText = "CMD";
      if (ack.cmd == CMD_SD_START) {
        cmdText = "SD START";
      } else if (ack.cmd == CMD_SD_STOP) {
        cmdText = "SD STOP";
      } else if (ack.cmd == CMD_SD_ROTATE) {
        cmdText = "SD ROTATE";
      } else if (ack.cmd == CMD_SD_FORMAT) {
        cmdText = "SD FORMAT";
      } else if (ack.cmd == CMD_SD_DUMP_SAMPLE) {
        cmdText = "SD DUMP";
      } else if (ack.cmd == CMD_TELEM_ENABLE) {
        cmdText = "TX ENABLE";
      } else if (ack.cmd == CMD_TELEM_DISABLE) {
        cmdText = "TX DISABLE";
      } else if (ack.cmd == CMD_SET_TX_POWER) {
        cmdText = "TX POWER";
      }

      ioState.tsMs = millis();
      ioState.seq++;

      const bool isSdAck = (ack.cmd == CMD_SD_START || ack.cmd == CMD_SD_STOP ||
                            ack.cmd == CMD_SD_ROTATE || ack.cmd == CMD_SD_FORMAT ||
                            ack.cmd == CMD_SD_DUMP_SAMPLE);
      if (isSdAck) {
        ioState.hasSdCardAckState = true;
        ioState.sdCardAckOk = (ack.ok != 0);
        ioState.sdCardAckToken = ioState.seq;
        if (ack.cmd == CMD_SD_START) {
          ioState.sdCardLastCommand = "sd_start";
        } else if (ack.cmd == CMD_SD_STOP) {
          ioState.sdCardLastCommand = "sd_stop";
        } else if (ack.cmd == CMD_SD_ROTATE) {
          ioState.sdCardLastCommand = "sd_rotate";
        } else if (ack.cmd == CMD_SD_FORMAT) {
          ioState.sdCardLastCommand = "sd_format";
        } else {
          ioState.sdCardLastCommand = "sd_dump_sample";
        }
        ioState.sdCardDetail = detail;
      }

      ioState.primaryAlert = cmdText + String(ack.ok ? " OK" : " FAIL");
      if (detail.length() > 0) {
        ioState.primaryAlert += ": ";
        ioState.primaryAlert += detail;
      }
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
  } else if (action == "sd_rotate") {
    cmd.cmd = CMD_SD_ROTATE;
    cmd.arg = 0;
  } else if (action == "sd_format") {
    cmd.cmd = CMD_SD_FORMAT;
    cmd.arg = 0;
  } else if (action == "sd_dump_sample") {
    cmd.cmd = CMD_SD_DUMP_SAMPLE;
    cmd.arg = 0;
  } else if (action == "buzzer") {
    cmd.cmd = CMD_BUZZER;
    cmd.arg = static_cast<uint8_t>(durationS);
  } else if (action == "telemetry_enable") {
    cmd.cmd = CMD_TELEM_ENABLE;
    cmd.arg = 0;
  } else if (action == "telemetry_disable") {
    cmd.cmd = CMD_TELEM_DISABLE;
    cmd.arg = 0;
  } else if (action == "alt_calibrate") {
    cmd.cmd = CMD_ALT_CALIBRATE;
    cmd.arg = 0;
  } else if (action == "phase_reset") {
    cmd.cmd = CMD_ALT_CALIBRATE;
    cmd.arg = 0;
  } else if (action == "imu_calibrate") {
    cmd.cmd = CMD_IMU_CALIBRATE;
    cmd.arg = 0;
  } else if (action == "shutdown") {
    cmd.cmd = CMD_SHUTDOWN;
    cmd.arg = 0;
  } else if (action == "reboot") {
    cmd.cmd = CMD_REBOOT;
    cmd.arg = 0;
  } else if (action == "telemetry_tx_power") {
    if (durationS < 2 || durationS > 17) {
      return false;
    }
    cmd.cmd = CMD_SET_TX_POWER;
    cmd.arg = static_cast<uint8_t>(durationS);
  } else if (action == "launch_arm") {
    cmd.cmd = CMD_LAUNCH_ARM;
    cmd.arg = (durationS > 0) ? 1 : 0;
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
