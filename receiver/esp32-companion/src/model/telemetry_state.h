#pragma once

#include <Arduino.h>

struct LinkState {
  bool connected = false;
  int rssi = 0;
  float snr = 0.0f;
  int lastPacketAgeMs = -1;
};

struct FlightState {
  String phase;
  uint32_t packetCount = 0;
  String callsign;
};

struct AltState {
  // Altitude above ground level from recovery/barometer solution.
  float altitudeAglM = NAN;
  // Absolute GPS altitude from GNSS solution.
  float gpsAltitudeM = NAN;
  // Recovery vertical speed downlinked from flight computer (cm/s over LoRa, converted to m/s here).
  float verticalSpeedMps = NAN;
  // Companion-derived vertical speed from BARO/AGL altitude delta.
  float baroVerticalSpeedMps = NAN;
  // Companion-derived vertical speed from GPS altitude delta.
  float gpsVerticalSpeedMps = NAN;
};

struct BatteryState {
  // Battery voltage reported by the remote telemetry transmitter over LoRa.
  float telemetryVbatV = NAN;
  // Ground-station battery voltage from Pi ADS1115 (A0 via divider).
  float groundVbatV = NAN;
  // Local companion display board battery voltage sampled from BAT_ADC.
  float companionVbatV = NAN;
  String label;
};

struct CompanionState {
  uint32_t seq = 0;
  uint32_t tsMs = 0;
  LinkState link;
  FlightState flight;
  AltState alt;
  BatteryState battery;
  bool hasSdLoggingState = false;
  bool sdLoggingEnabled = false;
  bool hasSdCardAckState = false;
  bool sdCardAckOk = false;
  uint32_t sdCardAckToken = 0;
  String sdCardLastCommand;
  String sdCardDetail;
  bool hasTelemetryTxState = false;
  bool telemetryTxEnabled = false;
  bool hasTelemetryTxPowerState = false;
  uint8_t telemetryTxPowerDbm = 0;
  bool hasCommandLockoutState = false;
  bool commandLockoutActive = false;
  bool hasRecoveryDeploymentState = false;
  bool recoveryDrogueDeployed = false;
  bool recoveryMainDeployed = false;
  bool hasRecoveryEventState = false;
  bool recoverySensorsCalibrated = false;
  bool recoveryLaunchArmed = false;
  bool recoveryGpsFix3d = false;
  bool recoveryLaunchDetected = false;
  bool recoveryApogee = false;
  bool recoveryLandingDetected = false;
  String primaryAlert;
  bool stale = true;
};
