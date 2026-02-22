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
  float altitudeAglM = NAN;
  float verticalSpeedMps = NAN;
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
  bool hasTelemetryTxState = false;
  bool telemetryTxEnabled = false;
  String primaryAlert;
  bool stale = true;
};
