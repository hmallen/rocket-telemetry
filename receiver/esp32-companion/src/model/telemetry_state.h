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
  float vbatV = NAN;
  String label;
};

struct CompanionState {
  uint32_t seq = 0;
  uint32_t tsMs = 0;
  LinkState link;
  FlightState flight;
  AltState alt;
  BatteryState battery;
  bool stale = true;
};
