#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

#include "../model/telemetry_state.h"
#include "uart_proto.h"

class UartLink {
 public:
  UartLink(HardwareSerial& serial, uint32_t baud, int rxPin, int txPin);

  void begin();
  bool poll(CompanionState& ioState);
  bool sendCommand(const String& action, int durationS = 0);

 private:
  HardwareSerial& serial_;
  uint32_t baud_;
  int rxPin_;
  int txPin_;
  uint16_t txSeq_ = 0;
  companion_proto::FrameParser parser_;
  uint32_t rxFrames_ = 0;
  uint32_t txFrames_ = 0;
  uint32_t lastDbgMs_ = 0;

  void applyTelemetry(const companion_proto::TelemetryV1& t, CompanionState& ioState);
  void debugTick();
};
