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
  bool haveAltHistory_ = false;
  uint32_t lastAltSampleTms_ = 0;
  float lastBaroAltM_ = NAN;
  float lastGpsAltM_ = NAN;
  bool havePacketCount_ = false;
  uint8_t lastPacketCount_ = 0;

  void applyTelemetry(const companion_proto::TelemetryV1& t,
                      bool hasTxPower,
                      bool hasRecoveryEvents,
                      CompanionState& ioState);
  void updateDerivedVerticalSpeeds(uint32_t sampleTms, uint8_t packetCount, CompanionState& ioState);
  void debugTick();
};
