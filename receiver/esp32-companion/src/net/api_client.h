#pragma once

#include <Arduino.h>
#include <WiFiClient.h>
#include "../model/telemetry_state.h"

class ApiClient {
 public:
  ApiClient(const String& host, uint16_t port);

  bool fetchInitialState(CompanionState& outState);
  bool openEventStream();
  bool pollEventStream(CompanionState& ioState);
  void closeEventStream();
  bool sendCommand(const String& action, int durationS = 0);

 private:
  bool applyStateJson(const String& jsonPayload, CompanionState& ioState);
  void markLastRx();

  String host_;
  uint16_t port_;
  WiFiClient sseClient_;
  String lineBuffer_;
  String eventType_;
  String dataBuffer_;
  uint32_t lastRxMs_ = 0;
};
