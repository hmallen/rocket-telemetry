#include "api_client.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>

#include "config.h"

ApiClient::ApiClient(const String& host, uint16_t port) : host_(host), port_(port) {}

bool ApiClient::fetchInitialState(CompanionState& outState) {
  HTTPClient http;
  String url = "http://" + host_ + ":" + String(port_) + "/api/companion/state";
  if (!http.begin(url)) {
    return false;
  }

  int code = http.GET();
  if (code != 200) {
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();
  bool ok = applyStateJson(payload, outState);
  if (ok) {
    markLastRx();
  }
  return ok;
}

bool ApiClient::openEventStream() {
  closeEventStream();

  if (!sseClient_.connect(host_.c_str(), port_)) {
    return false;
  }

  sseClient_.print("GET /api/companion/events HTTP/1.1\r\n");
  sseClient_.print("Host: " + host_ + "\r\n");
  sseClient_.print("Accept: text/event-stream\r\n");
  sseClient_.print("Cache-Control: no-cache\r\n");
  sseClient_.print("Connection: keep-alive\r\n\r\n");

  // Skip HTTP headers.
  uint32_t start = millis();
  String line;
  while (millis() - start < 3000) {
    if (!sseClient_.connected()) return false;
    if (!sseClient_.available()) {
      delay(2);
      continue;
    }
    char c = (char)sseClient_.read();
    line += c;
    if (line.endsWith("\r\n\r\n")) {
      lineBuffer_ = "";
      eventType_ = "";
      dataBuffer_ = "";
      return true;
    }
  }
  return false;
}

bool ApiClient::pollEventStream(CompanionState& ioState) {
  if (!sseClient_.connected()) return false;

  bool gotAny = false;
  while (sseClient_.available()) {
    char c = (char)sseClient_.read();
    if (c == '\r') continue;

    if (c != '\n') {
      lineBuffer_ += c;
      continue;
    }

    // End of line.
    String line = lineBuffer_;
    lineBuffer_ = "";

    if (line.length() == 0) {
      // End of event block.
      if (dataBuffer_.length() > 0 && (eventType_ == "telemetry" || eventType_.length() == 0)) {
        if (applyStateJson(dataBuffer_, ioState)) {
          markLastRx();
          gotAny = true;
        }
      }
      eventType_ = "";
      dataBuffer_ = "";
      continue;
    }

    if (line.startsWith("event:")) {
      eventType_ = line.substring(6);
      eventType_.trim();
      continue;
    }

    if (line.startsWith("data:")) {
      String part = line.substring(5);
      part.trim();
      if (dataBuffer_.length() > 0) dataBuffer_ += "\n";
      dataBuffer_ += part;
      continue;
    }
  }

  return gotAny;
}

void ApiClient::closeEventStream() {
  if (sseClient_.connected()) {
    sseClient_.stop();
  }
  lineBuffer_ = "";
  eventType_ = "";
  dataBuffer_ = "";
}

void ApiClient::markLastRx() { lastRxMs_ = millis(); }

bool ApiClient::sendCommand(const String& action, int durationS) {
  HTTPClient http;
  String url = "http://" + host_ + ":" + String(port_) + "/api/companion/cmd";
  if (!http.begin(url)) {
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  if (strlen(GS_AUTH_TOKEN) > 0) {
    http.addHeader("Authorization", "Bearer " + String(GS_AUTH_TOKEN));
  }

  JsonDocument doc;
  doc["action"] = action;
  if (action == "buzzer") {
    doc["duration_s"] = durationS;
  }

  String body;
  serializeJson(doc, body);
  int code = http.POST(body);
  http.end();
  return code == 200;
}

bool ApiClient::applyStateJson(const String& jsonPayload, CompanionState& ioState) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, jsonPayload);
  if (err) return false;

  JsonObject state = doc["state"].is<JsonObject>() ? doc["state"].as<JsonObject>() : JsonObject();
  if (state.isNull()) return false;

  ioState.seq = state["seq"] | ioState.seq;
  ioState.tsMs = millis();

  JsonObject link = state["link"];
  ioState.link.connected = link["connected"] | false;
  ioState.link.rssi = link["rssi"] | 0;
  ioState.link.snr = link["snr"] | 0.0f;
  ioState.link.lastPacketAgeMs = link["last_packet_age_ms"] | -1;

  JsonObject flight = state["flight"];
  ioState.flight.phase = String((const char*)(flight["phase"] | "unknown"));
  ioState.flight.packetCount = flight["packet_count"] | 0;
  ioState.flight.callsign = String((const char*)(flight["callsign"] | ""));

  JsonObject recovery = state["recovery"];
  ioState.alt.altitudeAglM = recovery["altitude_agl_m"].isNull() ? NAN : (float)recovery["altitude_agl_m"].as<float>();
  ioState.alt.verticalSpeedMps = recovery["vertical_speed_mps"].isNull() ? NAN : (float)recovery["vertical_speed_mps"].as<float>();

  JsonObject battery = state["battery"];
  ioState.battery.vbatV = battery["vbat_v"].isNull() ? NAN : (float)battery["vbat_v"].as<float>();
  ioState.battery.label = String((const char*)(battery["bat_state_label"] | ""));

  ioState.stale = false;
  return true;
}
