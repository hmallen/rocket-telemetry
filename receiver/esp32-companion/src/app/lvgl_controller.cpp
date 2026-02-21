#include "lvgl_controller.h"

#include <esp_heap_caps.h>
#include <math.h>

namespace {

constexpr uint32_t kUiRefreshIntervalMs = 120;
constexpr uint32_t kCommandStatusShowMs = 3000;
constexpr uint8_t kCalPointCount = 4;
constexpr int kCalMarginPx = 24;

#ifndef COMPANION_BAT_ADC_PIN
#define COMPANION_BAT_ADC_PIN 34
#endif

#ifndef COMPANION_BAT_ADC_DIVIDER_SCALE
#define COMPANION_BAT_ADC_DIVIDER_SCALE 2.0f
#endif

#ifndef COMPANION_BAT_ADC_CAL_SCALE
#define COMPANION_BAT_ADC_CAL_SCALE 1.0f
#endif

#ifndef COMPANION_BAT_ADC_SAMPLES
#define COMPANION_BAT_ADC_SAMPLES 16
#endif

#ifndef COMPANION_BAT_SAMPLE_INTERVAL_MS
#define COMPANION_BAT_SAMPLE_INTERVAL_MS 1000
#endif

constexpr uint32_t kBatterySampleIntervalMs = COMPANION_BAT_SAMPLE_INTERVAL_MS;

constexpr float kCompanionBatAdcRefV = 3.3f;
constexpr float kCompanionBatAdcMaxCounts = 4095.0f;

static HardwareSerial& companionUartPort() {
#if (UART_RX_PIN == 3) && (UART_TX_PIN == 1)
  return Serial;
#else
  return Serial2;
#endif
}

static String formatFloat(float value, uint8_t decimals, const char* fallback = "---") {
  if (isnan(value)) {
    return String(fallback);
  }
  return String(value, static_cast<unsigned int>(decimals));
}

static lv_obj_t* makeActionButton(lv_obj_t* parent, const char* text, lv_event_cb_t cb, void* userData) {
  lv_obj_t* btn = lv_btn_create(parent);
  lv_obj_set_width(btn, LV_PCT(100));
  lv_obj_set_height(btn, 34);
  lv_obj_set_style_radius(btn, 8, 0);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x1f2a3b), 0);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x2d4f86), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(btn, lv_color_hex(0x4b7dd1), 0);
  lv_obj_set_style_border_width(btn, 1, 0);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, userData);

  lv_obj_t* label = lv_label_create(btn);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_color(label, lv_color_hex(0xeaf1ff), 0);
  lv_obj_center(label);
  return btn;
}

}  // namespace

LvglController::LvglController(TFT_eSPI& tft, const String& host, uint16_t port)
    : api_(host, port),
      uart_(companionUartPort(), UART_BAUD, UART_RX_PIN, UART_TX_PIN),
      tft_(tft),
      touch_(TOUCH_CS_PIN, TOUCH_IRQ_PIN) {}

void LvglController::initLvgl() {
  lv_init();

  const size_t pixelCount = static_cast<size_t>(kScreenWidth) * kDrawBufferLines;
  drawBufPixels_ = static_cast<lv_color_t*>(
      heap_caps_malloc(pixelCount * sizeof(lv_color_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (drawBufPixels_ == nullptr) {
    drawBufPixels_ = static_cast<lv_color_t*>(malloc(pixelCount * sizeof(lv_color_t)));
  }

  if (drawBufPixels_ == nullptr) {
    while (true) {
      delay(1000);
    }
  }

  lv_disp_draw_buf_init(&drawBuf_, drawBufPixels_, nullptr, pixelCount);

  lv_disp_drv_init(&dispDrv_);
  dispDrv_.hor_res = kScreenWidth;
  dispDrv_.ver_res = kScreenHeight;
  dispDrv_.flush_cb = flushDisplayCb;
  dispDrv_.draw_buf = &drawBuf_;
  dispDrv_.user_data = this;
  lv_disp_drv_register(&dispDrv_);

  lv_indev_drv_init(&indevDrv_);
  indevDrv_.type = LV_INDEV_TYPE_POINTER;
  indevDrv_.read_cb = readTouchCb;
  indevDrv_.user_data = this;
  touchIndev_ = lv_indev_drv_register(&indevDrv_);
  (void)touchIndev_;
}

void LvglController::buildUi() {
  lv_obj_t* screen = lv_scr_act();
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x050a14), 0);
  lv_obj_set_style_bg_grad_color(screen, lv_color_hex(0x0d1626), 0);
  lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_VER, 0);

  root_ = lv_obj_create(screen);
  lv_obj_set_size(root_, LV_PCT(100), LV_PCT(100));
  lv_obj_set_style_bg_opa(root_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(root_, 0, 0);
  lv_obj_set_style_pad_all(root_, 8, 0);
  lv_obj_set_style_pad_gap(root_, 8, 0);
  lv_obj_set_flex_flow(root_, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(root_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_clear_flag(root_, LV_OBJ_FLAG_SCROLLABLE);

  telemetryPanel_ = lv_obj_create(root_);
  lv_obj_set_height(telemetryPanel_, LV_PCT(100));
  lv_obj_set_flex_grow(telemetryPanel_, 1);
  lv_obj_set_style_bg_color(telemetryPanel_, lv_color_hex(0x0b1322), 0);
  lv_obj_set_style_border_color(telemetryPanel_, lv_color_hex(0x203457), 0);
  lv_obj_set_style_border_width(telemetryPanel_, 1, 0);
  lv_obj_set_style_radius(telemetryPanel_, 12, 0);
  lv_obj_set_style_pad_all(telemetryPanel_, 12, 0);
  lv_obj_clear_flag(telemetryPanel_, LV_OBJ_FLAG_SCROLLABLE);

  linkLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_align(linkLabel_, LV_ALIGN_TOP_LEFT, 0, 0);

  linkMetaLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(linkMetaLabel_, lv_color_hex(0xa5b4cf), 0);
  lv_obj_align(linkMetaLabel_, LV_ALIGN_TOP_LEFT, 0, 32);

  phaseLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(phaseLabel_, lv_color_hex(0x8ed0ff), 0);
  lv_obj_align(phaseLabel_, LV_ALIGN_TOP_LEFT, 0, 64);

  altitudeLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(altitudeLabel_, lv_color_hex(0xffde6a), 0);
  lv_obj_align(altitudeLabel_, LV_ALIGN_TOP_LEFT, 0, 96);

  vsLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(vsLabel_, lv_color_hex(0x9df5b3), 0);
  lv_obj_align(vsLabel_, LV_ALIGN_TOP_LEFT, 0, 150);

  packetLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(packetLabel_, lv_color_hex(0xd9e3f8), 0);
  lv_obj_align(packetLabel_, LV_ALIGN_BOTTOM_LEFT, 0, -68);

  callsignLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(callsignLabel_, lv_color_hex(0xd9e3f8), 0);
  lv_obj_align(callsignLabel_, LV_ALIGN_BOTTOM_LEFT, 0, -48);

  batteryLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(batteryLabel_, lv_color_hex(0xd9e3f8), 0);
  lv_obj_align(batteryLabel_, LV_ALIGN_BOTTOM_LEFT, 0, -28);

  companionBatteryLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(companionBatteryLabel_, lv_color_hex(0xd9e3f8), 0);
  lv_obj_align(companionBatteryLabel_, LV_ALIGN_BOTTOM_LEFT, 0, -8);

  cmdStatusLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(cmdStatusLabel_, lv_color_hex(0x6be7a4), 0);
  lv_obj_align(cmdStatusLabel_, LV_ALIGN_BOTTOM_RIGHT, 0, -36);

  alertLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(alertLabel_, lv_color_hex(0xff8181), 0);
  lv_obj_align(alertLabel_, LV_ALIGN_BOTTOM_RIGHT, 0, -8);

  panelQuickToggle_ = lv_btn_create(telemetryPanel_);
  lv_obj_set_size(panelQuickToggle_, 44, 34);
  lv_obj_set_style_radius(panelQuickToggle_, 10, 0);
  lv_obj_set_style_bg_color(panelQuickToggle_, lv_color_hex(0x244374), 0);
  lv_obj_set_style_bg_color(panelQuickToggle_, lv_color_hex(0x2e5ca0), LV_STATE_PRESSED);
  lv_obj_align(panelQuickToggle_, LV_ALIGN_RIGHT_MID, -6, 0);
  lv_obj_add_event_cb(panelQuickToggle_, onPanelToggleEvent, LV_EVENT_CLICKED, this);
  panelQuickToggleLabel_ = lv_label_create(panelQuickToggle_);
  lv_label_set_text(panelQuickToggleLabel_, ">>");
  lv_obj_center(panelQuickToggleLabel_);
  lv_obj_add_flag(panelQuickToggle_, LV_OBJ_FLAG_HIDDEN);

  actionPanel_ = lv_obj_create(root_);
  lv_obj_set_size(actionPanel_, kActionPanelWidth, LV_PCT(100));
  lv_obj_set_style_bg_color(actionPanel_, lv_color_hex(0x111c2e), 0);
  lv_obj_set_style_border_color(actionPanel_, lv_color_hex(0x2a446a), 0);
  lv_obj_set_style_border_width(actionPanel_, 1, 0);
  lv_obj_set_style_radius(actionPanel_, 12, 0);
  lv_obj_set_style_pad_all(actionPanel_, 8, 0);
  lv_obj_set_style_pad_gap(actionPanel_, 8, 0);
  lv_obj_set_flex_flow(actionPanel_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(actionPanel_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_clear_flag(actionPanel_, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* panelHeader = lv_obj_create(actionPanel_);
  lv_obj_set_width(panelHeader, LV_PCT(100));
  lv_obj_set_height(panelHeader, 34);
  lv_obj_set_style_bg_color(panelHeader, lv_color_hex(0x18253f), 0);
  lv_obj_set_style_border_width(panelHeader, 0, 0);
  lv_obj_set_style_pad_all(panelHeader, 6, 0);
  lv_obj_set_flex_flow(panelHeader, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(panelHeader, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_clear_flag(panelHeader, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* panelTitle = lv_label_create(panelHeader);
  lv_label_set_text(panelTitle, "ACTIONS");
  lv_obj_set_style_text_color(panelTitle, lv_color_hex(0xe6eeff), 0);

  lv_obj_t* panelToggle = lv_btn_create(panelHeader);
  lv_obj_set_size(panelToggle, 46, 24);
  lv_obj_set_style_radius(panelToggle, 6, 0);
  lv_obj_set_style_bg_color(panelToggle, lv_color_hex(0x244374), 0);
  lv_obj_add_event_cb(panelToggle, onPanelToggleEvent, LV_EVENT_CLICKED, this);
  panelToggleLabel_ = lv_label_create(panelToggle);
  lv_label_set_text(panelToggleLabel_, "<<");
  lv_obj_center(panelToggleLabel_);

  actionContent_ = lv_obj_create(actionPanel_);
  lv_obj_set_width(actionContent_, LV_PCT(100));
  lv_obj_set_flex_grow(actionContent_, 1);
  lv_obj_set_style_bg_opa(actionContent_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(actionContent_, 0, 0);
  lv_obj_set_style_pad_all(actionContent_, 0, 0);
  lv_obj_set_style_pad_gap(actionContent_, 6, 0);
  lv_obj_set_flex_flow(actionContent_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(actionContent_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_clear_flag(actionContent_, LV_OBJ_FLAG_SCROLLABLE);

  makeActionButton(actionContent_, "BUZZER 1s", onBuzz1Event, this);
  makeActionButton(actionContent_, "BUZZER 5s", onBuzz5Event, this);
  makeActionButton(actionContent_, "SD START", onSdStartEvent, this);
  makeActionButton(actionContent_, "SD STOP", onSdStopEvent, this);
  makeActionButton(actionContent_, "TX ENABLE", onTelemEnableEvent, this);
  makeActionButton(actionContent_, "TX DISABLE", onTelemDisableEvent, this);

  lv_obj_t* settingsCard = lv_obj_create(actionContent_);
  lv_obj_set_width(settingsCard, LV_PCT(100));
  lv_obj_set_style_bg_color(settingsCard, lv_color_hex(0x17263f), 0);
  lv_obj_set_style_border_color(settingsCard, lv_color_hex(0x325588), 0);
  lv_obj_set_style_border_width(settingsCard, 1, 0);
  lv_obj_set_style_radius(settingsCard, 10, 0);
  lv_obj_set_style_pad_all(settingsCard, 6, 0);
  lv_obj_set_style_pad_gap(settingsCard, 6, 0);
  lv_obj_set_flex_flow(settingsCard, LV_FLEX_FLOW_COLUMN);
  lv_obj_clear_flag(settingsCard, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* settingsToggle = lv_btn_create(settingsCard);
  lv_obj_set_width(settingsToggle, LV_PCT(100));
  lv_obj_set_height(settingsToggle, 28);
  lv_obj_set_style_bg_color(settingsToggle, lv_color_hex(0x1c3356), 0);
  lv_obj_set_style_radius(settingsToggle, 6, 0);
  lv_obj_add_event_cb(settingsToggle, onSettingsToggleEvent, LV_EVENT_CLICKED, this);
  settingsToggleLabel_ = lv_label_create(settingsToggle);
  lv_label_set_text(settingsToggleLabel_, "Settings +");
  lv_obj_center(settingsToggleLabel_);

  settingsBody_ = lv_obj_create(settingsCard);
  lv_obj_set_width(settingsBody_, LV_PCT(100));
  lv_obj_set_height(settingsBody_, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(settingsBody_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(settingsBody_, 0, 0);
  lv_obj_set_style_pad_all(settingsBody_, 0, 0);
  lv_obj_set_style_pad_gap(settingsBody_, 6, 0);
  lv_obj_set_flex_flow(settingsBody_, LV_FLEX_FLOW_COLUMN);
  lv_obj_clear_flag(settingsBody_, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* settingsInfo = lv_label_create(settingsBody_);
  lv_label_set_text(settingsInfo, "Touch calibration\nstored in flash");
  lv_obj_set_style_text_color(settingsInfo, lv_color_hex(0xc2d4f4), 0);

  makeActionButton(settingsBody_, "CALIBRATE TOUCH", onCalibrateEvent, this);
  lv_obj_add_flag(settingsBody_, LV_OBJ_FLAG_HIDDEN);

  calibrationOverlay_ = lv_obj_create(screen);
  lv_obj_set_size(calibrationOverlay_, LV_PCT(100), LV_PCT(100));
  lv_obj_set_style_bg_color(calibrationOverlay_, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(calibrationOverlay_, LV_OPA_80, 0);
  lv_obj_set_style_border_width(calibrationOverlay_, 0, 0);
  lv_obj_set_style_pad_all(calibrationOverlay_, 0, 0);
  lv_obj_clear_flag(calibrationOverlay_, LV_OBJ_FLAG_SCROLLABLE);

  calibrationInstrLabel_ = lv_label_create(calibrationOverlay_);
  lv_obj_set_style_text_color(calibrationInstrLabel_, lv_color_hex(0xeef4ff), 0);
  lv_obj_align(calibrationInstrLabel_, LV_ALIGN_TOP_MID, 0, 18);

  calibrationRawLabel_ = lv_label_create(calibrationOverlay_);
  lv_obj_set_style_text_color(calibrationRawLabel_, lv_color_hex(0xb7d1ff), 0);
  lv_obj_align(calibrationRawLabel_, LV_ALIGN_BOTTOM_MID, 0, -20);

  calibrationTarget_ = lv_obj_create(calibrationOverlay_);
  lv_obj_set_size(calibrationTarget_, 18, 18);
  lv_obj_set_style_bg_color(calibrationTarget_, lv_color_hex(0xffbf3d), 0);
  lv_obj_set_style_border_color(calibrationTarget_, lv_color_hex(0xfff6da), 0);
  lv_obj_set_style_border_width(calibrationTarget_, 2, 0);
  lv_obj_set_style_radius(calibrationTarget_, LV_RADIUS_CIRCLE, 0);

  lv_obj_add_flag(calibrationOverlay_, LV_OBJ_FLAG_HIDDEN);
}

void LvglController::loadTouchCalibration() {
  touchCal_.xMin = prefs_.getInt("txmin", TOUCH_X_MIN);
  touchCal_.xMax = prefs_.getInt("txmax", TOUCH_X_MAX);
  touchCal_.yMin = prefs_.getInt("tymin", TOUCH_Y_MIN);
  touchCal_.yMax = prefs_.getInt("tymax", TOUCH_Y_MAX);
  touchCal_.swapXY = prefs_.getBool("tswap", TOUCH_SWAP_XY != 0);

  if (touchCal_.xMax == touchCal_.xMin) {
    touchCal_.xMin = TOUCH_X_MIN;
    touchCal_.xMax = TOUCH_X_MAX;
  }
  if (touchCal_.yMax == touchCal_.yMin) {
    touchCal_.yMin = TOUCH_Y_MIN;
    touchCal_.yMax = TOUCH_Y_MAX;
  }
}

void LvglController::saveTouchCalibration() {
  prefs_.putInt("txmin", touchCal_.xMin);
  prefs_.putInt("txmax", touchCal_.xMax);
  prefs_.putInt("tymin", touchCal_.yMin);
  prefs_.putInt("tymax", touchCal_.yMax);
  prefs_.putBool("tswap", touchCal_.swapXY);
}

void LvglController::begin() {
  prefs_.begin("companion-ui", false);
  loadTouchCalibration();

  tft_.init();
  tft_.setRotation(1);
  tft_.fillScreen(TFT_BLACK);

  touch_.begin();
  touch_.setRotation(1);

#if COMPANION_LINK_UART
  pinMode(COMPANION_BAT_ADC_PIN, INPUT);
#endif

  initLvgl();
  buildUi();

  if (COMPANION_LINK_UART) {
    uart_.begin();
  } else {
    if (api_.fetchInitialState(state_)) {
      lastRxMs_ = millis();
    }
    sseConnected_ = api_.openEventStream();
  }

  lastPhase_ = state_.flight.phase;
  lastConnected_ = state_.link.connected;
  lastStale_ = state_.stale;
  lastPrimaryAlert_ = state_.primaryAlert;
  lastLvTickMs_ = millis();

  refreshUi();
}

void LvglController::updateCompanionBattery() {
#if !COMPANION_LINK_UART
  return;
#endif

  const uint32_t now = millis();
  if (lastCompanionBatSampleMs_ != 0 && (now - lastCompanionBatSampleMs_) < kBatterySampleIntervalMs) {
    return;
  }
  lastCompanionBatSampleMs_ = now;

  uint32_t sum = 0;
  for (uint8_t i = 0; i < COMPANION_BAT_ADC_SAMPLES; ++i) {
    sum += static_cast<uint32_t>(analogRead(COMPANION_BAT_ADC_PIN));
  }

  const float counts = static_cast<float>(sum) / static_cast<float>(COMPANION_BAT_ADC_SAMPLES);
  const float vadc = counts * (kCompanionBatAdcRefV / kCompanionBatAdcMaxCounts);
  state_.battery.companionVbatV =
      vadc * COMPANION_BAT_ADC_DIVIDER_SCALE * COMPANION_BAT_ADC_CAL_SCALE;
}

bool LvglController::ensureConnected() {
  if (COMPANION_LINK_UART) {
    return true;
  }
  if (sseConnected_) {
    return true;
  }

  const uint32_t now = millis();
  if (now - lastReconnectAttemptMs_ < 2000) {
    return false;
  }
  lastReconnectAttemptMs_ = now;
  sseConnected_ = api_.openEventStream();
  return sseConnected_;
}

void LvglController::updateStaleness() {
  const uint32_t now = millis();
  const uint32_t age = (lastRxMs_ == 0) ? UINT32_MAX : (now - lastRxMs_);
  state_.stale = (age > 3000);
  if (state_.stale) {
    state_.link.connected = false;
    state_.link.lastPacketAgeMs = static_cast<int>(age);
  }
}

bool LvglController::mapTouchToScreen(int rawX, int rawY, int& outX, int& outY) const {
  if (touchCal_.xMax == touchCal_.xMin || touchCal_.yMax == touchCal_.yMin) {
    return false;
  }

  if (touchCal_.swapXY) {
    const int tmp = rawX;
    rawX = rawY;
    rawY = tmp;
  }

  const long sx = map(rawX, touchCal_.xMin, touchCal_.xMax, 0, kScreenWidth - 1);
  const long sy = map(rawY, touchCal_.yMin, touchCal_.yMax, 0, kScreenHeight - 1);

  outX = constrain(static_cast<int>(sx), 0, kScreenWidth - 1);
  outY = constrain(static_cast<int>(sy), 0, kScreenHeight - 1);
  return true;
}

void LvglController::setCommandStatus(const String& msg, bool ok) {
  cmdMsg_ = msg;
  cmdOk_ = ok;
  cmdTs_ = millis();
}

void LvglController::sendAction(const String& action, int durationS) {
  bool sent = false;
  if (COMPANION_LINK_UART) {
    sent = uart_.sendCommand(action, durationS);
  } else {
    sent = api_.sendCommand(action, durationS);
  }

  if (sent) {
    if (action == "sd_start") {
      sdLoggingEnabled_ = true;
    } else if (action == "sd_stop") {
      sdLoggingEnabled_ = false;
    } else if (action == "telemetry_enable") {
      telemetryTxEnabled_ = true;
    } else if (action == "telemetry_disable") {
      telemetryTxEnabled_ = false;
    }
  }

  String pretty = action;
  pretty.replace("_", " ");
  pretty.toUpperCase();

  if (action == "buzzer") {
    pretty = "BUZZER " + String(durationS) + "s";
  }

  setCommandStatus(sent ? (pretty + " sent") : (pretty + " failed"), sent);
  refreshUi();
}

void LvglController::togglePanel() {
  panelCollapsed_ = !panelCollapsed_;
  if (panelCollapsed_) {
    lv_obj_add_flag(actionPanel_, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(panelToggleLabel_, ">>");
    lv_label_set_text(panelQuickToggleLabel_, ">>");
    lv_obj_clear_flag(panelQuickToggle_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_clear_flag(actionPanel_, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(panelToggleLabel_, "<<");
    lv_obj_add_flag(panelQuickToggle_, LV_OBJ_FLAG_HIDDEN);
  }
  lv_obj_update_layout(root_);
}

void LvglController::toggleSettings() {
  settingsCollapsed_ = !settingsCollapsed_;
  if (settingsCollapsed_) {
    lv_obj_add_flag(settingsBody_, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(settingsToggleLabel_, "Settings +");
  } else {
    lv_obj_clear_flag(settingsBody_, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(settingsToggleLabel_, "Settings -");
  }
  lv_obj_update_layout(actionPanel_);
}

void LvglController::advanceCalibrationTarget() {
  int x = kCalMarginPx;
  int y = kCalMarginPx;
  const int maxX = kScreenWidth - kCalMarginPx;
  const int maxY = kScreenHeight - kCalMarginPx;

  switch (calibrationStep_) {
    case 0:
      x = kCalMarginPx;
      y = kCalMarginPx;
      lv_label_set_text(calibrationInstrLabel_, "Calibration 1/4: tap top-left");
      break;
    case 1:
      x = maxX;
      y = kCalMarginPx;
      lv_label_set_text(calibrationInstrLabel_, "Calibration 2/4: tap top-right");
      break;
    case 2:
      x = maxX;
      y = maxY;
      lv_label_set_text(calibrationInstrLabel_, "Calibration 3/4: tap bottom-right");
      break;
    case 3:
      x = kCalMarginPx;
      y = maxY;
      lv_label_set_text(calibrationInstrLabel_, "Calibration 4/4: tap bottom-left");
      break;
    default:
      break;
  }

  lv_obj_set_pos(calibrationTarget_, x - 9, y - 9);
}

void LvglController::startCalibration() {
  calibrationActive_ = true;
  calibrationTouchLatch_ = false;
  calibrationStep_ = 0;
  lv_obj_clear_flag(calibrationOverlay_, LV_OBJ_FLAG_HIDDEN);
  lv_label_set_text(calibrationRawLabel_, "Waiting for touch...");
  advanceCalibrationTarget();
}

void LvglController::cancelCalibration() {
  calibrationActive_ = false;
  calibrationTouchLatch_ = false;
  calibrationStep_ = 0;
  lv_obj_add_flag(calibrationOverlay_, LV_OBJ_FLAG_HIDDEN);
}

bool LvglController::completeCalibration() {
  const int32_t topDx = abs(calibrationRawX_[1] - calibrationRawX_[0]);
  const int32_t topDy = abs(calibrationRawY_[1] - calibrationRawY_[0]);
  const bool swapAxes = topDx < topDy;

  int32_t xs[kCalPointCount];
  int32_t ys[kCalPointCount];
  for (uint8_t i = 0; i < kCalPointCount; ++i) {
    xs[i] = calibrationRawX_[i];
    ys[i] = calibrationRawY_[i];
    if (swapAxes) {
      const int32_t t = xs[i];
      xs[i] = ys[i];
      ys[i] = t;
    }
  }

  const int32_t xLeft = (xs[0] + xs[3]) / 2;
  const int32_t xRight = (xs[1] + xs[2]) / 2;
  const int32_t yTop = (ys[0] + ys[1]) / 2;
  const int32_t yBottom = (ys[2] + ys[3]) / 2;

  if (abs(xRight - xLeft) < 300 || abs(yBottom - yTop) < 300) {
    setCommandStatus("Calibration failed", false);
    return false;
  }

  touchCal_.xMin = xLeft;
  touchCal_.xMax = xRight;
  touchCal_.yMin = yTop;
  touchCal_.yMax = yBottom;
  touchCal_.swapXY = swapAxes;
  saveTouchCalibration();
  setCommandStatus("Calibration saved", true);
  return true;
}

void LvglController::handleCalibrationTouch() {
  if (!calibrationActive_) {
    return;
  }

  if (!touch_.touched()) {
    calibrationTouchLatch_ = false;
    return;
  }

  if (calibrationTouchLatch_) {
    return;
  }

  TS_Point p = touch_.getPoint();
  if (calibrationStep_ < kCalPointCount) {
    calibrationRawX_[calibrationStep_] = p.x;
    calibrationRawY_[calibrationStep_] = p.y;

    lv_label_set_text_fmt(calibrationRawLabel_, "Raw sample: %d,%d", p.x, p.y);

    calibrationStep_++;
    if (calibrationStep_ >= kCalPointCount) {
      const bool ok = completeCalibration();
      cancelCalibration();
      if (!ok) {
        startCalibration();
      }
    } else {
      advanceCalibrationTarget();
    }
  }

  calibrationTouchLatch_ = true;
}

void LvglController::refreshUi() {
  const bool linkLive = state_.link.connected && !state_.stale;

  lv_label_set_text(linkLabel_, linkLive ? "LINK LIVE" : (state_.stale ? "LINK STALE" : "NO LINK"));
  lv_obj_set_style_text_color(
      linkLabel_,
      linkLive ? lv_color_hex(0x6cff95)
               : (state_.stale ? lv_color_hex(0xffc369) : lv_color_hex(0xff6b6b)),
      0);

  lv_label_set_text_fmt(linkMetaLabel_, "RSSI %d dBm   SNR %.1f   AGE %d ms", state_.link.rssi,
                        state_.link.snr, state_.link.lastPacketAgeMs);

  lv_label_set_text_fmt(phaseLabel_, "PHASE: %s", state_.flight.phase.length() ? state_.flight.phase.c_str() : "unknown");

  const String altText = formatFloat(state_.alt.altitudeAglM, 1);
  const String vsText = formatFloat(state_.alt.verticalSpeedMps, 1);

  lv_label_set_text_fmt(altitudeLabel_, "ALT %s m", altText.c_str());
  lv_label_set_text_fmt(vsLabel_, "VS  %s m/s", vsText.c_str());

  lv_label_set_text_fmt(packetLabel_, "Packets: %lu", static_cast<unsigned long>(state_.flight.packetCount));
  lv_label_set_text_fmt(callsignLabel_, "Callsign: %s",
                        state_.flight.callsign.length() ? state_.flight.callsign.c_str() : "(none)");

  const String txVbat = formatFloat(state_.battery.telemetryVbatV, 2, "--.-");
  const String companionVbat = formatFloat(state_.battery.companionVbatV, 2, "--.-");

  lv_label_set_text_fmt(batteryLabel_, "TX_VBAT: %s V", txVbat.c_str());
  lv_label_set_text_fmt(companionBatteryLabel_, "BAT_ADC: %s V", companionVbat.c_str());

  String cmdStatus = "";
  if (cmdMsg_.length() > 0 && (millis() - cmdTs_) <= kCommandStatusShowMs) {
    cmdStatus = cmdMsg_;
    lv_obj_set_style_text_color(cmdStatusLabel_, cmdOk_ ? lv_color_hex(0x87f0ae) : lv_color_hex(0xff8e8e), 0);
  }
  lv_label_set_text(cmdStatusLabel_, cmdStatus.c_str());

  String alert;
  if (state_.primaryAlert.length() > 0) {
    alert = state_.primaryAlert;
  } else if (state_.stale) {
    alert = "Telemetry stale";
  } else {
    alert = "Nominal";
  }
  lv_label_set_text(alertLabel_, alert.c_str());

  lv_timer_handler();
}

void LvglController::tick() {
  const uint32_t now = millis();
  if (lastLvTickMs_ == 0) {
    lastLvTickMs_ = now;
  }
  lv_tick_inc(now - lastLvTickMs_);
  lastLvTickMs_ = now;

  ensureConnected();
  updateCompanionBattery();

  bool updated = false;
  if (COMPANION_LINK_UART) {
    updated = uart_.poll(state_);
  } else if (sseConnected_) {
    updated = api_.pollEventStream(state_);
  }

  if (updated) {
    lastRxMs_ = now;

    if (state_.flight.phase != lastPhase_ && state_.flight.phase.length() > 0) {
      setCommandStatus("Phase -> " + state_.flight.phase, true);
      lastPhase_ = state_.flight.phase;
    }
    if (state_.primaryAlert != lastPrimaryAlert_) {
      lastPrimaryAlert_ = state_.primaryAlert;
    }
    lastConnected_ = state_.link.connected;
    lastStale_ = state_.stale;
  }

  handleCalibrationTouch();
  updateStaleness();

  const bool statusVisible = cmdMsg_.length() > 0 && (now - cmdTs_) <= kCommandStatusShowMs;
  if (updated || statusVisible || (now - lastUiRefreshMs_) >= kUiRefreshIntervalMs) {
    refreshUi();
    lastUiRefreshMs_ = now;
  } else {
    lv_timer_handler();
  }
}

void LvglController::flushDisplayCb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* colorP) {
  LvglController* self = static_cast<LvglController*>(disp->user_data);
  const uint32_t w = static_cast<uint32_t>(area->x2 - area->x1 + 1);
  const uint32_t h = static_cast<uint32_t>(area->y2 - area->y1 + 1);

  self->tft_.startWrite();
  self->tft_.setAddrWindow(area->x1, area->y1, w, h);
  self->tft_.pushColors(reinterpret_cast<uint16_t*>(&colorP->full), w * h, true);
  self->tft_.endWrite();

  lv_disp_flush_ready(disp);
}

void LvglController::readTouchCb(lv_indev_drv_t* indev, lv_indev_data_t* data) {
  LvglController* self = static_cast<LvglController*>(indev->user_data);

  if (self->calibrationActive_) {
    data->state = LV_INDEV_STATE_REL;
    return;
  }

  if (!self->touch_.touched()) {
    data->state = LV_INDEV_STATE_REL;
    return;
  }

  TS_Point p = self->touch_.getPoint();
  int x = 0;
  int y = 0;
  if (!self->mapTouchToScreen(p.x, p.y, x, y)) {
    data->state = LV_INDEV_STATE_REL;
    return;
  }

  data->state = LV_INDEV_STATE_PR;
  data->point.x = x;
  data->point.y = y;
}

void LvglController::onPanelToggleEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  self->togglePanel();
}

void LvglController::onSettingsToggleEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  self->toggleSettings();
}

void LvglController::onCalibrateEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  self->startCalibration();
}

void LvglController::onBuzz1Event(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  self->sendAction("buzzer", 1);
}

void LvglController::onBuzz5Event(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  self->sendAction("buzzer", 5);
}

void LvglController::onSdStartEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  self->sendAction("sd_start", 0);
}

void LvglController::onSdStopEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  self->sendAction("sd_stop", 0);
}

void LvglController::onTelemEnableEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  self->sendAction("telemetry_enable", 0);
}

void LvglController::onTelemDisableEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  self->sendAction("telemetry_disable", 0);
}
