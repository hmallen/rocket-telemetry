#include "lvgl_controller.h"

#include <esp_heap_caps.h>
#include <math.h>
#include <SPI.h>

namespace {

constexpr uint32_t kUiRefreshIntervalMs = 120;
constexpr uint32_t kCommandStatusShowMs = 3000;
constexpr uint32_t kCommandConfirmTimeoutMs = 5000;
constexpr uint8_t kCalPointCount = 4;
constexpr int kCalMarginPx = 24;
constexpr int32_t kCalRetouchDistancePx = 120;
constexpr int32_t kTouchPressThreshold = 140;
constexpr int32_t kTouchReleaseThreshold = 90;

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) {
    const uint16_t t = a;
    a = b;
    b = t;
  }
  if (b > c) {
    const uint16_t t = b;
    b = c;
    c = t;
  }
  if (a > b) {
    const uint16_t t = a;
    a = b;
    b = t;
  }
  return b;
}

static int32_t mapLinearRange(int32_t value,
                              int32_t inMin,
                              int32_t inMax,
                              int32_t outMin,
                              int32_t outMax) {
  if (inMax == inMin) {
    return outMin;
  }
  const int64_t numerator = static_cast<int64_t>(value - inMin) * static_cast<int64_t>(outMax - outMin);
  const int64_t denominator = static_cast<int64_t>(inMax - inMin);
  return static_cast<int32_t>(static_cast<int64_t>(outMin) + (numerator / denominator));
}

#ifndef AUTO_TOUCH_CALIBRATION_AT_BOOT
#define AUTO_TOUCH_CALIBRATION_AT_BOOT 0
#endif

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

#ifndef COMPANION_BAT_ADC_MIN_VALID_MV
#define COMPANION_BAT_ADC_MIN_VALID_MV 50
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
  lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_width(btn, LV_PCT(100));
  lv_obj_set_height(btn, 34);
  lv_obj_set_style_radius(btn, 8, 0);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x1f2a3b), 0);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x2d4f86), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(btn, lv_color_hex(0x4b7dd1), 0);
  lv_obj_set_style_border_width(btn, 1, 0);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_PRESSED, userData);

  lv_obj_t* label = lv_label_create(btn);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_color(label, lv_color_hex(0xeaf1ff), 0);
  lv_obj_clear_flag(label, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(label);
  return btn;
}

}  // namespace

LvglController::LvglController(TFT_eSPI& tft, const String& host, uint16_t port)
    : api_(host, port),
      uart_(companionUartPort(), UART_BAUD, UART_RX_PIN, UART_TX_PIN),
      tft_(tft) {}

void LvglController::initLvgl() {
  lv_init();

  const size_t pixelCount = static_cast<size_t>(kScreenWidth) * kDrawBufferLines;
  const size_t drawBufSize = pixelCount * sizeof(uint16_t);
  drawBufPixels_ = static_cast<uint8_t*>(
      heap_caps_malloc(drawBufSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (drawBufPixels_ == nullptr) {
    drawBufPixels_ = static_cast<uint8_t*>(malloc(drawBufSize));
  }

  if (drawBufPixels_ == nullptr) {
    while (true) {
      delay(1000);
    }
  }

  display_ = lv_display_create(kScreenWidth, kScreenHeight);
  if (display_ == nullptr) {
    while (true) {
      delay(1000);
    }
  }

  lv_display_set_color_format(display_, LV_COLOR_FORMAT_RGB565);
  lv_display_set_buffers(display_, drawBufPixels_, nullptr, drawBufSize, LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(display_, flushDisplayCb);
  lv_display_set_user_data(display_, this);

  touchIndev_ = lv_indev_create();
  lv_indev_set_type(touchIndev_, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(touchIndev_, readTouchCb);
  lv_indev_set_user_data(touchIndev_, this);
  lv_indev_set_display(touchIndev_, display_);
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
  lv_obj_clear_flag(root_, LV_OBJ_FLAG_CLICKABLE);

  telemetryPanel_ = lv_obj_create(root_);
  lv_obj_set_height(telemetryPanel_, LV_PCT(100));
  lv_obj_set_flex_grow(telemetryPanel_, 1);
  lv_obj_set_style_bg_color(telemetryPanel_, lv_color_hex(0x0b1322), 0);
  lv_obj_set_style_border_color(telemetryPanel_, lv_color_hex(0x203457), 0);
  lv_obj_set_style_border_width(telemetryPanel_, 1, 0);
  lv_obj_set_style_radius(telemetryPanel_, 12, 0);
  lv_obj_set_style_pad_all(telemetryPanel_, 12, 0);
  lv_obj_clear_flag(telemetryPanel_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(telemetryPanel_, LV_OBJ_FLAG_CLICKABLE);

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
  lv_obj_align(packetLabel_, LV_ALIGN_BOTTOM_LEFT, 0, -88);

  callsignLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(callsignLabel_, lv_color_hex(0xd9e3f8), 0);
  lv_obj_align(callsignLabel_, LV_ALIGN_BOTTOM_LEFT, 0, -68);

  batteryLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(batteryLabel_, lv_color_hex(0xd9e3f8), 0);
  lv_obj_align(batteryLabel_, LV_ALIGN_BOTTOM_LEFT, 0, -48);

  companionBatteryLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(companionBatteryLabel_, lv_color_hex(0xd9e3f8), 0);
  lv_obj_align(companionBatteryLabel_, LV_ALIGN_BOTTOM_LEFT, 0, -28);

  companionBatteryDebugLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(companionBatteryDebugLabel_, lv_color_hex(0x9fb0cc), 0);
  lv_obj_align(companionBatteryDebugLabel_, LV_ALIGN_BOTTOM_LEFT, 0, -8);

  cmdStatusLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(cmdStatusLabel_, lv_color_hex(0x6be7a4), 0);
  lv_obj_align(cmdStatusLabel_, LV_ALIGN_BOTTOM_RIGHT, 0, -36);

  alertLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(alertLabel_, lv_color_hex(0xff8181), 0);
  lv_obj_align(alertLabel_, LV_ALIGN_BOTTOM_RIGHT, 0, -8);

  touchDebugLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_width(touchDebugLabel_, 162);
  lv_label_set_long_mode(touchDebugLabel_, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align(touchDebugLabel_, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_set_style_bg_opa(touchDebugLabel_, LV_OPA_70, 0);
  lv_obj_set_style_bg_color(touchDebugLabel_, lv_color_hex(0x000000), 0);
  lv_obj_set_style_radius(touchDebugLabel_, 6, 0);
  lv_obj_set_style_pad_all(touchDebugLabel_, 4, 0);
  lv_obj_set_style_text_color(touchDebugLabel_, lv_color_hex(0x9aa8c5), 0);
  lv_obj_align(touchDebugLabel_, LV_ALIGN_TOP_RIGHT, 0, 44);
  lv_label_set_text(touchDebugLabel_, "TOUCH init");
  lv_obj_clear_flag(touchDebugLabel_, LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t* menuIconRow = lv_obj_create(telemetryPanel_);
  lv_obj_set_size(menuIconRow, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(menuIconRow, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(menuIconRow, 0, 0);
  lv_obj_set_style_pad_all(menuIconRow, 0, 0);
  lv_obj_set_style_pad_gap(menuIconRow, 6, 0);
  lv_obj_set_flex_flow(menuIconRow, LV_FLEX_FLOW_ROW);
  lv_obj_clear_flag(menuIconRow, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(menuIconRow, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_align(menuIconRow, LV_ALIGN_TOP_RIGHT, 0, 0);

  sdToggleBtn_ = lv_btn_create(menuIconRow);
  lv_obj_add_flag(sdToggleBtn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(sdToggleBtn_, 72, 30);
  lv_obj_set_style_radius(sdToggleBtn_, 8, 0);
  lv_obj_add_event_cb(sdToggleBtn_, onSdToggleEvent, LV_EVENT_PRESSED, this);
  lv_obj_add_event_cb(sdToggleBtn_, onSdToggleEvent, LV_EVENT_LONG_PRESSED, this);
  sdToggleLabel_ = lv_label_create(sdToggleBtn_);
  lv_obj_set_style_text_color(sdToggleLabel_, lv_color_hex(0xe6eeff), 0);
  lv_obj_clear_flag(sdToggleLabel_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(sdToggleLabel_);

  txToggleBtn_ = lv_btn_create(menuIconRow);
  lv_obj_add_flag(txToggleBtn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(txToggleBtn_, 72, 30);
  lv_obj_set_style_radius(txToggleBtn_, 8, 0);
  lv_obj_add_event_cb(txToggleBtn_, onTxToggleEvent, LV_EVENT_PRESSED, this);
  lv_obj_add_event_cb(txToggleBtn_, onTxToggleEvent, LV_EVENT_LONG_PRESSED, this);
  txToggleLabel_ = lv_label_create(txToggleBtn_);
  lv_obj_set_style_text_color(txToggleLabel_, lv_color_hex(0xe6eeff), 0);
  lv_obj_clear_flag(txToggleLabel_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(txToggleLabel_);

  lv_obj_t* actionMenuBtn = lv_btn_create(menuIconRow);
  lv_obj_add_flag(actionMenuBtn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(actionMenuBtn, 36, 30);
  lv_obj_set_style_radius(actionMenuBtn, 8, 0);
  lv_obj_set_style_bg_color(actionMenuBtn, lv_color_hex(0x244374), 0);
  lv_obj_set_style_bg_color(actionMenuBtn, lv_color_hex(0x2e5ca0), LV_STATE_PRESSED);
  lv_obj_add_event_cb(actionMenuBtn, onPanelToggleEvent, LV_EVENT_PRESSED, this);
  lv_obj_t* actionMenuIcon = lv_label_create(actionMenuBtn);
  lv_label_set_text(actionMenuIcon, LV_SYMBOL_CHARGE);
  lv_obj_set_style_text_color(actionMenuIcon, lv_color_hex(0xe6eeff), 0);
  lv_obj_clear_flag(actionMenuIcon, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(actionMenuIcon);

  lv_obj_t* settingsMenuBtn = lv_btn_create(menuIconRow);
  lv_obj_add_flag(settingsMenuBtn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(settingsMenuBtn, 36, 30);
  lv_obj_set_style_radius(settingsMenuBtn, 8, 0);
  lv_obj_set_style_bg_color(settingsMenuBtn, lv_color_hex(0x244374), 0);
  lv_obj_set_style_bg_color(settingsMenuBtn, lv_color_hex(0x2e5ca0), LV_STATE_PRESSED);
  lv_obj_add_event_cb(settingsMenuBtn, onSettingsToggleEvent, LV_EVENT_PRESSED, this);
  lv_obj_t* settingsMenuIcon = lv_label_create(settingsMenuBtn);
  lv_label_set_text(settingsMenuIcon, LV_SYMBOL_SETTINGS);
  lv_obj_set_style_text_color(settingsMenuIcon, lv_color_hex(0xe6eeff), 0);
  lv_obj_clear_flag(settingsMenuIcon, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(settingsMenuIcon);

  actionPanel_ = lv_obj_create(telemetryPanel_);
  lv_obj_set_width(actionPanel_, kActionPanelWidth);
  lv_obj_set_height(actionPanel_, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_color(actionPanel_, lv_color_hex(0x111c2e), 0);
  lv_obj_set_style_border_color(actionPanel_, lv_color_hex(0x2a446a), 0);
  lv_obj_set_style_border_width(actionPanel_, 1, 0);
  lv_obj_set_style_radius(actionPanel_, 10, 0);
  lv_obj_set_style_pad_all(actionPanel_, 8, 0);
  lv_obj_set_style_pad_gap(actionPanel_, 8, 0);
  lv_obj_set_flex_flow(actionPanel_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(actionPanel_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_clear_flag(actionPanel_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(actionPanel_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_align(actionPanel_, LV_ALIGN_TOP_RIGHT, 0, 38);

  lv_obj_t* actionTitle = lv_label_create(actionPanel_);
  lv_label_set_text(actionTitle, "ACTIONS");
  lv_obj_set_style_text_color(actionTitle, lv_color_hex(0xe6eeff), 0);
  lv_obj_clear_flag(actionTitle, LV_OBJ_FLAG_CLICKABLE);

  actionContent_ = lv_obj_create(actionPanel_);
  lv_obj_set_width(actionContent_, LV_PCT(100));
  lv_obj_set_height(actionContent_, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(actionContent_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(actionContent_, 0, 0);
  lv_obj_set_style_pad_all(actionContent_, 0, 0);
  lv_obj_set_style_pad_gap(actionContent_, 6, 0);
  lv_obj_set_flex_flow(actionContent_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(actionContent_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_clear_flag(actionContent_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(actionContent_, LV_OBJ_FLAG_CLICKABLE);

  makeActionButton(actionContent_, "BUZZER", onBuzzerToggleEvent, this);

  buzzerConfigRow_ = lv_obj_create(actionContent_);
  lv_obj_set_width(buzzerConfigRow_, LV_PCT(100));
  lv_obj_set_height(buzzerConfigRow_, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(buzzerConfigRow_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(buzzerConfigRow_, 0, 0);
  lv_obj_set_style_pad_all(buzzerConfigRow_, 0, 0);
  lv_obj_set_style_pad_gap(buzzerConfigRow_, 6, 0);
  lv_obj_set_flex_flow(buzzerConfigRow_, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(buzzerConfigRow_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_clear_flag(buzzerConfigRow_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(buzzerConfigRow_, LV_OBJ_FLAG_CLICKABLE);

  buzzerDurationSlider_ = lv_slider_create(buzzerConfigRow_);
  lv_obj_set_width(buzzerDurationSlider_, 112);
  lv_slider_set_range(buzzerDurationSlider_, 1, 10);
  lv_slider_set_value(buzzerDurationSlider_, buzzerDurationS_, LV_ANIM_OFF);
  lv_obj_add_event_cb(buzzerDurationSlider_, onBuzzerDurationChangedEvent, LV_EVENT_VALUE_CHANGED, this);

  buzzerDurationLabel_ = lv_label_create(buzzerConfigRow_);
  lv_obj_set_width(buzzerDurationLabel_, 24);
  lv_obj_set_style_text_align(buzzerDurationLabel_, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_set_style_text_color(buzzerDurationLabel_, lv_color_hex(0xcfe0ff), 0);
  lv_obj_clear_flag(buzzerDurationLabel_, LV_OBJ_FLAG_CLICKABLE);
  lv_label_set_text_fmt(buzzerDurationLabel_, "%us", static_cast<unsigned>(buzzerDurationS_));

  lv_obj_t* buzzerSendBtn = lv_btn_create(buzzerConfigRow_);
  lv_obj_add_flag(buzzerSendBtn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(buzzerSendBtn, 56, 30);
  lv_obj_set_style_radius(buzzerSendBtn, 8, 0);
  lv_obj_set_style_bg_color(buzzerSendBtn, lv_color_hex(0x1f2a3b), 0);
  lv_obj_set_style_bg_color(buzzerSendBtn, lv_color_hex(0x2d4f86), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(buzzerSendBtn, lv_color_hex(0x4b7dd1), 0);
  lv_obj_set_style_border_width(buzzerSendBtn, 1, 0);
  lv_obj_add_event_cb(buzzerSendBtn, onBuzzerSendEvent, LV_EVENT_PRESSED, this);

  lv_obj_t* buzzerSendLabel = lv_label_create(buzzerSendBtn);
  lv_label_set_text(buzzerSendLabel, "SEND");
  lv_obj_set_style_text_color(buzzerSendLabel, lv_color_hex(0xeaf1ff), 0);
  lv_obj_clear_flag(buzzerSendLabel, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(buzzerSendLabel);

  setBuzzerConfigVisible(false);
  lv_obj_add_flag(actionPanel_, LV_OBJ_FLAG_HIDDEN);

  settingsBody_ = lv_obj_create(telemetryPanel_);
  lv_obj_set_width(settingsBody_, kActionPanelWidth);
  lv_obj_set_height(settingsBody_, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_color(settingsBody_, lv_color_hex(0x111c2e), 0);
  lv_obj_set_style_border_color(settingsBody_, lv_color_hex(0x2a446a), 0);
  lv_obj_set_style_border_width(settingsBody_, 1, 0);
  lv_obj_set_style_radius(settingsBody_, 10, 0);
  lv_obj_set_style_pad_all(settingsBody_, 8, 0);
  lv_obj_set_style_pad_gap(settingsBody_, 6, 0);
  lv_obj_set_flex_flow(settingsBody_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(settingsBody_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_clear_flag(settingsBody_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(settingsBody_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_align(settingsBody_, LV_ALIGN_TOP_RIGHT, 0, 38);

  lv_obj_t* settingsTitle = lv_label_create(settingsBody_);
  lv_label_set_text(settingsTitle, "SETTINGS");
  lv_obj_set_style_text_color(settingsTitle, lv_color_hex(0xe6eeff), 0);
  lv_obj_clear_flag(settingsTitle, LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t* settingsInfo = lv_label_create(settingsBody_);
  lv_label_set_text(settingsInfo, "Touch calibration\nstored in flash");
  lv_obj_set_style_text_color(settingsInfo, lv_color_hex(0xc2d4f4), 0);
  lv_obj_clear_flag(settingsInfo, LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t* settingsActions = lv_obj_create(settingsBody_);
  lv_obj_set_width(settingsActions, LV_PCT(100));
  lv_obj_set_height(settingsActions, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(settingsActions, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(settingsActions, 0, 0);
  lv_obj_set_style_pad_all(settingsActions, 0, 0);
  lv_obj_set_style_pad_gap(settingsActions, 6, 0);
  lv_obj_set_flex_flow(settingsActions, LV_FLEX_FLOW_COLUMN);
  lv_obj_clear_flag(settingsActions, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(settingsActions, LV_OBJ_FLAG_CLICKABLE);

  makeActionButton(settingsActions, "SCREEN CALIBRATION", onCalibrateEvent, this);
  lv_obj_add_flag(settingsBody_, LV_OBJ_FLAG_HIDDEN);

  updateDashboardActionButtons();

  calibrationOverlay_ = lv_obj_create(screen);
  lv_obj_set_size(calibrationOverlay_, LV_PCT(100), LV_PCT(100));
  lv_obj_set_style_bg_color(calibrationOverlay_, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(calibrationOverlay_, LV_OPA_80, 0);
  lv_obj_set_style_border_width(calibrationOverlay_, 0, 0);
  lv_obj_set_style_pad_all(calibrationOverlay_, 0, 0);
  lv_obj_clear_flag(calibrationOverlay_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(calibrationOverlay_, LV_OBJ_FLAG_CLICKABLE);

  calibrationInstrLabel_ = lv_label_create(calibrationOverlay_);
  lv_obj_set_style_text_color(calibrationInstrLabel_, lv_color_hex(0xeef4ff), 0);
  lv_obj_align(calibrationInstrLabel_, LV_ALIGN_TOP_MID, 0, 18);
  lv_obj_clear_flag(calibrationInstrLabel_, LV_OBJ_FLAG_CLICKABLE);

  calibrationRawLabel_ = lv_label_create(calibrationOverlay_);
  lv_obj_set_style_text_color(calibrationRawLabel_, lv_color_hex(0xb7d1ff), 0);
  lv_obj_align(calibrationRawLabel_, LV_ALIGN_BOTTOM_MID, 0, -20);
  lv_obj_clear_flag(calibrationRawLabel_, LV_OBJ_FLAG_CLICKABLE);

  calibrationTarget_ = lv_obj_create(calibrationOverlay_);
  lv_obj_set_size(calibrationTarget_, 18, 18);
  lv_obj_set_style_bg_color(calibrationTarget_, lv_color_hex(0xffbf3d), 0);
  lv_obj_set_style_border_color(calibrationTarget_, lv_color_hex(0xfff6da), 0);
  lv_obj_set_style_border_width(calibrationTarget_, 2, 0);
  lv_obj_set_style_radius(calibrationTarget_, LV_RADIUS_CIRCLE, 0);
  lv_obj_clear_flag(calibrationTarget_, LV_OBJ_FLAG_CLICKABLE);

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

#if COMPANION_LINK_UART
  pinMode(COMPANION_BAT_ADC_PIN, INPUT);
#if defined(ESP32)
  analogReadResolution(12);
#if defined(ADC_11db)
  analogSetPinAttenuation(COMPANION_BAT_ADC_PIN, ADC_11db);
#elif defined(ADC_ATTEN_DB_12)
  analogSetPinAttenuation(COMPANION_BAT_ADC_PIN, ADC_ATTEN_DB_12);
#endif
#endif
#endif

  initLvgl();
  buildUi();

#if AUTO_TOUCH_CALIBRATION_AT_BOOT
  startCalibration();
#endif

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

  uint32_t mvSum = 0;
  for (uint8_t i = 0; i < COMPANION_BAT_ADC_SAMPLES; ++i) {
#if defined(ESP32)
    mvSum += static_cast<uint32_t>(analogReadMilliVolts(COMPANION_BAT_ADC_PIN));
#else
    mvSum += static_cast<uint32_t>(analogRead(COMPANION_BAT_ADC_PIN) *
                                   (kCompanionBatAdcRefV / kCompanionBatAdcMaxCounts) * 1000.0f);
#endif
  }

  const float mvAvg = static_cast<float>(mvSum) / static_cast<float>(COMPANION_BAT_ADC_SAMPLES);
  lastCompanionBatRawMv_ = static_cast<uint32_t>(mvAvg + 0.5f);
  if (mvAvg < static_cast<float>(COMPANION_BAT_ADC_MIN_VALID_MV)) {
    lastCompanionBatRawValid_ = false;
    state_.battery.companionVbatV = NAN;
    return;
  }
  lastCompanionBatRawValid_ = true;

  const float vadc = mvAvg / 1000.0f;
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

  // Calibration points are sampled at margins (not exact screen edges), so map from
  // raw calibration span to those on-screen sample positions and extrapolate naturally.
  const int32_t calXMinPx = kCalMarginPx;
  const int32_t calXMaxPx = kScreenWidth - kCalMarginPx;
  const int32_t calYMinPx = kCalMarginPx;
  const int32_t calYMaxPx = kScreenHeight - kCalMarginPx;

  const int32_t sx = mapLinearRange(rawX, touchCal_.xMin, touchCal_.xMax, calXMinPx, calXMaxPx);
  const int32_t sy = mapLinearRange(rawY, touchCal_.yMin, touchCal_.yMax, calYMinPx, calYMaxPx);

  outX = constrain(static_cast<int>(sx), 0, kScreenWidth - 1);
  outY = constrain(static_cast<int>(sy), 0, kScreenHeight - 1);
  return true;
}

bool LvglController::readTouchRaw(int32_t& rawX, int32_t& rawY, int32_t& rawZ) {
  uint16_t x1 = 0;
  uint16_t y1 = 0;
  uint16_t x2 = 0;
  uint16_t y2 = 0;
  uint16_t x3 = 0;
  uint16_t y3 = 0;

  rawZ = static_cast<int32_t>(tft_.getTouchRawZ());
  const int32_t threshold = touchPressed_ ? kTouchReleaseThreshold : kTouchPressThreshold;
  if (rawZ < threshold) {
    touchPressed_ = false;
    return false;
  }

  if (!tft_.getTouchRaw(&x1, &y1) || !tft_.getTouchRaw(&x2, &y2) || !tft_.getTouchRaw(&x3, &y3)) {
    touchPressed_ = false;
    return false;
  }

  rawX = static_cast<int32_t>(median3(x1, x2, x3));
  rawY = static_cast<int32_t>(median3(y1, y2, y3));

  const bool inRange = (rawX >= 0 && rawY >= 0 && rawX <= 4095 && rawY <= 4095);
  touchPressed_ = inRange;
  return inRange;
}

void LvglController::setCommandStatus(const String& msg, bool ok) {
  cmdMsg_ = msg;
  cmdOk_ = ok;
  cmdTs_ = millis();
}

void LvglController::updateDashboardActionButtons() {
  if (sdToggleBtn_ != nullptr && sdToggleLabel_ != nullptr) {
    if (sdCommandPending_) {
      lv_obj_add_state(sdToggleBtn_, LV_STATE_DISABLED);
      lv_label_set_text_fmt(sdToggleLabel_, "SD %s...", sdPendingTargetEnabled_ ? "ON" : "OFF");
      lv_obj_set_style_bg_color(sdToggleBtn_, lv_color_hex(0x4b566d), 0);
      lv_obj_set_style_bg_color(sdToggleBtn_, lv_color_hex(0x4b566d), LV_STATE_PRESSED);
    } else {
      lv_obj_clear_state(sdToggleBtn_, LV_STATE_DISABLED);
      lv_label_set_text_fmt(sdToggleLabel_, "SD %s", sdLoggingEnabled_ ? "ON" : "OFF");
      lv_obj_set_style_bg_color(sdToggleBtn_, sdLoggingEnabled_ ? lv_color_hex(0x1f6a42) : lv_color_hex(0x3a2b2b), 0);
      lv_obj_set_style_bg_color(sdToggleBtn_, sdLoggingEnabled_ ? lv_color_hex(0x2a8e5a) : lv_color_hex(0x5a3f3f),
                                LV_STATE_PRESSED);
    }
    lv_obj_set_style_border_color(sdToggleBtn_, lv_color_hex(0x6ea4ff), 0);
    lv_obj_set_style_border_width(sdToggleBtn_, 1, 0);
  }

  if (txToggleBtn_ != nullptr && txToggleLabel_ != nullptr) {
    if (txCommandPending_) {
      lv_obj_add_state(txToggleBtn_, LV_STATE_DISABLED);
      lv_label_set_text_fmt(txToggleLabel_, "TX %s...", txPendingTargetEnabled_ ? "ON" : "OFF");
      lv_obj_set_style_bg_color(txToggleBtn_, lv_color_hex(0x4b566d), 0);
      lv_obj_set_style_bg_color(txToggleBtn_, lv_color_hex(0x4b566d), LV_STATE_PRESSED);
    } else {
      lv_obj_clear_state(txToggleBtn_, LV_STATE_DISABLED);
      lv_label_set_text_fmt(txToggleLabel_, "TX %s", telemetryTxEnabled_ ? "ON" : "OFF");
      lv_obj_set_style_bg_color(txToggleBtn_, telemetryTxEnabled_ ? lv_color_hex(0x1f5f73) : lv_color_hex(0x3a2b2b), 0);
      lv_obj_set_style_bg_color(txToggleBtn_, telemetryTxEnabled_ ? lv_color_hex(0x2c839f) : lv_color_hex(0x5a3f3f),
                                LV_STATE_PRESSED);
    }
    lv_obj_set_style_border_color(txToggleBtn_, lv_color_hex(0x6ea4ff), 0);
    lv_obj_set_style_border_width(txToggleBtn_, 1, 0);
  }
}

void LvglController::syncCommandStateFromTelemetry() {
  bool changed = false;

  if (state_.hasSdLoggingState) {
    const bool reported = state_.sdLoggingEnabled;
    if (sdCommandPending_) {
      if (reported == sdPendingTargetEnabled_) {
        sdCommandPending_ = false;
        sdPendingSinceMs_ = 0;
        setCommandStatus(String("SD ") + (reported ? "ON" : "OFF") + " confirmed", true);
        changed = true;
      }
    }
    if (!sdCommandPending_) {
      if (sdLoggingEnabled_ != reported) {
        changed = true;
      }
      sdLoggingEnabled_ = reported;
    }
  }

  if (state_.hasTelemetryTxState) {
    const bool reported = state_.telemetryTxEnabled;
    if (txCommandPending_) {
      if (reported == txPendingTargetEnabled_) {
        txCommandPending_ = false;
        txPendingSinceMs_ = 0;
        setCommandStatus(String("TX ") + (reported ? "ON" : "OFF") + " confirmed", true);
        changed = true;
      }
    }
    if (!txCommandPending_) {
      if (telemetryTxEnabled_ != reported) {
        changed = true;
      }
      telemetryTxEnabled_ = reported;
    }
  }

  if (changed) {
    updateDashboardActionButtons();
  }
}

void LvglController::updatePendingCommandTimeouts(uint32_t now) {
  bool changed = false;
  if (sdCommandPending_ && (now - sdPendingSinceMs_) > kCommandConfirmTimeoutMs) {
    sdCommandPending_ = false;
    sdPendingSinceMs_ = 0;
    sdLoggingEnabled_ = sdPendingPreviousEnabled_;
    setCommandStatus("SD command failed (timeout)", false);
    changed = true;
  }

  if (txCommandPending_ && (now - txPendingSinceMs_) > kCommandConfirmTimeoutMs) {
    txCommandPending_ = false;
    txPendingSinceMs_ = 0;
    telemetryTxEnabled_ = txPendingPreviousEnabled_;
    setCommandStatus("TX command failed (timeout)", false);
    changed = true;
  }

  if (changed) {
    updateDashboardActionButtons();
  }
}

void LvglController::requestSdToggle(bool enable) {
  if (sdCommandPending_ || sdLoggingEnabled_ == enable) {
    return;
  }

  sdPendingPreviousEnabled_ = sdLoggingEnabled_;
  sdPendingTargetEnabled_ = enable;
  sdCommandPending_ = true;
  sdPendingSinceMs_ = millis();
  sdLoggingEnabled_ = enable;
  updateDashboardActionButtons();

  const bool sent = sendAction(enable ? "sd_start" : "sd_stop", 0);
  if (!sent) {
    sdCommandPending_ = false;
    sdPendingSinceMs_ = 0;
    sdLoggingEnabled_ = sdPendingPreviousEnabled_;
    setCommandStatus("SD command failed", false);
    updateDashboardActionButtons();
    refreshUi();
    return;
  }

  setCommandStatus(String("SD ") + (enable ? "ON" : "OFF") + " requested", true);
  refreshUi();
}

void LvglController::requestTxToggle(bool enable) {
  if (txCommandPending_ || telemetryTxEnabled_ == enable) {
    return;
  }

  txPendingPreviousEnabled_ = telemetryTxEnabled_;
  txPendingTargetEnabled_ = enable;
  txCommandPending_ = true;
  txPendingSinceMs_ = millis();
  telemetryTxEnabled_ = enable;
  updateDashboardActionButtons();

  const bool sent = sendAction(enable ? "telemetry_enable" : "telemetry_disable", 0);
  if (!sent) {
    txCommandPending_ = false;
    txPendingSinceMs_ = 0;
    telemetryTxEnabled_ = txPendingPreviousEnabled_;
    setCommandStatus("TX command failed", false);
    updateDashboardActionButtons();
    refreshUi();
    return;
  }

  setCommandStatus(String("TX ") + (enable ? "ON" : "OFF") + " requested", true);
  refreshUi();
}

void LvglController::setBuzzerConfigVisible(bool visible) {
  buzzerConfigVisible_ = visible;
  if (buzzerConfigRow_ == nullptr) {
    return;
  }

  if (buzzerConfigVisible_) {
    lv_obj_clear_flag(buzzerConfigRow_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(buzzerConfigRow_, LV_OBJ_FLAG_HIDDEN);
  }
  if (actionPanel_ != nullptr) {
    lv_obj_update_layout(actionPanel_);
  }
}

bool LvglController::sendAction(const String& action, int durationS) {
  bool sent = false;
  if (COMPANION_LINK_UART) {
    sent = uart_.sendCommand(action, durationS);
  } else {
    sent = api_.sendCommand(action, durationS);
  }

  String pretty = action;
  pretty.replace("_", " ");
  pretty.toUpperCase();

  if (action == "buzzer") {
    pretty = "BUZZER " + String(durationS) + "s";
  }

  updateDashboardActionButtons();
  setCommandStatus(sent ? (pretty + " sent") : (pretty + " failed"), sent);
  return sent;
}

void LvglController::togglePanel() {
  panelCollapsed_ = !panelCollapsed_;
  if (panelCollapsed_) {
    lv_obj_add_flag(actionPanel_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_clear_flag(actionPanel_, LV_OBJ_FLAG_HIDDEN);
    settingsCollapsed_ = true;
    lv_obj_add_flag(settingsBody_, LV_OBJ_FLAG_HIDDEN);
  }
  lv_obj_update_layout(telemetryPanel_);
}

void LvglController::toggleSettings() {
  settingsCollapsed_ = !settingsCollapsed_;
  if (settingsCollapsed_) {
    lv_obj_add_flag(settingsBody_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_clear_flag(settingsBody_, LV_OBJ_FLAG_HIDDEN);
    panelCollapsed_ = true;
    lv_obj_add_flag(actionPanel_, LV_OBJ_FLAG_HIDDEN);
  }
  lv_obj_update_layout(telemetryPanel_);
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
  calibrationLastRawX_ = -1;
  calibrationLastRawY_ = -1;
  calibrationLastSampleMs_ = 0;
  touchDebugPressed_ = false;
  touchDebugIrqPressed_ = false;
  touchDebugMapOk_ = false;
  touchDebugRawZ_ = -1;
  lv_obj_clear_flag(calibrationOverlay_, LV_OBJ_FLAG_HIDDEN);
  lv_label_set_text(calibrationRawLabel_, "Waiting for touch...");
  advanceCalibrationTarget();
}

void LvglController::cancelCalibration() {
  calibrationActive_ = false;
  calibrationTouchLatch_ = false;
  calibrationStep_ = 0;
  calibrationLastRawX_ = -1;
  calibrationLastRawY_ = -1;
  calibrationLastSampleMs_ = 0;
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

  const uint32_t now = millis();
  int32_t rawX = -1;
  int32_t rawY = -1;
  int32_t rawZ = -1;
  if (!readTouchRaw(rawX, rawY, rawZ)) {
    calibrationTouchLatch_ = false;
    touchDebugPressed_ = false;
    touchDebugMapOk_ = false;
    touchDebugRawX_ = -1;
    touchDebugRawY_ = -1;
    touchDebugRawZ_ = -1;
    return;
  }

  bool allowSample = !calibrationTouchLatch_;
  if (!allowSample && calibrationLastRawX_ >= 0 && calibrationLastRawY_ >= 0) {
    const int32_t dx = abs(rawX - calibrationLastRawX_);
    const int32_t dy = abs(rawY - calibrationLastRawY_);
    const bool movedEnough = (dx >= kCalRetouchDistancePx) || (dy >= kCalRetouchDistancePx);
    allowSample = movedEnough;
  }

  if (!allowSample) {
    return;
  }

  touchDebugPressed_ = true;
  touchDebugRawX_ = rawX;
  touchDebugRawY_ = rawY;
  touchDebugRawZ_ = rawZ;
#if TOUCH_IRQ_PIN >= 0
  touchDebugIrqPressed_ = (digitalRead(TOUCH_IRQ_PIN) == LOW);
#else
  touchDebugIrqPressed_ = false;
#endif
  touchDebugTs_ = now;
  if (calibrationStep_ < kCalPointCount) {
    calibrationRawX_[calibrationStep_] = rawX;
    calibrationRawY_[calibrationStep_] = rawY;

    lv_label_set_text_fmt(calibrationRawLabel_, "Raw sample: %d,%d", static_cast<int>(rawX), static_cast<int>(rawY));

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
  calibrationLastRawX_ = rawX;
  calibrationLastRawY_ = rawY;
  calibrationLastSampleMs_ = now;
}

void LvglController::refreshUi() {
  const uint32_t now = millis();
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
  const String groundVbat = formatFloat(state_.battery.groundVbatV, 2, "--.-");
  const String companionVbat = formatFloat(state_.battery.companionVbatV, 2, "--.-");

  lv_label_set_text_fmt(batteryLabel_, "TX_VBAT: %s V", txVbat.c_str());
  lv_label_set_text_fmt(companionBatteryLabel_, "GS_VBAT: %s V", groundVbat.c_str());
  lv_label_set_text_fmt(companionBatteryDebugLabel_,
                        "BAT_ADC: %s V (%lu mV @ GPIO%d%s)",
                        companionVbat.c_str(),
                        static_cast<unsigned long>(lastCompanionBatRawMv_),
                        static_cast<int>(COMPANION_BAT_ADC_PIN),
                        lastCompanionBatRawValid_ ? "" : " (invalid)");

  String cmdStatus = "";
  if (cmdMsg_.length() > 0 && (now - cmdTs_) <= kCommandStatusShowMs) {
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

  const int touchAgeMs = (touchDebugTs_ == 0) ? -1 : static_cast<int>(now - touchDebugTs_);
#if TOUCH_IRQ_PIN >= 0
  const char* irqState = touchDebugIrqPressed_ ? "LOW" : "HIGH";
#else
  const char* irqState = "NA";
#endif
  lv_label_set_text_fmt(touchDebugLabel_,
                        "TOUCH %s\nIRQ %s\nRAW %d,%d Z%d\nMAP %d,%d\nAGE %dms",
                        touchDebugPressed_ ? "DOWN" : "UP", irqState,
                        static_cast<int>(touchDebugRawX_), static_cast<int>(touchDebugRawY_),
                        static_cast<int>(touchDebugRawZ_), static_cast<int>(touchDebugMapX_),
                        static_cast<int>(touchDebugMapY_), touchAgeMs);
  lv_obj_clear_flag(touchDebugLabel_, LV_OBJ_FLAG_HIDDEN);

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
    syncCommandStateFromTelemetry();

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
  updatePendingCommandTimeouts(now);
  updateStaleness();

  const bool statusVisible = cmdMsg_.length() > 0 && (now - cmdTs_) <= kCommandStatusShowMs;
  if (updated || statusVisible || (now - lastUiRefreshMs_) >= kUiRefreshIntervalMs) {
    refreshUi();
    lastUiRefreshMs_ = now;
  } else {
    lv_timer_handler();
  }
}

void LvglController::flushDisplayCb(lv_display_t* disp, const lv_area_t* area, uint8_t* pxMap) {
  LvglController* self = static_cast<LvglController*>(lv_display_get_user_data(disp));
  if (self == nullptr) {
    lv_display_flush_ready(disp);
    return;
  }
  const uint32_t w = static_cast<uint32_t>(area->x2 - area->x1 + 1);
  const uint32_t h = static_cast<uint32_t>(area->y2 - area->y1 + 1);

  self->tft_.startWrite();
  self->tft_.setAddrWindow(area->x1, area->y1, w, h);
  self->tft_.pushColors(reinterpret_cast<uint16_t*>(pxMap), w * h, true);
  self->tft_.endWrite();

  lv_display_flush_ready(disp);
}

void LvglController::readTouchCb(lv_indev_t* indev, lv_indev_data_t* data) {
  LvglController* self = static_cast<LvglController*>(lv_indev_get_user_data(indev));
  if (self == nullptr) {
    data->state = LV_INDEV_STATE_REL;
    data->point.x = 0;
    data->point.y = 0;
    data->continue_reading = false;
    return;
  }
  data->continue_reading = false;

  if (self->calibrationActive_) {
    self->touchDebugPressed_ = false;
    self->touchDebugMapOk_ = false;
    self->touchDebugRawX_ = -1;
    self->touchDebugRawY_ = -1;
    self->touchDebugRawZ_ = -1;
    self->touchDebugMapX_ = -1;
    self->touchDebugMapY_ = -1;
    data->state = LV_INDEV_STATE_REL;
    data->point.x = 0;
    data->point.y = 0;
    return;
  }

#if TOUCH_IRQ_PIN >= 0
  self->touchDebugIrqPressed_ = (digitalRead(TOUCH_IRQ_PIN) == LOW);
#else
  self->touchDebugIrqPressed_ = false;
#endif

  int32_t rawX = -1;
  int32_t rawY = -1;
  int32_t rawZ = -1;
  const bool touched = self->readTouchRaw(rawX, rawY, rawZ);
  self->touchDebugRawX_ = rawX;
  self->touchDebugRawY_ = rawY;
  self->touchDebugRawZ_ = rawZ;
  self->touchDebugTs_ = millis();

  if (!touched) {
    self->touchDebugPressed_ = false;
    self->touchDebugMapOk_ = false;
    self->touchDebugRawX_ = -1;
    self->touchDebugRawY_ = -1;
    self->touchDebugRawZ_ = -1;
    self->touchDebugMapX_ = -1;
    self->touchDebugMapY_ = -1;
    data->state = LV_INDEV_STATE_REL;
    data->point.x = 0;
    data->point.y = 0;
    return;
  }
  self->touchDebugPressed_ = true;

  int x = 0;
  int y = 0;
  if (!self->mapTouchToScreen(rawX, rawY, x, y)) {
    self->touchDebugMapOk_ = false;
    self->touchDebugMapX_ = -1;
    self->touchDebugMapY_ = -1;
    data->state = LV_INDEV_STATE_REL;
    data->point.x = 0;
    data->point.y = 0;
    return;
  }

  self->touchDebugMapOk_ = true;
  self->touchDebugMapX_ = x;
  self->touchDebugMapY_ = y;

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

void LvglController::onBuzzerToggleEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  self->setBuzzerConfigVisible(!self->buzzerConfigVisible_);
}

void LvglController::onBuzzerDurationChangedEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self->buzzerDurationSlider_ == nullptr) {
    return;
  }
  int value = lv_slider_get_value(self->buzzerDurationSlider_);
  if (value < 1) {
    value = 1;
  } else if (value > 10) {
    value = 10;
  }
  self->buzzerDurationS_ = static_cast<uint8_t>(value);
  if (self->buzzerDurationLabel_ != nullptr) {
    lv_label_set_text_fmt(self->buzzerDurationLabel_, "%us", static_cast<unsigned>(self->buzzerDurationS_));
  }
}

void LvglController::onBuzzerSendEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  self->sendAction("buzzer", self->buzzerDurationS_);
  self->refreshUi();
}

void LvglController::onSdToggleEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  const lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_PRESSED) {
    if (!self->sdLoggingEnabled_) {
      self->requestSdToggle(true);
    }
  } else if (code == LV_EVENT_LONG_PRESSED) {
    if (self->sdLoggingEnabled_) {
      self->requestSdToggle(false);
    }
  }
}

void LvglController::onTxToggleEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  const lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_PRESSED) {
    if (!self->telemetryTxEnabled_) {
      self->requestTxToggle(true);
    }
  } else if (code == LV_EVENT_LONG_PRESSED) {
    if (self->telemetryTxEnabled_) {
      self->requestTxToggle(false);
    }
  }
}
