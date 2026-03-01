#include "lvgl_controller.h"

#include <SD.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <SPI.h>
#include <WiFi.h>
#include <string.h>

namespace {

constexpr uint32_t kUiRefreshIntervalMs = 120;
constexpr uint32_t kCommandStatusShowMs = 3000;
constexpr uint32_t kCommandConfirmTimeoutMs = 5000;
constexpr uint32_t kGroundStationOfflineDetectMs = 8000;
constexpr uint8_t kCalPointCount = 4;
constexpr uint8_t kTelemetryTxPowerMinDbm = 2;
constexpr uint8_t kTelemetryTxPowerMaxDbm = 17;
constexpr lv_coord_t kSettingsPanelTopOffsetPx = 40;
constexpr lv_coord_t kSettingsPanelRightInsetPx = 4;
constexpr lv_coord_t kSettingsPanelBottomMarginPx = 8;
constexpr int kCalMarginPx = 24;
constexpr int32_t kCalRetouchDistancePx = 120;
constexpr int32_t kTouchPressThreshold = 140;
constexpr int32_t kTouchReleaseThreshold = 90;
constexpr lv_coord_t kActionPanelGapPx = 10;
constexpr lv_coord_t kActionItemsGapPx = 16;
constexpr lv_coord_t kSettingsButtonHeight = 28;
constexpr lv_coord_t kSettingsStackGapPx = 24;
constexpr lv_coord_t kSettingsRowGapPx = 14;
constexpr int32_t kUiFontSizePx = 14;

extern const uint8_t kEmbeddedSlkscrTtfStart[] asm("_binary_resources_slkscr_ttf_start");
extern const uint8_t kEmbeddedSlkscrTtfEnd[] asm("_binary_resources_slkscr_ttf_end");

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

#ifndef COMPANION_SD_ENABLE
#define COMPANION_SD_ENABLE 0
#endif

#ifndef COMPANION_SD_CS_PIN
#define COMPANION_SD_CS_PIN -1
#endif

#ifndef COMPANION_SD_SPI_FREQ
#define COMPANION_SD_SPI_FREQ 10000000
#endif

#ifndef COMPANION_SD_SCK_PIN
#define COMPANION_SD_SCK_PIN 18
#endif

#ifndef COMPANION_SD_MISO_PIN
#define COMPANION_SD_MISO_PIN 19
#endif

#ifndef COMPANION_SD_MOSI_PIN
#define COMPANION_SD_MOSI_PIN 23
#endif

#ifndef COMPANION_SD_SOUND_DIR
#define COMPANION_SD_SOUND_DIR "/sounds"
#endif

#ifndef COMPANION_AUDIO_ENABLE
#define COMPANION_AUDIO_ENABLE 0
#endif

#ifndef COMPANION_AUDIO_SPK_PIN
#define COMPANION_AUDIO_SPK_PIN -1
#endif

#ifndef COMPANION_AUDIO_PWM_CHANNEL
#define COMPANION_AUDIO_PWM_CHANNEL 0
#endif

#ifndef COMPANION_AUDIO_PWM_FREQ
#define COMPANION_AUDIO_PWM_FREQ 32000
#endif

#ifndef COMPANION_AUDIO_PWM_BITS
#define COMPANION_AUDIO_PWM_BITS 8
#endif

#ifndef COMPANION_SOUND_FILE_POWER_ON
#define COMPANION_SOUND_FILE_POWER_ON "/sounds/power_on.wav"
#endif

#ifndef COMPANION_SOUND_FILE_ARMED
#define COMPANION_SOUND_FILE_ARMED "/sounds/armed.wav"
#endif

#ifndef COMPANION_SOUND_FILE_CALIBRATING
#define COMPANION_SOUND_FILE_CALIBRATING "/sounds/calibrating.wav"
#endif

#ifndef COMPANION_SOUND_FILE_SENSORS_READY
#define COMPANION_SOUND_FILE_SENSORS_READY "/sounds/sensors_ready.wav"
#endif

#ifndef COMPANION_SOUND_FILE_WAITING_FOR_LOCATION_FIX
#define COMPANION_SOUND_FILE_WAITING_FOR_LOCATION_FIX "/sounds/waiting_for_location_fix.wav"
#endif

#ifndef COMPANION_SOUND_FILE_LOCATION_FIX_ACQUIRED
#define COMPANION_SOUND_FILE_LOCATION_FIX_ACQUIRED "/sounds/location_fix_acquired.wav"
#endif

#ifndef COMPANION_SOUND_FILE_LAUNCH_DETECT_MODE
#define COMPANION_SOUND_FILE_LAUNCH_DETECT_MODE "/sounds/launch_detect_mode.wav"
#endif

#ifndef COMPANION_SOUND_FILE_LAUNCH_DETECTED
#define COMPANION_SOUND_FILE_LAUNCH_DETECTED "/sounds/launch_detected.wav"
#endif

#ifndef COMPANION_SOUND_FILE_APOGEE
#define COMPANION_SOUND_FILE_APOGEE "/sounds/apogee.wav"
#endif

#ifndef COMPANION_SOUND_FILE_DROGUE_DEPLOYED
#define COMPANION_SOUND_FILE_DROGUE_DEPLOYED "/sounds/drogue_deployed.wav"
#endif

#ifndef COMPANION_SOUND_FILE_MAIN_DEPLOYED
#define COMPANION_SOUND_FILE_MAIN_DEPLOYED "/sounds/main_deployed.wav"
#endif

#ifndef COMPANION_SOUND_FILE_LANDING_DETECTED
#define COMPANION_SOUND_FILE_LANDING_DETECTED "/sounds/landing_detected.wav"
#endif

#ifndef COMPANION_SOUND_FILE_HOME_POINT_SET
#define COMPANION_SOUND_FILE_HOME_POINT_SET "/sounds/home_point_set.wav"
#endif

#ifndef COMPANION_SOUND_FILE_AUDIO_TEST
#define COMPANION_SOUND_FILE_AUDIO_TEST "/sounds/audio_test.wav"
#endif

constexpr const char* kFallbackSoundFilePowerOn = "/sounds/power_on.wav";
constexpr const char* kFallbackSoundFileArmed = "/sounds/armed.wav";
constexpr const char* kFallbackSoundFileCalibrating = "/sounds/calibrating.wav";
constexpr const char* kFallbackSoundFileSensorsReady = "/sounds/sensors_ready.wav";
constexpr const char* kFallbackSoundFileWaitingForFix = "/sounds/waiting_for_location_fix.wav";
constexpr const char* kFallbackSoundFileLocationFixAcquired = "/sounds/location_fix_acquired.wav";
constexpr const char* kFallbackSoundFileLaunchDetectMode = "/sounds/launch_detect_mode.wav";
constexpr const char* kFallbackSoundFileLaunchDetected = "/sounds/launch_detected.wav";
constexpr const char* kFallbackSoundFileApogee = "/sounds/apogee.wav";
constexpr const char* kFallbackSoundFileDrogue = "/sounds/drogue_deployed.wav";
constexpr const char* kFallbackSoundFileMain = "/sounds/main_deployed.wav";
constexpr const char* kFallbackSoundFileMainLegacyDir = "/sound/main_deployed.wav";
constexpr const char* kFallbackSoundFileLanding = "/sounds/landing_detected.wav";
constexpr const char* kFallbackSoundFileHomePointSet = "/sounds/home_point_set.wav";

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

static String formatClockText(uint32_t totalMs) {
  const uint32_t totalSeconds = totalMs / 1000U;
  const uint32_t day = totalSeconds / 86400U;
  const uint32_t hour = (totalSeconds % 86400U) / 3600U;
  const uint32_t minute = (totalSeconds % 3600U) / 60U;
  const uint32_t second = totalSeconds % 60U;
  char buf[32];
  snprintf(buf, sizeof(buf), "D%lu %02lu:%02lu:%02lu",
           static_cast<unsigned long>(day),
           static_cast<unsigned long>(hour),
           static_cast<unsigned long>(minute),
           static_cast<unsigned long>(second));
  return String(buf);
}

static String formatDurationText(uint32_t durationMs) {
  const uint32_t totalSeconds = durationMs / 1000U;
  const uint32_t hours = totalSeconds / 3600U;
  const uint32_t minutes = (totalSeconds % 3600U) / 60U;
  const uint32_t seconds = totalSeconds % 60U;
  char buf[20];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu",
           static_cast<unsigned long>(hours),
           static_cast<unsigned long>(minutes),
           static_cast<unsigned long>(seconds));
  return String(buf);
}

static bool phaseIndicatesInFlight(const String& phaseText) {
  String phase = phaseText;
  phase.toLowerCase();
  return phase == "ascent" || phase == "descent" || phase == "boost" || phase == "coast";
}

static bool phaseEquals(const String& phaseText, const char* expectedLower) {
  String phase = phaseText;
  phase.toLowerCase();
  return phase == expectedLower;
}

static int8_t phaseProgressStep(const String& phaseText) {
  String phase = phaseText;
  phase.toLowerCase();
  if (phase == "idle") {
    return 0;
  }
  if (phase == "boost" || phase == "coast" || phase == "ascent") {
    return 1;
  }
  if (phase == "descent") {
    return 2;
  }
  if (phase == "landed") {
    return 3;
  }
  return -1;
}

static int8_t phaseChecklistIndex(const String& phaseText) {
  String phase = phaseText;
  phase.toLowerCase();
  if (phase == "idle" || phase == "pad") {
    return 0;
  }
  if (phase == "boost" || phase == "ascent") {
    return 1;
  }
  if (phase == "coast") {
    return 2;
  }
  if (phase == "descent") {
    return 3;
  }
  if (phase == "landed") {
    return 4;
  }
  return -1;
}

static const char* checklistMarkerForStage(int8_t currentPhaseIndex, int8_t stageIndex) {
  if (currentPhaseIndex < 0) {
    return "[ ]";
  }
  if (stageIndex < currentPhaseIndex) {
    return "[x]";
  }
  if (stageIndex == currentPhaseIndex) {
    return "[>]";
  }
  return "[ ]";
}

static lv_obj_t* makeActionButton(lv_obj_t* parent,
                                  const char* text,
                                  lv_event_cb_t cb,
                                  void* userData,
                                  lv_coord_t height = 34,
                                  lv_event_code_t eventCode = LV_EVENT_PRESSED) {
  lv_obj_t* btn = lv_btn_create(parent);
  lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_width(btn, LV_PCT(100));
  lv_obj_set_height(btn, height);
  lv_obj_set_style_radius(btn, 8, 0);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x1f2a3b), 0);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x2d4f86), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(btn, lv_color_hex(0x4b7dd1), 0);
  lv_obj_set_style_border_width(btn, 1, 0);
  lv_obj_add_event_cb(btn, cb, eventCode, userData);

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

void LvglController::initUiFont() {
#if LV_USE_TINY_TTF
  const size_t fontDataLen = static_cast<size_t>(kEmbeddedSlkscrTtfEnd - kEmbeddedSlkscrTtfStart);
  if (fontDataLen == 0) {
    uiFont_ = nullptr;
    return;
  }

  uiFont_ = lv_tiny_ttf_create_data(kEmbeddedSlkscrTtfStart, fontDataLen, kUiFontSizePx);
  if (uiFont_ != nullptr) {
    uiFont_->fallback = LV_FONT_DEFAULT;
  }
#else
  uiFont_ = nullptr;
#endif
}

const lv_font_t* LvglController::uiTextFont() const {
  return (uiFont_ != nullptr) ? uiFont_ : LV_FONT_DEFAULT;
}

void LvglController::buildUi() {
  lv_obj_t* screen = lv_scr_act();
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x050a14), 0);
  lv_obj_set_style_bg_grad_color(screen, lv_color_hex(0x0d1626), 0);
  lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_text_font(screen, uiTextFont(), 0);

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
  lv_obj_set_width(altitudeLabel_, 210);
  lv_label_set_long_mode(altitudeLabel_, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(altitudeLabel_, lv_color_hex(0xffde6a), 0);
  lv_obj_align(altitudeLabel_, LV_ALIGN_TOP_LEFT, 0, 96);

  //vsLabel_ = lv_label_create(telemetryPanel_);
  //lv_obj_set_style_text_color(vsLabel_, lv_color_hex(0x9df5b3), 0);
  //lv_obj_align(vsLabel_, LV_ALIGN_TOP_LEFT, 0, 164);

  packetLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(packetLabel_, lv_color_hex(0xd9e3f8), 0);
  lv_obj_add_flag(packetLabel_, LV_OBJ_FLAG_HIDDEN);

  phaseChecklistLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_width(phaseChecklistLabel_, 166);
  lv_label_set_long_mode(phaseChecklistLabel_, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_font(phaseChecklistLabel_, uiTextFont(), 0);
  lv_obj_set_style_text_color(phaseChecklistLabel_, lv_color_hex(0xb8c9e6), 0);
  lv_obj_set_style_text_align(phaseChecklistLabel_, LV_TEXT_ALIGN_LEFT, 0);
  lv_obj_clear_flag(phaseChecklistLabel_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_align(phaseChecklistLabel_, LV_ALIGN_TOP_RIGHT, 4, 82);

  callsignLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_width(callsignLabel_, 220);
  lv_label_set_long_mode(callsignLabel_, LV_LABEL_LONG_DOT);
  lv_obj_set_style_text_color(callsignLabel_, lv_color_hex(0xeaf1ff), 0);
  lv_obj_set_style_text_font(callsignLabel_, uiTextFont(), 0);
  lv_obj_align(callsignLabel_, LV_ALIGN_TOP_LEFT, 0, 24);

  lv_obj_align(linkMetaLabel_, LV_ALIGN_TOP_LEFT, 0, 54);
  lv_obj_align(phaseLabel_, LV_ALIGN_TOP_LEFT, 0, 82);
  lv_obj_align(altitudeLabel_, LV_ALIGN_TOP_LEFT, 0, 112);

  batteryLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_width(batteryLabel_, 250);
  lv_label_set_long_mode(batteryLabel_, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(batteryLabel_, lv_color_hex(0xd9e3f8), 0);
  lv_obj_align(batteryLabel_, LV_ALIGN_BOTTOM_LEFT, 0, -20);

  companionBatteryLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(companionBatteryLabel_, lv_color_hex(0xd9e3f8), 0);
  lv_obj_add_flag(companionBatteryLabel_, LV_OBJ_FLAG_HIDDEN);

  companionBatteryDebugLabel_ = lv_label_create(telemetryPanel_);
  lv_obj_set_style_text_color(companionBatteryDebugLabel_, lv_color_hex(0x9fb0cc), 0);
  lv_obj_align(companionBatteryDebugLabel_, LV_ALIGN_BOTTOM_LEFT, 0, -8);
  lv_obj_add_flag(companionBatteryDebugLabel_, LV_OBJ_FLAG_HIDDEN);

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
  if (!touchDebugVisible_) {
    lv_obj_add_flag(touchDebugLabel_, LV_OBJ_FLAG_HIDDEN);
  }

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

  wifiIndicator_ = lv_obj_create(menuIconRow);
  lv_obj_set_size(wifiIndicator_, 30, 30);
  lv_obj_set_style_bg_opa(wifiIndicator_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(wifiIndicator_, 0, 0);
  lv_obj_set_style_pad_all(wifiIndicator_, 0, 0);
  lv_obj_clear_flag(wifiIndicator_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(wifiIndicator_, LV_OBJ_FLAG_CLICKABLE);

  wifiIndicatorIcon_ = lv_label_create(wifiIndicator_);
  lv_label_set_text(wifiIndicatorIcon_, LV_SYMBOL_WIFI);
  lv_obj_set_style_text_color(wifiIndicatorIcon_, lv_color_hex(0xaab8cf), 0);
  lv_obj_clear_flag(wifiIndicatorIcon_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_align(wifiIndicatorIcon_, LV_ALIGN_CENTER, 0, -1);

  wifiIndicatorState_ = lv_label_create(wifiIndicator_);
  lv_label_set_text(wifiIndicatorState_, LV_SYMBOL_CLOSE);
  lv_obj_set_style_text_color(wifiIndicatorState_, lv_color_hex(0x2d0e0e), 0);
  lv_obj_set_style_text_align(wifiIndicatorState_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_bg_opa(wifiIndicatorState_, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(wifiIndicatorState_, lv_color_hex(0xff9c9c), 0);
  lv_obj_set_style_border_width(wifiIndicatorState_, 1, 0);
  lv_obj_set_style_border_color(wifiIndicatorState_, lv_color_hex(0xffd2d2), 0);
  lv_obj_set_style_radius(wifiIndicatorState_, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_pad_hor(wifiIndicatorState_, 2, 0);
  lv_obj_set_style_pad_ver(wifiIndicatorState_, 1, 0);
  lv_obj_clear_flag(wifiIndicatorState_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_align(wifiIndicatorState_, LV_ALIGN_BOTTOM_RIGHT, 0, 0);

  launchPrepBtn_ = lv_btn_create(menuIconRow);
  lv_obj_add_flag(launchPrepBtn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(launchPrepBtn_, 146, 30);
  lv_obj_set_style_radius(launchPrepBtn_, 8, 0);
  lv_obj_add_event_cb(launchPrepBtn_, onLaunchPrepEvent, LV_EVENT_PRESSED, this);
  lv_obj_add_event_cb(launchPrepBtn_, onLaunchPrepEvent, LV_EVENT_LONG_PRESSED, this);
  launchPrepLabel_ = lv_label_create(launchPrepBtn_);
  lv_obj_set_style_text_color(launchPrepLabel_, lv_color_hex(0xe6eeff), 0);
  lv_obj_clear_flag(launchPrepLabel_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(launchPrepLabel_);

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
  lv_obj_set_style_pad_gap(actionPanel_, kActionPanelGapPx, 0);
  lv_obj_set_flex_flow(actionPanel_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(actionPanel_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_clear_flag(actionPanel_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(actionPanel_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_align(actionPanel_, LV_ALIGN_TOP_RIGHT, -kSettingsPanelRightInsetPx, kSettingsPanelTopOffsetPx);

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
  lv_obj_set_style_pad_gap(actionContent_, kActionItemsGapPx, 0);
  lv_obj_set_flex_flow(actionContent_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(actionContent_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_clear_flag(actionContent_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(actionContent_, LV_OBJ_FLAG_CLICKABLE);

  sdToggleBtn_ = makeActionButton(actionContent_, "SD OFF", onSdToggleEvent, this);
  lv_obj_add_event_cb(sdToggleBtn_, onSdToggleEvent, LV_EVENT_LONG_PRESSED, this);
  sdToggleLabel_ = lv_obj_get_child(sdToggleBtn_, 0);

  txToggleBtn_ = makeActionButton(actionContent_, "TX OFF", onTxToggleEvent, this);
  lv_obj_add_event_cb(txToggleBtn_, onTxToggleEvent, LV_EVENT_LONG_PRESSED, this);
  txToggleLabel_ = lv_obj_get_child(txToggleBtn_, 0);

  armBtn_ = makeActionButton(actionContent_, "ARM LAUNCH", onArmEvent, this);
  armLabel_ = lv_obj_get_child(armBtn_, 0);

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
  lv_obj_set_width(buzzerDurationSlider_, 100);
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
  lv_obj_set_size(buzzerSendBtn, 52, 30);
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

  // Resolve percentage/flex layout before deriving pixel height for settings panel.
  lv_obj_update_layout(root_);

  settingsBody_ = lv_obj_create(telemetryPanel_);
  lv_obj_set_width(settingsBody_, kSettingsPanelWidth);
  const lv_coord_t telemetryContentHeight =
      lv_obj_get_height(telemetryPanel_) -
      lv_obj_get_style_pad_top(telemetryPanel_, LV_PART_MAIN) -
      lv_obj_get_style_pad_bottom(telemetryPanel_, LV_PART_MAIN);
  const lv_coord_t settingsBodyHeight =
      telemetryContentHeight - kSettingsPanelTopOffsetPx - kSettingsPanelBottomMarginPx;
  lv_obj_set_height(settingsBody_, settingsBodyHeight > 0 ? settingsBodyHeight : 0);
  lv_obj_set_style_bg_color(settingsBody_, lv_color_hex(0x111c2e), 0);
  lv_obj_set_style_border_color(settingsBody_, lv_color_hex(0x2a446a), 0);
  lv_obj_set_style_border_width(settingsBody_, 1, 0);
  lv_obj_set_style_radius(settingsBody_, 10, 0);
  lv_obj_set_style_pad_top(settingsBody_, 6, 0);
  lv_obj_set_style_pad_bottom(settingsBody_, 6, 0);
  lv_obj_set_style_pad_left(settingsBody_, 10, 0);
  lv_obj_set_style_pad_right(settingsBody_, 10, 0);
  lv_obj_set_style_pad_gap(settingsBody_, kSettingsStackGapPx, 0);
  lv_obj_set_flex_flow(settingsBody_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(settingsBody_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_clear_flag(settingsBody_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(settingsBody_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_align(settingsBody_, LV_ALIGN_TOP_RIGHT, -kSettingsPanelRightInsetPx, kSettingsPanelTopOffsetPx);

  lv_obj_t* settingsTitle = lv_label_create(settingsBody_);
  lv_label_set_text(settingsTitle, "SETTINGS");
  lv_obj_set_style_text_color(settingsTitle, lv_color_hex(0xe6eeff), 0);
  lv_obj_clear_flag(settingsTitle, LV_OBJ_FLAG_CLICKABLE);

  // lv_obj_t* settingsInfo = lv_label_create(settingsBody_);
  // lv_label_set_text(settingsInfo, "Touch calibration\nstored in flash");
  // lv_obj_set_style_text_color(settingsInfo, lv_color_hex(0xc2d4f4), 0);
  // lv_obj_clear_flag(settingsInfo, LV_OBJ_FLAG_CLICKABLE);

  settingsActions_ = lv_obj_create(settingsBody_);
  lv_obj_set_width(settingsActions_, LV_PCT(100));
  lv_obj_set_height(settingsActions_, 0);
  lv_obj_set_flex_grow(settingsActions_, 1);
  lv_obj_set_style_bg_opa(settingsActions_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(settingsActions_, 0, 0);
  lv_obj_set_style_pad_all(settingsActions_, 0, 0);
  lv_obj_set_style_pad_gap(settingsActions_, kSettingsStackGapPx, 0);
  lv_obj_set_flex_flow(settingsActions_, LV_FLEX_FLOW_COLUMN);
  lv_obj_add_flag(settingsActions_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scroll_dir(settingsActions_, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(settingsActions_, LV_SCROLLBAR_MODE_AUTO);

  resetFlightBtn_ = lv_btn_create(settingsActions_);
  lv_obj_add_flag(resetFlightBtn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_width(resetFlightBtn_, LV_PCT(100));
  lv_obj_set_height(resetFlightBtn_, kSettingsButtonHeight);
  lv_obj_set_style_radius(resetFlightBtn_, 8, 0);
  lv_obj_set_style_bg_color(resetFlightBtn_, lv_color_hex(0x7a1d1d), 0);
  lv_obj_set_style_bg_color(resetFlightBtn_, lv_color_hex(0xa02828), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(resetFlightBtn_, lv_color_hex(0xdc7070), 0);
  lv_obj_set_style_border_width(resetFlightBtn_, 1, 0);
  lv_obj_add_event_cb(resetFlightBtn_, onPhaseResetEvent, LV_EVENT_PRESSED, this);
  lv_obj_add_event_cb(resetFlightBtn_, onPhaseResetEvent, LV_EVENT_PRESSING, this);
  lv_obj_add_event_cb(resetFlightBtn_, onPhaseResetEvent, LV_EVENT_RELEASED, this);

  resetFlightLabel_ = lv_label_create(resetFlightBtn_);
  lv_label_set_text(resetFlightLabel_, "HOLD: RESET FLIGHT");
  lv_obj_set_style_text_color(resetFlightLabel_, lv_color_hex(0xfff0f0), 0);
  lv_obj_clear_flag(resetFlightLabel_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(resetFlightLabel_);

  makeActionButton(settingsActions_,
                   "HOLD: IMU CALIBRATION",
                   onImuCalibrateEvent,
                   this,
                   kSettingsButtonHeight,
                   LV_EVENT_LONG_PRESSED);
  makeActionButton(settingsActions_,
                   "SD CARD FUNCTIONS",
                   onSdFunctionsOpenEvent,
                   this,
                   kSettingsButtonHeight);
  makeActionButton(settingsActions_,
                   "SOUND SETTINGS",
                   onSoundSettingsOpenEvent,
                   this,
                   kSettingsButtonHeight);

  lv_obj_t* txPowerTitle = lv_label_create(settingsActions_);
  lv_label_set_text(txPowerTitle, "TELEMETRY TX POWER");
  lv_obj_set_style_text_color(txPowerTitle, lv_color_hex(0xcfe0ff), 0);
  lv_obj_clear_flag(txPowerTitle, LV_OBJ_FLAG_CLICKABLE);

  txPowerConfigRow_ = lv_obj_create(settingsActions_);
  lv_obj_set_width(txPowerConfigRow_, LV_PCT(100));
  lv_obj_set_height(txPowerConfigRow_, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(txPowerConfigRow_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(txPowerConfigRow_, 0, 0);
  lv_obj_set_style_pad_all(txPowerConfigRow_, 0, 0);
  lv_obj_set_style_pad_gap(txPowerConfigRow_, kSettingsRowGapPx, 0);
  lv_obj_set_flex_flow(txPowerConfigRow_, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(txPowerConfigRow_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_clear_flag(txPowerConfigRow_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(txPowerConfigRow_, LV_OBJ_FLAG_CLICKABLE);

  txPowerSlider_ = lv_slider_create(txPowerConfigRow_);
  lv_obj_set_width(txPowerSlider_, 1);
  lv_obj_set_flex_grow(txPowerSlider_, 1);
  lv_slider_set_range(txPowerSlider_, kTelemetryTxPowerMinDbm, kTelemetryTxPowerMaxDbm);
  lv_slider_set_value(txPowerSlider_, txPowerDbm_, LV_ANIM_OFF);
  lv_obj_add_event_cb(txPowerSlider_, onTxPowerChangedEvent, LV_EVENT_VALUE_CHANGED, this);

  txPowerLabel_ = lv_label_create(txPowerConfigRow_);
  lv_obj_set_width(txPowerLabel_, 58);
  lv_label_set_long_mode(txPowerLabel_, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_align(txPowerLabel_, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_set_style_text_color(txPowerLabel_, lv_color_hex(0xcfe0ff), 0);
  lv_obj_clear_flag(txPowerLabel_, LV_OBJ_FLAG_CLICKABLE);
  lv_label_set_text_fmt(txPowerLabel_, "%udBm", static_cast<unsigned>(txPowerDbm_));

  lv_obj_t* txPowerSendBtn = lv_btn_create(txPowerConfigRow_);
  lv_obj_add_flag(txPowerSendBtn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(txPowerSendBtn, 52, kSettingsButtonHeight);
  lv_obj_set_style_radius(txPowerSendBtn, 8, 0);
  lv_obj_set_style_bg_color(txPowerSendBtn, lv_color_hex(0x1f2a3b), 0);
  lv_obj_set_style_bg_color(txPowerSendBtn, lv_color_hex(0x2d4f86), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(txPowerSendBtn, lv_color_hex(0x4b7dd1), 0);
  lv_obj_set_style_border_width(txPowerSendBtn, 1, 0);
  lv_obj_add_event_cb(txPowerSendBtn, onTxPowerSendEvent, LV_EVENT_PRESSED, this);

  lv_obj_t* txPowerSendLabel = lv_label_create(txPowerSendBtn);
  lv_label_set_text(txPowerSendLabel, "APPLY");
  lv_obj_set_style_text_color(txPowerSendLabel, lv_color_hex(0xeaf1ff), 0);
  lv_obj_clear_flag(txPowerSendLabel, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(txPowerSendLabel);

  txPowerActiveLabel_ = lv_label_create(settingsActions_);
  lv_obj_set_width(txPowerActiveLabel_, LV_PCT(100));
  lv_obj_set_style_text_color(txPowerActiveLabel_, lv_color_hex(0x9fb7df), 0);
  lv_obj_clear_flag(txPowerActiveLabel_, LV_OBJ_FLAG_CLICKABLE);
  lv_label_set_text(txPowerActiveLabel_, "Active: -- dBm");

  lv_obj_t* touchDebugCheckbox = lv_checkbox_create(settingsActions_);
  lv_checkbox_set_text(touchDebugCheckbox, "Touchscreen debug");
  lv_obj_set_width(touchDebugCheckbox, LV_PCT(100));
  lv_obj_set_style_text_color(touchDebugCheckbox, lv_color_hex(0xeaf1ff), 0);
  lv_obj_set_style_text_color(touchDebugCheckbox, lv_color_hex(0xeaf1ff), LV_STATE_CHECKED);
  if (touchDebugVisible_) {
    lv_obj_add_state(touchDebugCheckbox, LV_STATE_CHECKED);
  } else {
    lv_obj_clear_state(touchDebugCheckbox, LV_STATE_CHECKED);
  }
  lv_obj_add_event_cb(touchDebugCheckbox, onTouchDebugToggleEvent, LV_EVENT_VALUE_CHANGED, this);

  armNoGpsCheckbox_ = lv_checkbox_create(settingsActions_);
  lv_checkbox_set_text(armNoGpsCheckbox_, "Allow arm without GPS fix");
  lv_obj_set_width(armNoGpsCheckbox_, LV_PCT(100));
  lv_obj_set_style_text_color(armNoGpsCheckbox_, lv_color_hex(0xeaf1ff), 0);
  lv_obj_set_style_text_color(armNoGpsCheckbox_, lv_color_hex(0xeaf1ff), LV_STATE_CHECKED);
  if (allowArmWithoutGpsFix_) {
    lv_obj_add_state(armNoGpsCheckbox_, LV_STATE_CHECKED);
  } else {
    lv_obj_clear_state(armNoGpsCheckbox_, LV_STATE_CHECKED);
  }
  lv_obj_add_event_cb(armNoGpsCheckbox_, onArmNoGpsToggleEvent, LV_EVENT_VALUE_CHANGED, this);

  wifiApToggleBtn_ = lv_btn_create(settingsActions_);
  lv_obj_add_flag(wifiApToggleBtn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_width(wifiApToggleBtn_, LV_PCT(100));
  lv_obj_set_height(wifiApToggleBtn_, kSettingsButtonHeight);
  lv_obj_set_style_radius(wifiApToggleBtn_, 8, 0);
  lv_obj_set_style_bg_color(wifiApToggleBtn_, lv_color_hex(0x244374), 0);
  lv_obj_set_style_bg_color(wifiApToggleBtn_, lv_color_hex(0x2e5ca0), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(wifiApToggleBtn_, lv_color_hex(0x6ea4ff), 0);
  lv_obj_set_style_border_width(wifiApToggleBtn_, 1, 0);
  lv_obj_add_event_cb(wifiApToggleBtn_, onWifiApToggleEvent, LV_EVENT_LONG_PRESSED, this);

  wifiApToggleLabel_ = lv_label_create(wifiApToggleBtn_);
  lv_label_set_text(wifiApToggleLabel_, "HOLD: TOGGLE AP");
  lv_obj_set_style_text_color(wifiApToggleLabel_, lv_color_hex(0xeaf1ff), 0);
  lv_obj_clear_flag(wifiApToggleLabel_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(wifiApToggleLabel_);

  makeActionButton(settingsActions_,
                   "SCREEN CALIBRATION",
                   onCalibrateEvent,
                   this,
                   kSettingsButtonHeight);

  rebootBtn_ = lv_btn_create(settingsActions_);
  lv_obj_add_flag(rebootBtn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_width(rebootBtn_, LV_PCT(100));
  lv_obj_set_height(rebootBtn_, kSettingsButtonHeight);
  lv_obj_set_style_radius(rebootBtn_, 8, 0);
  lv_obj_set_style_bg_color(rebootBtn_, lv_color_hex(0x6a4a1f), 0);
  lv_obj_set_style_bg_color(rebootBtn_, lv_color_hex(0x8b6127), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(rebootBtn_, lv_color_hex(0xe3b36d), 0);
  lv_obj_set_style_border_width(rebootBtn_, 1, 0);
  lv_obj_add_event_cb(rebootBtn_, onRebootEvent, LV_EVENT_LONG_PRESSED, this);

  lv_obj_t* rebootLabel = lv_label_create(rebootBtn_);
  lv_label_set_text(rebootLabel, "HOLD: REBOOT PI");
  lv_obj_set_style_text_color(rebootLabel, lv_color_hex(0xfff0e0), 0);
  lv_obj_clear_flag(rebootLabel, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(rebootLabel);

  shutdownBtn_ = lv_btn_create(settingsActions_);
  lv_obj_add_flag(shutdownBtn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_width(shutdownBtn_, LV_PCT(100));
  lv_obj_set_height(shutdownBtn_, kSettingsButtonHeight);
  lv_obj_set_style_radius(shutdownBtn_, 8, 0);
  lv_obj_set_style_bg_color(shutdownBtn_, lv_color_hex(0x7a1d1d), 0);
  lv_obj_set_style_bg_color(shutdownBtn_, lv_color_hex(0xa02828), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(shutdownBtn_, lv_color_hex(0xdc7070), 0);
  lv_obj_set_style_border_width(shutdownBtn_, 1, 0);
  lv_obj_add_event_cb(shutdownBtn_, onShutdownEvent, LV_EVENT_LONG_PRESSED, this);

  lv_obj_t* shutdownLabel = lv_label_create(shutdownBtn_);
  lv_label_set_text(shutdownLabel, "HOLD: SHUTDOWN PI");
  lv_obj_set_style_text_color(shutdownLabel, lv_color_hex(0xfff0f0), 0);
  lv_obj_clear_flag(shutdownLabel, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(shutdownLabel);

  soundSettingsPanel_ = lv_obj_create(settingsBody_);
  lv_obj_set_width(soundSettingsPanel_, LV_PCT(100));
  lv_obj_set_height(soundSettingsPanel_, 0);
  lv_obj_set_flex_grow(soundSettingsPanel_, 1);
  lv_obj_set_style_bg_opa(soundSettingsPanel_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(soundSettingsPanel_, 0, 0);
  lv_obj_set_style_pad_all(soundSettingsPanel_, 0, 0);
  lv_obj_set_style_pad_gap(soundSettingsPanel_, kSettingsStackGapPx, 0);
  lv_obj_set_flex_flow(soundSettingsPanel_, LV_FLEX_FLOW_COLUMN);
  lv_obj_add_flag(soundSettingsPanel_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scroll_dir(soundSettingsPanel_, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(soundSettingsPanel_, LV_SCROLLBAR_MODE_AUTO);

  soundEnabledCheckbox_ = lv_checkbox_create(soundSettingsPanel_);
  lv_checkbox_set_text(soundEnabledCheckbox_, "Enable sound cues");
  lv_obj_set_width(soundEnabledCheckbox_, LV_PCT(100));
  lv_obj_set_style_text_color(soundEnabledCheckbox_, lv_color_hex(0xeaf1ff), 0);
  lv_obj_set_style_text_color(soundEnabledCheckbox_, lv_color_hex(0xeaf1ff), LV_STATE_CHECKED);
  if (soundEnabled_) {
    lv_obj_add_state(soundEnabledCheckbox_, LV_STATE_CHECKED);
  } else {
    lv_obj_clear_state(soundEnabledCheckbox_, LV_STATE_CHECKED);
  }
  lv_obj_add_event_cb(soundEnabledCheckbox_, onSoundEnableToggleEvent, LV_EVENT_VALUE_CHANGED, this);

  makeActionButton(soundSettingsPanel_,
                   "BACK",
                   onSoundSettingsBackEvent,
                   this,
                   kSettingsButtonHeight);
  makeActionButton(soundSettingsPanel_,
                   "PLAY TEST SOUND",
                   onSoundTestEvent,
                   this,
                   kSettingsButtonHeight);

  lv_obj_t* soundVolumeTitle = lv_label_create(soundSettingsPanel_);
  lv_label_set_text(soundVolumeTitle, "SOUND VOLUME");
  lv_obj_set_style_text_color(soundVolumeTitle, lv_color_hex(0xcfe0ff), 0);
  lv_obj_clear_flag(soundVolumeTitle, LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t* soundVolumeRow = lv_obj_create(soundSettingsPanel_);
  lv_obj_set_width(soundVolumeRow, LV_PCT(100));
  lv_obj_set_height(soundVolumeRow, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(soundVolumeRow, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(soundVolumeRow, 0, 0);
  lv_obj_set_style_pad_all(soundVolumeRow, 0, 0);
  lv_obj_set_style_pad_gap(soundVolumeRow, kSettingsRowGapPx, 0);
  lv_obj_set_flex_flow(soundVolumeRow, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(soundVolumeRow, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_clear_flag(soundVolumeRow, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(soundVolumeRow, LV_OBJ_FLAG_CLICKABLE);

  soundVolumeSlider_ = lv_slider_create(soundVolumeRow);
  lv_obj_set_width(soundVolumeSlider_, 170);
  lv_slider_set_range(soundVolumeSlider_, 0, 100);
  lv_slider_set_value(soundVolumeSlider_, audioVolumePercent_, LV_ANIM_OFF);
  lv_obj_add_event_cb(soundVolumeSlider_, onSoundVolumeChangedEvent, LV_EVENT_VALUE_CHANGED, this);

  soundVolumeLabel_ = lv_label_create(soundVolumeRow);
  lv_obj_set_width(soundVolumeLabel_, 52);
  lv_obj_set_style_text_align(soundVolumeLabel_, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_set_style_text_color(soundVolumeLabel_, lv_color_hex(0xcfe0ff), 0);
  lv_obj_clear_flag(soundVolumeLabel_, LV_OBJ_FLAG_CLICKABLE);
  lv_label_set_text_fmt(soundVolumeLabel_, "%u%%", static_cast<unsigned>(audioVolumePercent_));

  sdFunctionsPanel_ = lv_obj_create(settingsBody_);
  lv_obj_set_width(sdFunctionsPanel_, LV_PCT(100));
  lv_obj_set_height(sdFunctionsPanel_, 0);
  lv_obj_set_flex_grow(sdFunctionsPanel_, 1);
  lv_obj_set_style_bg_opa(sdFunctionsPanel_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(sdFunctionsPanel_, 0, 0);
  lv_obj_set_style_pad_all(sdFunctionsPanel_, 0, 0);
  lv_obj_set_style_pad_gap(sdFunctionsPanel_, kSettingsStackGapPx, 0);
  lv_obj_set_flex_flow(sdFunctionsPanel_, LV_FLEX_FLOW_COLUMN);
  lv_obj_add_flag(sdFunctionsPanel_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scroll_dir(sdFunctionsPanel_, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(sdFunctionsPanel_, LV_SCROLLBAR_MODE_AUTO);

  makeActionButton(sdFunctionsPanel_,
                   "BACK",
                   onSdFunctionsBackEvent,
                   this,
                   kSettingsButtonHeight);

  sdRotateBtn_ = lv_btn_create(sdFunctionsPanel_);
  lv_obj_add_flag(sdRotateBtn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_width(sdRotateBtn_, LV_PCT(100));
  lv_obj_set_height(sdRotateBtn_, kSettingsButtonHeight);
  lv_obj_set_style_radius(sdRotateBtn_, 8, 0);
  lv_obj_set_style_bg_color(sdRotateBtn_, lv_color_hex(0x1f2a3b), 0);
  lv_obj_set_style_bg_color(sdRotateBtn_, lv_color_hex(0x2d4f86), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(sdRotateBtn_, lv_color_hex(0x4b7dd1), 0);
  lv_obj_set_style_border_width(sdRotateBtn_, 1, 0);
  lv_obj_add_event_cb(sdRotateBtn_, onSdRotateEvent, LV_EVENT_PRESSED, this);

  sdRotateLabel_ = lv_label_create(sdRotateBtn_);
  lv_label_set_text(sdRotateLabel_, "ROTATE LOGFILE");
  lv_obj_set_style_text_color(sdRotateLabel_, lv_color_hex(0xeaf1ff), 0);
  lv_obj_clear_flag(sdRotateLabel_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(sdRotateLabel_);

  sdFormatBtn_ = lv_btn_create(sdFunctionsPanel_);
  lv_obj_add_flag(sdFormatBtn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_width(sdFormatBtn_, LV_PCT(100));
  lv_obj_set_height(sdFormatBtn_, kSettingsButtonHeight);
  lv_obj_set_style_radius(sdFormatBtn_, 8, 0);
  lv_obj_set_style_bg_color(sdFormatBtn_, lv_color_hex(0x7a1d1d), 0);
  lv_obj_set_style_bg_color(sdFormatBtn_, lv_color_hex(0xa02828), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(sdFormatBtn_, lv_color_hex(0xdc7070), 0);
  lv_obj_set_style_border_width(sdFormatBtn_, 1, 0);
  lv_obj_add_event_cb(sdFormatBtn_, onSdFormatEvent, LV_EVENT_PRESSED, this);
  lv_obj_add_event_cb(sdFormatBtn_, onSdFormatEvent, LV_EVENT_LONG_PRESSED, this);

  sdFormatLabel_ = lv_label_create(sdFormatBtn_);
  lv_label_set_text(sdFormatLabel_, "HOLD: FORMAT SD");
  lv_obj_set_style_text_color(sdFormatLabel_, lv_color_hex(0xfff0f0), 0);
  lv_obj_clear_flag(sdFormatLabel_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(sdFormatLabel_);

  sdDumpBtn_ = lv_btn_create(sdFunctionsPanel_);
  lv_obj_add_flag(sdDumpBtn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_width(sdDumpBtn_, LV_PCT(100));
  lv_obj_set_height(sdDumpBtn_, kSettingsButtonHeight);
  lv_obj_set_style_radius(sdDumpBtn_, 8, 0);
  lv_obj_set_style_bg_color(sdDumpBtn_, lv_color_hex(0x1f2a3b), 0);
  lv_obj_set_style_bg_color(sdDumpBtn_, lv_color_hex(0x2d4f86), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(sdDumpBtn_, lv_color_hex(0x4b7dd1), 0);
  lv_obj_set_style_border_width(sdDumpBtn_, 1, 0);
  lv_obj_add_event_cb(sdDumpBtn_, onSdDumpSampleEvent, LV_EVENT_PRESSED, this);

  sdDumpLabel_ = lv_label_create(sdDumpBtn_);
  lv_label_set_text(sdDumpLabel_, "DUMP SAMPLE");
  lv_obj_set_style_text_color(sdDumpLabel_, lv_color_hex(0xeaf1ff), 0);
  lv_obj_clear_flag(sdDumpLabel_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(sdDumpLabel_);

  setSoundSettingsVisible(false);
  setSdFunctionsVisible(false);

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

  calibrationCancelBtn_ = lv_btn_create(calibrationOverlay_);
  lv_obj_set_size(calibrationCancelBtn_, 140, 38);
  lv_obj_set_style_radius(calibrationCancelBtn_, 10, 0);
  lv_obj_set_style_bg_color(calibrationCancelBtn_, lv_color_hex(0x5e2323), 0);
  lv_obj_set_style_bg_color(calibrationCancelBtn_, lv_color_hex(0x8a2f2f), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(calibrationCancelBtn_, lv_color_hex(0xffb3b3), 0);
  lv_obj_set_style_border_width(calibrationCancelBtn_, 1, 0);
  lv_obj_add_event_cb(calibrationCancelBtn_, onCalibrationCancelEvent, LV_EVENT_PRESSED, this);
  lv_obj_align(calibrationCancelBtn_, LV_ALIGN_CENTER, 0, 0);

  lv_obj_t* calibrationCancelLabel = lv_label_create(calibrationCancelBtn_);
  lv_label_set_text(calibrationCancelLabel, "CANCEL");
  lv_obj_set_style_text_color(calibrationCancelLabel, lv_color_hex(0xfff0f0), 0);
  lv_obj_clear_flag(calibrationCancelLabel, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(calibrationCancelLabel);

  lv_obj_add_flag(calibrationOverlay_, LV_OBJ_FLAG_HIDDEN);

  sdDumpOverlay_ = lv_obj_create(screen);
  lv_obj_set_size(sdDumpOverlay_, LV_PCT(100), LV_PCT(100));
  lv_obj_set_style_bg_color(sdDumpOverlay_, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(sdDumpOverlay_, LV_OPA_80, 0);
  lv_obj_set_style_border_width(sdDumpOverlay_, 0, 0);
  lv_obj_set_style_pad_all(sdDumpOverlay_, 0, 0);
  lv_obj_clear_flag(sdDumpOverlay_, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* sdDumpCard = lv_obj_create(sdDumpOverlay_);
  lv_obj_set_size(sdDumpCard, 430, 255);
  lv_obj_align(sdDumpCard, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(sdDumpCard, lv_color_hex(0x0f1728), 0);
  lv_obj_set_style_border_color(sdDumpCard, lv_color_hex(0x2a446a), 0);
  lv_obj_set_style_border_width(sdDumpCard, 1, 0);
  lv_obj_set_style_radius(sdDumpCard, 10, 0);
  lv_obj_set_style_pad_all(sdDumpCard, 10, 0);
  lv_obj_set_style_pad_gap(sdDumpCard, 8, 0);
  lv_obj_set_flex_flow(sdDumpCard, LV_FLEX_FLOW_COLUMN);
  lv_obj_clear_flag(sdDumpCard, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(sdDumpCard, LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t* sdDumpHeader = lv_obj_create(sdDumpCard);
  lv_obj_set_width(sdDumpHeader, LV_PCT(100));
  lv_obj_set_height(sdDumpHeader, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(sdDumpHeader, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(sdDumpHeader, 0, 0);
  lv_obj_set_style_pad_all(sdDumpHeader, 0, 0);
  lv_obj_set_style_pad_gap(sdDumpHeader, 8, 0);
  lv_obj_set_flex_flow(sdDumpHeader, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(sdDumpHeader, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_clear_flag(sdDumpHeader, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(sdDumpHeader, LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t* sdDumpTitle = lv_label_create(sdDumpHeader);
  lv_label_set_text(sdDumpTitle, "SD DUMP SAMPLE");
  lv_obj_set_style_text_color(sdDumpTitle, lv_color_hex(0xeaf1ff), 0);
  lv_obj_clear_flag(sdDumpTitle, LV_OBJ_FLAG_CLICKABLE);

  sdDumpCloseBtn_ = lv_btn_create(sdDumpHeader);
  lv_obj_add_flag(sdDumpCloseBtn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(sdDumpCloseBtn_, 34, 26);
  lv_obj_set_style_radius(sdDumpCloseBtn_, 8, 0);
  lv_obj_set_style_bg_color(sdDumpCloseBtn_, lv_color_hex(0x6a1f1f), 0);
  lv_obj_set_style_bg_color(sdDumpCloseBtn_, lv_color_hex(0x8b2a2a), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(sdDumpCloseBtn_, lv_color_hex(0xe08b8b), 0);
  lv_obj_set_style_border_width(sdDumpCloseBtn_, 1, 0);
  lv_obj_add_event_cb(sdDumpCloseBtn_, onSdDumpCloseEvent, LV_EVENT_PRESSED, this);

  lv_obj_t* sdDumpCloseLabel = lv_label_create(sdDumpCloseBtn_);
  lv_label_set_text(sdDumpCloseLabel, "X");
  lv_obj_set_style_text_color(sdDumpCloseLabel, lv_color_hex(0xfff0f0), 0);
  lv_obj_clear_flag(sdDumpCloseLabel, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_center(sdDumpCloseLabel);

  lv_obj_t* sdDumpBody = lv_obj_create(sdDumpCard);
  lv_obj_set_width(sdDumpBody, LV_PCT(100));
  lv_obj_set_height(sdDumpBody, 190);
  lv_obj_set_style_bg_color(sdDumpBody, lv_color_hex(0x0b1220), 0);
  lv_obj_set_style_border_color(sdDumpBody, lv_color_hex(0x223754), 0);
  lv_obj_set_style_border_width(sdDumpBody, 1, 0);
  lv_obj_set_style_radius(sdDumpBody, 8, 0);
  lv_obj_set_style_pad_all(sdDumpBody, 8, 0);
  lv_obj_set_scroll_dir(sdDumpBody, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(sdDumpBody, LV_SCROLLBAR_MODE_AUTO);

  sdDumpTextLabel_ = lv_label_create(sdDumpBody);
  lv_obj_set_width(sdDumpTextLabel_, LV_PCT(100));
  lv_label_set_long_mode(sdDumpTextLabel_, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(sdDumpTextLabel_, lv_color_hex(0xcfe0ff), 0);
  lv_obj_clear_flag(sdDumpTextLabel_, LV_OBJ_FLAG_CLICKABLE);
  lv_label_set_text(sdDumpTextLabel_, "No sample yet.");

  lv_obj_add_flag(sdDumpOverlay_, LV_OBJ_FLAG_HIDDEN);
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
  soundEnabled_ = prefs_.getBool("sound_enabled", true);
  audioVolumePercent_ = prefs_.getUChar("sound_volume", 100);
  allowArmWithoutGpsFix_ = false;
  prefs_.putBool("arm_no_gps", false);
  if (audioVolumePercent_ > 100) {
    audioVolumePercent_ = 100;
  }
  loadTouchCalibration();

  tft_.init();
  tft_.setRotation(1);
  tft_.fillScreen(TFT_BLACK);
  initSdStorage();
  initSoundOutput();

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
  initUiFont();
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
  hasRecoveryEventHistory_ = state_.hasRecoveryEventState;
  lastRecoveryLaunchDetected_ = state_.recoveryLaunchDetected;
  lastRecoveryApogee_ = state_.recoveryApogee;
  lastRecoveryLandingDetected_ = state_.recoveryLandingDetected;
  lastFlightLaunchDetected_ = false;
  lastFlightLandingDetected_ = false;
  recoveryLaunchArmed_ = state_.recoveryLaunchArmed;
  recoveryGpsFix3d_ = state_.recoveryGpsFix3d;
  lastLvTickMs_ = millis();

  if (lastRxMs_ != 0) {
    hasLastLoraPacketCount_ = true;
    lastLoraPacketCount_ = state_.flight.packetCount;
    if (state_.link.lastPacketAgeMs >= 0) {
      loraAgeBaseMs_ = state_.link.lastPacketAgeMs;
      loraAgeBaseTickMs_ = lastLvTickMs_;
    }
  }

  updateFlightTimerState(lastLvTickMs_);

  refreshUi();
}

void LvglController::requestSdRotate() {
  if (sdUtilityCommandPending_ || sdCommandPending_) {
    return;
  }
  if (!sdLoggingEnabled_) {
    setCommandStatus("Rotate requires SD logging ON", false);
    refreshUi();
    return;
  }

  sdUtilityCommandPending_ = true;
  sdUtilityPendingAction_ = "sd_rotate";
  sdUtilityPendingSinceMs_ = millis();
  updateDashboardActionButtons();

  if (!sendAction("sd_rotate", 0)) {
    sdUtilityCommandPending_ = false;
    sdUtilityPendingAction_ = "";
    sdUtilityPendingSinceMs_ = 0;
    setCommandStatus("SD rotate command failed", false);
    updateDashboardActionButtons();
    refreshUi();
    return;
  }

  setCommandStatus("SD rotate requested", true);
  refreshUi();
}

void LvglController::requestSdFormat() {
  if (sdUtilityCommandPending_ || sdCommandPending_) {
    return;
  }
  if (sdLoggingEnabled_) {
    setCommandStatus("Stop SD logging before format", false);
    refreshUi();
    return;
  }
  if (commandLockoutActive_) {
    setCommandStatus("SD format locked until landing", false);
    refreshUi();
    return;
  }

  sdUtilityCommandPending_ = true;
  sdUtilityPendingAction_ = "sd_format";
  sdUtilityPendingSinceMs_ = millis();
  sdFormatArmed_ = false;
  updateDashboardActionButtons();

  if (!sendAction("sd_format", 0)) {
    sdUtilityCommandPending_ = false;
    sdUtilityPendingAction_ = "";
    sdUtilityPendingSinceMs_ = 0;
    setCommandStatus("SD format command failed", false);
    updateDashboardActionButtons();
    refreshUi();
    return;
  }

  setCommandStatus("SD format requested", true);
  refreshUi();
}

void LvglController::requestSdDumpSample() {
  if (sdUtilityCommandPending_ || sdCommandPending_) {
    return;
  }
  if (sdLoggingEnabled_) {
    setCommandStatus("Stop SD logging before dump", false);
    refreshUi();
    return;
  }

  sdUtilityCommandPending_ = true;
  sdUtilityPendingAction_ = "sd_dump_sample";
  sdUtilityPendingSinceMs_ = millis();
  updateDashboardActionButtons();

  if (!sendAction("sd_dump_sample", 0)) {
    sdUtilityCommandPending_ = false;
    sdUtilityPendingAction_ = "";
    sdUtilityPendingSinceMs_ = 0;
    setCommandStatus("SD dump sample command failed", false);
    updateDashboardActionButtons();
    refreshUi();
    return;
  }

  setCommandStatus("SD dump sample requested", true);
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

void LvglController::initSdStorage() {
  sdStorageReady_ = false;
  sdStorageTotalBytes_ = 0;
  sdStorageUsedBytes_ = 0;

#if COMPANION_SD_ENABLE
#if COMPANION_SD_CS_PIN < 0
  return;
#else
  pinMode(COMPANION_SD_CS_PIN, OUTPUT);
  digitalWrite(COMPANION_SD_CS_PIN, HIGH);

#if defined(ESP32)
  SPIClass* sdBus = &SPI;
  if (COMPANION_SD_SCK_PIN >= 0 && COMPANION_SD_MISO_PIN >= 0 && COMPANION_SD_MOSI_PIN >= 0) {
    sdSpi_.begin(COMPANION_SD_SCK_PIN,
                 COMPANION_SD_MISO_PIN,
                 COMPANION_SD_MOSI_PIN,
                 COMPANION_SD_CS_PIN);
    sdBus = &sdSpi_;
  }

  if (!SD.begin(COMPANION_SD_CS_PIN, *sdBus, COMPANION_SD_SPI_FREQ)) {
    return;
  }
#else
  if (!SD.begin(COMPANION_SD_CS_PIN, SPI, COMPANION_SD_SPI_FREQ)) {
    return;
  }
#endif

  const uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    SD.end();
    return;
  }

  const char* soundDir = COMPANION_SD_SOUND_DIR;
  if (soundDir != nullptr && soundDir[0] == '/' && strlen(soundDir) > 1U && !SD.exists(soundDir)) {
    (void)SD.mkdir(soundDir);
  }

  sdStorageReady_ = true;
  sdStorageTotalBytes_ = SD.totalBytes();
  sdStorageUsedBytes_ = SD.usedBytes();
#endif
#endif
}

void LvglController::initSoundOutput() {
  audioOutputReady_ = false;
  audioPwmMaxDuty_ = 255;

#if COMPANION_AUDIO_ENABLE
#if COMPANION_AUDIO_SPK_PIN >= 0
  const uint8_t pwmBits = (COMPANION_AUDIO_PWM_BITS > 15) ? 15 : COMPANION_AUDIO_PWM_BITS;
  ledcSetup(COMPANION_AUDIO_PWM_CHANNEL, COMPANION_AUDIO_PWM_FREQ, pwmBits);
  ledcAttachPin(COMPANION_AUDIO_SPK_PIN, COMPANION_AUDIO_PWM_CHANNEL);
  const uint32_t maxDuty = (1UL << pwmBits) - 1UL;
  audioPwmMaxDuty_ = static_cast<uint16_t>(maxDuty);
  ledcWrite(COMPANION_AUDIO_PWM_CHANNEL, audioPwmMaxDuty_ / 2U);
  audioOutputReady_ = true;
#endif
#endif
}

void LvglController::queueSoundCue(SoundCue cue) {
  if (!soundEnabled_) {
    return;
  }
  if (soundQueueCount_ >= kSoundQueueCapacity) {
    return;
  }

  if (soundQueueCount_ > 0) {
    const uint8_t lastIdx = static_cast<uint8_t>((soundQueueTail_ + kSoundQueueCapacity - 1) % kSoundQueueCapacity);
    if (soundQueue_[lastIdx] == cue) {
      return;
    }
  }

  soundQueue_[soundQueueTail_] = cue;
  soundQueueTail_ = static_cast<uint8_t>((soundQueueTail_ + 1) % kSoundQueueCapacity);
  soundQueueCount_++;
}

const char* LvglController::cueFilePath(SoundCue cue) const {
  switch (cue) {
    case SoundCue::kPowerOn:
      return COMPANION_SOUND_FILE_POWER_ON;
    case SoundCue::kArmed:
      return COMPANION_SOUND_FILE_ARMED;
    case SoundCue::kCalibrating:
      return COMPANION_SOUND_FILE_CALIBRATING;
    case SoundCue::kSensorsReady:
      return COMPANION_SOUND_FILE_SENSORS_READY;
    case SoundCue::kWaitingForLocationFix:
      return COMPANION_SOUND_FILE_WAITING_FOR_LOCATION_FIX;
    case SoundCue::kLocationFixAcquired:
      return COMPANION_SOUND_FILE_LOCATION_FIX_ACQUIRED;
    case SoundCue::kLaunchDetectMode:
      return COMPANION_SOUND_FILE_LAUNCH_DETECT_MODE;
    case SoundCue::kLaunchDetected:
      return COMPANION_SOUND_FILE_LAUNCH_DETECTED;
    case SoundCue::kApogee:
      return COMPANION_SOUND_FILE_APOGEE;
    case SoundCue::kDrogueDeploy:
      return COMPANION_SOUND_FILE_DROGUE_DEPLOYED;
    case SoundCue::kMainDeploy:
      return COMPANION_SOUND_FILE_MAIN_DEPLOYED;
    case SoundCue::kLandingDetected:
      return COMPANION_SOUND_FILE_LANDING_DETECTED;
    case SoundCue::kHomePointSet:
      return COMPANION_SOUND_FILE_HOME_POINT_SET;
    default:
      return "";
  }
}

void LvglController::handleEventSoundTriggers(const String& previousPhase,
                                              const String& currentPhase,
                                              bool phaseChanged) {
  const float currentAglM = state_.alt.altitudeAglM;
  const int8_t currentPhaseStep = phaseProgressStep(currentPhase);
  const bool preflightPhase = (currentPhaseStep <= 0);

  if (!isnan(currentAglM) && (isnan(maxObservedAglM_) || currentAglM > maxObservedAglM_)) {
    maxObservedAglM_ = currentAglM;
  }

  if (state_.recoveryGpsFix3d != recoveryGpsFix3d_) {
    if (preflightPhase) {
      if (state_.recoveryGpsFix3d) {
        queueSoundCue(SoundCue::kLocationFixAcquired);
      } else if (!allowArmWithoutGpsFix_ && !state_.recoveryLaunchArmed) {
        queueSoundCue(SoundCue::kWaitingForLocationFix);
      }
    }
  }

  if (state_.recoveryLaunchArmed && !recoveryLaunchArmed_) {
    const bool armedWithoutGpsFix = !state_.recoveryGpsFix3d && allowArmWithoutGpsFix_;
    if (!armedWithoutGpsFix) {
      queueSoundCue(SoundCue::kLaunchDetectMode);
    }
  }
  recoveryLaunchArmed_ = state_.recoveryLaunchArmed;
  recoveryGpsFix3d_ = state_.recoveryGpsFix3d;

  if (state_.hasRecoveryEventState) {
    if (hasRecoveryEventHistory_) {
      if (!lastRecoveryLaunchDetected_ && state_.recoveryLaunchDetected) {
        if (soundQueueCount_ > 0) {
          SoundCue filteredQueue[kSoundQueueCapacity] = {};
          uint8_t filteredCount = 0;
          for (uint8_t i = 0; i < soundQueueCount_; ++i) {
            const uint8_t idx = static_cast<uint8_t>((soundQueueHead_ + i) % kSoundQueueCapacity);
            const SoundCue queued = soundQueue_[idx];
            if (queued == SoundCue::kPowerOn || queued == SoundCue::kArmed || queued == SoundCue::kCalibrating ||
                queued == SoundCue::kSensorsReady ||
                queued == SoundCue::kWaitingForLocationFix || queued == SoundCue::kLocationFixAcquired ||
                queued == SoundCue::kLaunchDetectMode) {
              continue;
            }
            if (filteredCount < kSoundQueueCapacity) {
              filteredQueue[filteredCount++] = queued;
            }
          }
          for (uint8_t i = 0; i < filteredCount; ++i) {
            soundQueue_[i] = filteredQueue[i];
          }
          soundQueueHead_ = 0;
          soundQueueTail_ = static_cast<uint8_t>(filteredCount % kSoundQueueCapacity);
          soundQueueCount_ = filteredCount;
        }
        queueSoundCue(SoundCue::kLaunchDetected);
        maxObservedAglM_ = currentAglM;
        apogeeCalloutPending_ = false;
      }

      if (!lastRecoveryApogee_ && state_.recoveryApogee) {
        queueSoundCue(SoundCue::kApogee);
      }

      if (!lastRecoveryLandingDetected_ && state_.recoveryLandingDetected) {
        queueSoundCue(SoundCue::kLandingDetected);
        apogeeCalloutPending_ = true;
        apogeeCalloutReadyAtMs_ = millis() + kApogeeCalloutDelayMs;
      }
    }

    lastRecoveryLaunchDetected_ = state_.recoveryLaunchDetected;
    lastRecoveryApogee_ = state_.recoveryApogee;
    lastRecoveryLandingDetected_ = state_.recoveryLandingDetected;
    hasRecoveryEventHistory_ = true;
  } else if (phaseChanged) {
    if (phaseEquals(currentPhase, "idle") && !phaseEquals(previousPhase, "idle")) {
      soundQueueHead_ = 0;
      soundQueueTail_ = 0;
      soundQueueCount_ = 0;
      queueSoundCue(SoundCue::kPowerOn);
      queueSoundCue(SoundCue::kCalibrating);
      queueSoundCue(SoundCue::kSensorsReady);
      if (state_.recoveryGpsFix3d) {
        queueSoundCue(SoundCue::kLocationFixAcquired);
      } else if (!allowArmWithoutGpsFix_) {
        queueSoundCue(SoundCue::kWaitingForLocationFix);
      }
      // Starting a new flight lifecycle: re-prime deployment edge tracking so
      // main/drogue cues are not skipped if prior flight state was still latched.
      hasRecoveryDeployHistory_ = true;
      lastRecoveryDrogueDeployed_ = false;
      lastRecoveryMainDeployed_ = false;
      hasRecoveryEventHistory_ = true;
      lastRecoveryLaunchDetected_ = false;
      lastRecoveryApogee_ = false;
      lastRecoveryLandingDetected_ = false;
      maxObservedAglM_ = NAN;
      apogeeCalloutPending_ = false;
      apogeeCalloutReadyAtMs_ = 0;
    }

    const bool prevInAscent = phaseEquals(previousPhase, "ascent") || phaseEquals(previousPhase, "boost") ||
                              phaseEquals(previousPhase, "coast");
    const bool currInAscent = phaseEquals(currentPhase, "ascent") || phaseEquals(currentPhase, "boost") ||
                              phaseEquals(currentPhase, "coast");

    if (currInAscent && !prevInAscent) {
      if (soundQueueCount_ > 0) {
        SoundCue filteredQueue[kSoundQueueCapacity] = {};
        uint8_t filteredCount = 0;
        for (uint8_t i = 0; i < soundQueueCount_; ++i) {
          const uint8_t idx = static_cast<uint8_t>((soundQueueHead_ + i) % kSoundQueueCapacity);
          const SoundCue queued = soundQueue_[idx];
          if (queued == SoundCue::kPowerOn || queued == SoundCue::kArmed || queued == SoundCue::kCalibrating ||
              queued == SoundCue::kSensorsReady ||
              queued == SoundCue::kWaitingForLocationFix || queued == SoundCue::kLocationFixAcquired ||
              queued == SoundCue::kLaunchDetectMode) {
            continue;
          }
          if (filteredCount < kSoundQueueCapacity) {
            filteredQueue[filteredCount++] = queued;
          }
        }
        for (uint8_t i = 0; i < filteredCount; ++i) {
          soundQueue_[i] = filteredQueue[i];
        }
        soundQueueHead_ = 0;
        soundQueueTail_ = static_cast<uint8_t>(filteredCount % kSoundQueueCapacity);
        soundQueueCount_ = filteredCount;
      }
      queueSoundCue(SoundCue::kLaunchDetected);
      maxObservedAglM_ = currentAglM;
      apogeeCalloutPending_ = false;
      apogeeCalloutReadyAtMs_ = 0;
    }

    if (phaseEquals(currentPhase, "descent") && prevInAscent) {
      queueSoundCue(SoundCue::kApogee);
    }

    if (phaseEquals(currentPhase, "landed") && !phaseEquals(previousPhase, "landed")) {
      queueSoundCue(SoundCue::kLandingDetected);
      apogeeCalloutPending_ = true;
      apogeeCalloutReadyAtMs_ = millis() + kApogeeCalloutDelayMs;
    }
  }

  if (state_.hasRecoveryDeploymentState) {
    if (hasRecoveryDeployHistory_) {
      if (!lastRecoveryDrogueDeployed_ && state_.recoveryDrogueDeployed) {
        queueSoundCue(SoundCue::kDrogueDeploy);
      }
      if (!lastRecoveryMainDeployed_ && state_.recoveryMainDeployed) {
        queueSoundCue(SoundCue::kMainDeploy);
      }
    }
    lastRecoveryDrogueDeployed_ = state_.recoveryDrogueDeployed;
    lastRecoveryMainDeployed_ = state_.recoveryMainDeployed;
    hasRecoveryDeployHistory_ = true;
  }
}

bool LvglController::playNumberCue(int value) {
  if (!soundEnabled_) {
    return false;
  }
  if (value <= 0) {
    return false;
  }

  auto playExact = [this](int v) {
    const String path = String("/sounds/") + String(v) + String(".wav");
    return playWavFromSd(path.c_str());
  };

  if (value <= 20 ||
      (value < 100 && (value % 10) == 0) ||
      (value < 1000 && (value % 100) == 0) ||
      (value <= 10000 && (value % 1000) == 0)) {
    return playExact(value);
  }

  if (value < 100) {
    const int tens = (value / 10) * 10;
    const int ones = value % 10;
    if (!playNumberCue(tens)) {
      return false;
    }
    if (ones > 0) {
      return playNumberCue(ones);
    }
    return true;
  }

  if (value < 1000) {
    const int hundreds = (value / 100) * 100;
    const int remainder = value % 100;
    if (!playNumberCue(hundreds)) {
      return false;
    }
    if (remainder > 0) {
      return playNumberCue(remainder);
    }
    return true;
  }

  if (value <= 10000) {
    const int thousands = (value / 1000) * 1000;
    const int remainder = value % 1000;
    if (!playNumberCue(thousands)) {
      return false;
    }
    if (remainder > 0) {
      return playNumberCue(remainder);
    }
    return true;
  }

  return false;
}

bool LvglController::playApogeeAltitudeCallout() {
  if (!soundEnabled_) {
    return false;
  }
  if (isnan(maxObservedAglM_) || maxObservedAglM_ <= 0.0f) {
    return false;
  }

  int apogeeM = static_cast<int>(lroundf(maxObservedAglM_));
  if (apogeeM < 1) {
    apogeeM = 1;
  }
  if (apogeeM > 10000) {
    apogeeM = 10000;
  }

  bool played = playWavFromSd(cueFilePath(SoundCue::kApogee));
  if (!played) {
    (void)playWavFromSd(kFallbackSoundFileApogee);
  }

  if (!playNumberCue(apogeeM)) {
    return false;
  }
  return playWavFromSd("/sounds/meters.wav");
}

void LvglController::playNextQueuedSound() {
  if (soundQueueCount_ == 0) {
    return;
  }

  const SoundCue cue = soundQueue_[soundQueueHead_];
  soundQueueHead_ = static_cast<uint8_t>((soundQueueHead_ + 1) % kSoundQueueCapacity);
  soundQueueCount_--;

  bool played = playWavFromSd(cueFilePath(cue));
  if (played) {
    return;
  }

  switch (cue) {
    case SoundCue::kPowerOn:
      (void)playWavFromSd(kFallbackSoundFilePowerOn);
      break;
    case SoundCue::kArmed:
      (void)playWavFromSd(kFallbackSoundFileArmed);
      break;
    case SoundCue::kCalibrating:
      (void)playWavFromSd(kFallbackSoundFileCalibrating);
      break;
    case SoundCue::kSensorsReady:
      (void)playWavFromSd(kFallbackSoundFileSensorsReady);
      break;
    case SoundCue::kWaitingForLocationFix:
      (void)playWavFromSd(kFallbackSoundFileWaitingForFix);
      break;
    case SoundCue::kLocationFixAcquired:
      (void)playWavFromSd(kFallbackSoundFileLocationFixAcquired);
      break;
    case SoundCue::kLaunchDetectMode:
      (void)playWavFromSd(kFallbackSoundFileLaunchDetectMode);
      break;
    case SoundCue::kLaunchDetected:
      (void)playWavFromSd(kFallbackSoundFileLaunchDetected);
      break;
    case SoundCue::kApogee:
      (void)playWavFromSd(kFallbackSoundFileApogee);
      break;
    case SoundCue::kDrogueDeploy:
      (void)playWavFromSd(kFallbackSoundFileDrogue);
      break;
    case SoundCue::kMainDeploy:
      played = playWavFromSd(kFallbackSoundFileMain);
      if (!played) {
        (void)playWavFromSd(kFallbackSoundFileMainLegacyDir);
      }
      break;
    case SoundCue::kLandingDetected:
      (void)playWavFromSd(kFallbackSoundFileLanding);
      break;
    case SoundCue::kHomePointSet:
      (void)playWavFromSd(kFallbackSoundFileHomePointSet);
      break;
    default:
      break;
  }
}

bool LvglController::playWavFromSd(const char* path, const char** failReason) {
#if !COMPANION_AUDIO_ENABLE
  (void)path;
  if (failReason != nullptr) {
    *failReason = "audio disabled";
  }
  return false;
#else
  if (!audioOutputReady_) {
    if (failReason != nullptr) {
      *failReason = "audio output not ready";
    }
    return false;
  }

  if (!sdStorageReady_) {
    if (failReason != nullptr) {
      *failReason = "sd storage not ready";
    }
    return false;
  }

  if (path == nullptr || path[0] == '\0') {
    if (failReason != nullptr) {
      *failReason = "invalid path";
    }
    return false;
  }

  File wav = SD.open(path, FILE_READ);
  if (!wav || wav.isDirectory()) {
    if (failReason != nullptr) {
      *failReason = "file open failed";
    }
    return false;
  }

  uint8_t riffHeader[12] = {0};
  if (wav.read(riffHeader, sizeof(riffHeader)) != static_cast<int>(sizeof(riffHeader)) ||
      memcmp(riffHeader, "RIFF", 4) != 0 || memcmp(riffHeader + 8, "WAVE", 4) != 0) {
    wav.close();
    if (failReason != nullptr) {
      *failReason = "invalid wav header";
    }
    return false;
  }

  uint16_t audioFormat = 0;
  uint16_t numChannels = 0;
  uint16_t bitsPerSample = 0;
  uint32_t sampleRate = 0;
  uint32_t dataOffset = 0;
  uint32_t dataBytes = 0;

  while (wav.available() >= 8) {
    uint8_t chunkHeader[8] = {0};
    if (wav.read(chunkHeader, sizeof(chunkHeader)) != static_cast<int>(sizeof(chunkHeader))) {
      break;
    }

    const uint32_t chunkSize = static_cast<uint32_t>(chunkHeader[4]) |
                               (static_cast<uint32_t>(chunkHeader[5]) << 8) |
                               (static_cast<uint32_t>(chunkHeader[6]) << 16) |
                               (static_cast<uint32_t>(chunkHeader[7]) << 24);
    const uint32_t chunkDataPos = static_cast<uint32_t>(wav.position());

    if (memcmp(chunkHeader, "fmt ", 4) == 0 && chunkSize >= 16) {
      uint8_t fmt[16] = {0};
      if (wav.read(fmt, sizeof(fmt)) != static_cast<int>(sizeof(fmt))) {
        break;
      }
      audioFormat = static_cast<uint16_t>(fmt[0]) | (static_cast<uint16_t>(fmt[1]) << 8);
      numChannels = static_cast<uint16_t>(fmt[2]) | (static_cast<uint16_t>(fmt[3]) << 8);
      sampleRate = static_cast<uint32_t>(fmt[4]) |
                   (static_cast<uint32_t>(fmt[5]) << 8) |
                   (static_cast<uint32_t>(fmt[6]) << 16) |
                   (static_cast<uint32_t>(fmt[7]) << 24);
      bitsPerSample = static_cast<uint16_t>(fmt[14]) | (static_cast<uint16_t>(fmt[15]) << 8);
    } else if (memcmp(chunkHeader, "data", 4) == 0) {
      dataOffset = chunkDataPos;
      dataBytes = chunkSize;
    }

    const uint32_t paddedSize = chunkSize + (chunkSize & 1U);
    const uint32_t nextPos = chunkDataPos + paddedSize;
    if (!wav.seek(nextPos)) {
      break;
    }
  }

  const bool formatOk = (audioFormat == 1U) && ((bitsPerSample == 8U) || (bitsPerSample == 16U)) &&
                        (numChannels >= 1U) && (sampleRate > 0U) && (dataBytes > 0U);
  if (!formatOk) {
    wav.close();
    if (failReason != nullptr) {
      *failReason = "unsupported wav format";
    }
    return false;
  }

  if (!wav.seek(dataOffset)) {
    wav.close();
    if (failReason != nullptr) {
      *failReason = "wav seek failed";
    }
    return false;
  }

  const uint32_t bytesPerSample = (bitsPerSample / 8U) * static_cast<uint32_t>(numChannels);
  if (bytesPerSample == 0U) {
    wav.close();
    if (failReason != nullptr) {
      *failReason = "invalid bytes per sample";
    }
    return false;
  }

  uint32_t samplePeriodUs = 1000000UL / sampleRate;
  if (samplePeriodUs == 0U) {
    samplePeriodUs = 1U;
  }

  uint32_t bytesRemaining = dataBytes;
  uint32_t nextSampleUs = micros();
  bool readOk = true;
  while (bytesRemaining >= bytesPerSample && readOk) {
    int32_t mixed = 0;
    for (uint16_t ch = 0; ch < numChannels; ++ch) {
      int32_t sample = 0;
      if (bitsPerSample == 8U) {
        const int b = wav.read();
        if (b < 0) {
          readOk = false;
          break;
        }
        sample = (static_cast<int32_t>(b) - 128) << 8;
      } else {
        const int lo = wav.read();
        const int hi = wav.read();
        if (lo < 0 || hi < 0) {
          readOk = false;
          break;
        }
        const uint16_t raw = static_cast<uint16_t>(static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo);
        sample = static_cast<int16_t>(raw);
      }
      mixed += sample;
    }
    if (!readOk) {
      break;
    }

    if (numChannels > 1U) {
      mixed /= static_cast<int32_t>(numChannels);
    }

    const uint8_t clampedVolume = (audioVolumePercent_ > 100U) ? 100U : audioVolumePercent_;
    mixed = (mixed * static_cast<int32_t>(clampedVolume)) / 100;

    int32_t sampleU8 = (mixed + 32768) >> 8;
    if (sampleU8 < 0) {
      sampleU8 = 0;
    } else if (sampleU8 > 255) {
      sampleU8 = 255;
    }

    const uint32_t duty = (static_cast<uint32_t>(sampleU8) * audioPwmMaxDuty_) / 255U;
    ledcWrite(COMPANION_AUDIO_PWM_CHANNEL, duty);

    bytesRemaining -= bytesPerSample;
    nextSampleUs += samplePeriodUs;
    int32_t waitUs = static_cast<int32_t>(nextSampleUs - micros());
    while (waitUs > 0) {
      const uint32_t chunkUs = (waitUs > 200) ? 200U : static_cast<uint32_t>(waitUs);
      delayMicroseconds(chunkUs);
      waitUs = static_cast<int32_t>(nextSampleUs - micros());
    }
  }

  ledcWrite(COMPANION_AUDIO_PWM_CHANNEL, audioPwmMaxDuty_ / 2U);
  wav.close();
  if (!readOk && failReason != nullptr) {
    *failReason = "wav read underrun";
  } else if (failReason != nullptr) {
    *failReason = nullptr;
  }
  return readOk;
#endif
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
  const uint32_t commAge = (lastRxMs_ == 0) ? UINT32_MAX : (now - lastRxMs_);
  state_.stale = (commAge > 3000);

  if (loraAgeBaseMs_ >= 0) {
    state_.link.lastPacketAgeMs =
        loraAgeBaseMs_ + static_cast<int32_t>(now - loraAgeBaseTickMs_);
  } else if (lastRxMs_ == 0) {
    state_.link.lastPacketAgeMs = -1;
  }

  if (state_.stale) {
    state_.link.connected = false;
  }
}

void LvglController::updateFlightTimerState(uint32_t now) {
  bool launchDetected = false;
  bool landingDetected = false;

  if (state_.hasRecoveryEventState) {
    launchDetected = state_.recoveryLaunchDetected;
    landingDetected = state_.recoveryLandingDetected;
  } else {
    String phase = state_.flight.phase;
    phase.toLowerCase();
    launchDetected = (phase == "boost" || phase == "coast" || phase == "ascent" ||
                      phase == "descent" || phase == "landed");
    landingDetected = (phase == "landed");
  }

  if (phaseChecklistCleared_) {
    flightTimerActive_ = false;
    flightTimerStartMs_ = 0;
    flightDurationMs_ = 0;
    flightTimerInitialized_ = true;
    lastFlightLaunchDetected_ = false;
    lastFlightLandingDetected_ = false;
    return;
  }

  if (phaseEquals(state_.flight.phase, "idle") && !launchDetected && !landingDetected) {
    flightTimerActive_ = false;
    flightTimerStartMs_ = 0;
    flightDurationMs_ = 0;
  }

  if (lastRxMs_ == 0) {
    return;
  }

  if (!flightTimerInitialized_) {
    lastFlightLaunchDetected_ = launchDetected;
    lastFlightLandingDetected_ = landingDetected;
    flightTimerInitialized_ = true;
    return;
  }

  if (launchDetected && !lastFlightLaunchDetected_) {
    flightTimerActive_ = true;
    flightTimerStartMs_ = now;
    flightDurationMs_ = 0;
  }

  if (landingDetected && !lastFlightLandingDetected_) {
    if (flightTimerActive_ && flightTimerStartMs_ > 0) {
      flightDurationMs_ = now - flightTimerStartMs_;
    }
    flightTimerActive_ = false;
  }

  if (flightTimerActive_ && flightTimerStartMs_ > 0) {
    flightDurationMs_ = now - flightTimerStartMs_;
  }

  lastFlightLaunchDetected_ = launchDetected;
  lastFlightLandingDetected_ = landingDetected;
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
  const uint32_t nowMs = millis();

  if (launchPrepBtn_ != nullptr && launchPrepLabel_ != nullptr) {
    const bool gpsFix3d = state_.recoveryGpsFix3d;
    const bool armGpsReady = gpsFix3d || allowArmWithoutGpsFix_;
    const bool launchArmed = state_.recoveryLaunchArmed;
    const bool prepBusy = sdCommandPending_ || txCommandPending_;
    const bool prepArmedWindow = launchPrepArmed_ && ((nowMs - launchPrepArmSinceMs_) <= kLaunchPrepArmWindowMs);

    if (launchArmed) {
      launchPrepArmed_ = false;
      lv_obj_add_state(launchPrepBtn_, LV_STATE_DISABLED);
      lv_label_set_text(launchPrepLabel_, "PREP: ARMED");
      lv_obj_set_style_bg_color(launchPrepBtn_, lv_color_hex(0x1f6a42), 0);
      lv_obj_set_style_bg_color(launchPrepBtn_, lv_color_hex(0x1f6a42), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(launchPrepBtn_, lv_color_hex(0x9de3bd), 0);
    } else if (commandLockoutActive_) {
      launchPrepArmed_ = false;
      lv_obj_add_state(launchPrepBtn_, LV_STATE_DISABLED);
      lv_label_set_text(launchPrepLabel_, "PREP LOCKED");
      lv_obj_set_style_bg_color(launchPrepBtn_, lv_color_hex(0x4b566d), 0);
      lv_obj_set_style_bg_color(launchPrepBtn_, lv_color_hex(0x4b566d), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(launchPrepBtn_, lv_color_hex(0xffb34f), 0);
    } else if (!armGpsReady) {
      launchPrepArmed_ = false;
      lv_obj_add_state(launchPrepBtn_, LV_STATE_DISABLED);
      lv_label_set_text(launchPrepLabel_, "PREP: WAIT GPS");
      lv_obj_set_style_bg_color(launchPrepBtn_, lv_color_hex(0x4b566d), 0);
      lv_obj_set_style_bg_color(launchPrepBtn_, lv_color_hex(0x4b566d), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(launchPrepBtn_, lv_color_hex(0xffb34f), 0);
    } else if (prepBusy) {
      lv_obj_add_state(launchPrepBtn_, LV_STATE_DISABLED);
      lv_label_set_text(launchPrepLabel_, "PREP: WAIT...");
      lv_obj_set_style_bg_color(launchPrepBtn_, lv_color_hex(0x4b566d), 0);
      lv_obj_set_style_bg_color(launchPrepBtn_, lv_color_hex(0x4b566d), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(launchPrepBtn_, lv_color_hex(0x6ea4ff), 0);
    } else {
      lv_obj_clear_state(launchPrepBtn_, LV_STATE_DISABLED);
      if (prepArmedWindow) {
        lv_label_set_text(launchPrepLabel_, "HOLD TO CONFIRM");
        lv_obj_set_style_bg_color(launchPrepBtn_, lv_color_hex(0x7a4f1d), 0);
        lv_obj_set_style_bg_color(launchPrepBtn_, lv_color_hex(0x9a6524), LV_STATE_PRESSED);
      } else {
        lv_label_set_text(launchPrepLabel_, "HOLD: TX+SD+ARM");
        lv_obj_set_style_bg_color(launchPrepBtn_, lv_color_hex(0x1f2a3b), 0);
        lv_obj_set_style_bg_color(launchPrepBtn_, lv_color_hex(0x2d4f86), LV_STATE_PRESSED);
      }
      lv_obj_set_style_border_color(launchPrepBtn_, gpsFix3d ? lv_color_hex(0x6ea4ff) : lv_color_hex(0xffb34f), 0);
    }
    lv_obj_set_style_border_width(launchPrepBtn_, 1, 0);
  }

  if (sdToggleBtn_ != nullptr && sdToggleLabel_ != nullptr) {
    const bool sdDisableLocked = commandLockoutActive_ && sdLoggingEnabled_;
    if (sdCommandPending_ || sdDisableLocked) {
      lv_obj_add_state(sdToggleBtn_, LV_STATE_DISABLED);
      if (sdCommandPending_) {
        lv_label_set_text_fmt(sdToggleLabel_, "SD %s...", sdPendingTargetEnabled_ ? "ON" : "OFF");
        lv_obj_set_style_bg_color(sdToggleBtn_, lv_color_hex(0x4b566d), 0);
        lv_obj_set_style_bg_color(sdToggleBtn_, lv_color_hex(0x4b566d), LV_STATE_PRESSED);
      } else {
        lv_label_set_text(sdToggleLabel_, "SD LOCK");
        lv_obj_set_style_bg_color(sdToggleBtn_, lv_color_hex(0x1f6a42), 0);
        lv_obj_set_style_bg_color(sdToggleBtn_, lv_color_hex(0x1f6a42), LV_STATE_PRESSED);
      }
    } else {
      lv_obj_clear_state(sdToggleBtn_, LV_STATE_DISABLED);
      lv_label_set_text_fmt(sdToggleLabel_, "SD %s", sdLoggingEnabled_ ? "ON" : "OFF");
      lv_obj_set_style_bg_color(sdToggleBtn_, sdLoggingEnabled_ ? lv_color_hex(0x1f6a42) : lv_color_hex(0x3a2b2b), 0);
      lv_obj_set_style_bg_color(sdToggleBtn_, sdLoggingEnabled_ ? lv_color_hex(0x2a8e5a) : lv_color_hex(0x5a3f3f),
                                LV_STATE_PRESSED);
    }
    lv_obj_set_style_border_color(sdToggleBtn_, sdDisableLocked ? lv_color_hex(0xffb34f) : lv_color_hex(0x6ea4ff), 0);
    lv_obj_set_style_border_width(sdToggleBtn_, 1, 0);
  }

  if (txToggleBtn_ != nullptr && txToggleLabel_ != nullptr) {
    const bool txDisableLocked = commandLockoutActive_ && telemetryTxEnabled_;
    if (txCommandPending_ || txDisableLocked) {
      lv_obj_add_state(txToggleBtn_, LV_STATE_DISABLED);
      if (txCommandPending_) {
        lv_label_set_text_fmt(txToggleLabel_, "TX %s...", txPendingTargetEnabled_ ? "ON" : "OFF");
        lv_obj_set_style_bg_color(txToggleBtn_, lv_color_hex(0x4b566d), 0);
        lv_obj_set_style_bg_color(txToggleBtn_, lv_color_hex(0x4b566d), LV_STATE_PRESSED);
      } else {
        lv_label_set_text(txToggleLabel_, "TX LOCK");
        lv_obj_set_style_bg_color(txToggleBtn_, lv_color_hex(0x1f5f73), 0);
        lv_obj_set_style_bg_color(txToggleBtn_, lv_color_hex(0x1f5f73), LV_STATE_PRESSED);
      }
    } else {
      lv_obj_clear_state(txToggleBtn_, LV_STATE_DISABLED);
      lv_label_set_text_fmt(txToggleLabel_, "TX %s", telemetryTxEnabled_ ? "ON" : "OFF");
      lv_obj_set_style_bg_color(txToggleBtn_, telemetryTxEnabled_ ? lv_color_hex(0x1f5f73) : lv_color_hex(0x3a2b2b), 0);
      lv_obj_set_style_bg_color(txToggleBtn_, telemetryTxEnabled_ ? lv_color_hex(0x2c839f) : lv_color_hex(0x5a3f3f),
                                LV_STATE_PRESSED);
    }
    lv_obj_set_style_border_color(txToggleBtn_, txDisableLocked ? lv_color_hex(0xffb34f) : lv_color_hex(0x6ea4ff), 0);
    lv_obj_set_style_border_width(txToggleBtn_, 1, 0);
  }

  if (armBtn_ != nullptr && armLabel_ != nullptr) {
    const bool gpsFix3d = state_.recoveryGpsFix3d;
    const bool armGpsReady = gpsFix3d || allowArmWithoutGpsFix_;
    const bool launchArmed = state_.recoveryLaunchArmed;
    const bool armLocked = commandLockoutActive_ || launchArmed || !armGpsReady;
    if (armLocked) {
      lv_obj_add_state(armBtn_, LV_STATE_DISABLED);
      if (launchArmed) {
        lv_label_set_text(armLabel_, "ARMED");
        lv_obj_set_style_bg_color(armBtn_, lv_color_hex(0x1f6a42), 0);
        lv_obj_set_style_bg_color(armBtn_, lv_color_hex(0x1f6a42), LV_STATE_PRESSED);
      } else if (commandLockoutActive_) {
        lv_label_set_text(armLabel_, "ARM LOCKED");
        lv_obj_set_style_bg_color(armBtn_, lv_color_hex(0x4b566d), 0);
        lv_obj_set_style_bg_color(armBtn_, lv_color_hex(0x4b566d), LV_STATE_PRESSED);
      } else {
        lv_label_set_text(armLabel_, "ARM: WAIT GPS");
        lv_obj_set_style_bg_color(armBtn_, lv_color_hex(0x4b566d), 0);
        lv_obj_set_style_bg_color(armBtn_, lv_color_hex(0x4b566d), LV_STATE_PRESSED);
      }
    } else {
      lv_obj_clear_state(armBtn_, LV_STATE_DISABLED);
      lv_label_set_text(armLabel_, gpsFix3d ? "ARM LAUNCH" : "ARM: NO GPS OK");
      lv_obj_set_style_bg_color(armBtn_, lv_color_hex(0x1f2a3b), 0);
      lv_obj_set_style_bg_color(armBtn_, lv_color_hex(0x2d4f86), LV_STATE_PRESSED);
    }
    const lv_color_t armBorderColor = gpsFix3d ? lv_color_hex(0x6ea4ff) : lv_color_hex(0xffb34f);
    lv_obj_set_style_border_color(armBtn_, armBorderColor, 0);
    lv_obj_set_style_border_width(armBtn_, 1, 0);
  }

  if (rebootBtn_ != nullptr) {
    if (commandLockoutActive_) {
      lv_obj_add_state(rebootBtn_, LV_STATE_DISABLED);
      lv_obj_set_style_bg_color(rebootBtn_, lv_color_hex(0x4b566d), 0);
      lv_obj_set_style_bg_color(rebootBtn_, lv_color_hex(0x4b566d), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(rebootBtn_, lv_color_hex(0xffb34f), 0);
    } else {
      lv_obj_clear_state(rebootBtn_, LV_STATE_DISABLED);
      lv_obj_set_style_bg_color(rebootBtn_, lv_color_hex(0x6a4a1f), 0);
      lv_obj_set_style_bg_color(rebootBtn_, lv_color_hex(0x8b6127), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(rebootBtn_, lv_color_hex(0xe3b36d), 0);
    }
    lv_obj_set_style_border_width(rebootBtn_, 1, 0);
  }

  if (shutdownBtn_ != nullptr) {
    if (commandLockoutActive_) {
      lv_obj_add_state(shutdownBtn_, LV_STATE_DISABLED);
      lv_obj_set_style_bg_color(shutdownBtn_, lv_color_hex(0x4b566d), 0);
      lv_obj_set_style_bg_color(shutdownBtn_, lv_color_hex(0x4b566d), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(shutdownBtn_, lv_color_hex(0xffb34f), 0);
    } else {
      lv_obj_clear_state(shutdownBtn_, LV_STATE_DISABLED);
      lv_obj_set_style_bg_color(shutdownBtn_, lv_color_hex(0x7a1d1d), 0);
      lv_obj_set_style_bg_color(shutdownBtn_, lv_color_hex(0xa02828), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(shutdownBtn_, lv_color_hex(0xdc7070), 0);
    }
    lv_obj_set_style_border_width(shutdownBtn_, 1, 0);
  }

  if (wifiApToggleBtn_ != nullptr && wifiApToggleLabel_ != nullptr) {
    const bool apKnown = state_.hasWifiApState;
    const bool apActive = apKnown && state_.wifiApActive;

    lv_obj_clear_state(wifiApToggleBtn_, LV_STATE_DISABLED);
    if (apActive) {
      lv_label_set_text(wifiApToggleLabel_, "HOLD: DISABLE AP");
      lv_obj_set_style_bg_color(wifiApToggleBtn_, lv_color_hex(0x6a2a1f), 0);
      lv_obj_set_style_bg_color(wifiApToggleBtn_, lv_color_hex(0x8e3a2a), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(wifiApToggleBtn_, lv_color_hex(0xffb199), 0);
    } else if (apKnown) {
      lv_label_set_text(wifiApToggleLabel_, "HOLD: ENABLE AP");
      lv_obj_set_style_bg_color(wifiApToggleBtn_, lv_color_hex(0x1f5a45), 0);
      lv_obj_set_style_bg_color(wifiApToggleBtn_, lv_color_hex(0x26755b), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(wifiApToggleBtn_, lv_color_hex(0xa3f2cf), 0);
    } else {
      lv_label_set_text(wifiApToggleLabel_, "HOLD: TOGGLE AP");
      lv_obj_set_style_bg_color(wifiApToggleBtn_, lv_color_hex(0x244374), 0);
      lv_obj_set_style_bg_color(wifiApToggleBtn_, lv_color_hex(0x2e5ca0), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(wifiApToggleBtn_, lv_color_hex(0x6ea4ff), 0);
    }
    lv_obj_set_style_border_width(wifiApToggleBtn_, 1, 0);
  }

  if (resetFlightBtn_ != nullptr && resetFlightLabel_ != nullptr) {
    lv_obj_clear_state(resetFlightBtn_, LV_STATE_DISABLED);
    lv_obj_set_style_bg_color(resetFlightBtn_, lv_color_hex(0x7a1d1d), 0);
    lv_obj_set_style_bg_color(resetFlightBtn_, lv_color_hex(0xa02828), LV_STATE_PRESSED);
    lv_obj_set_style_border_color(resetFlightBtn_, lv_color_hex(0xdc7070), 0);
    lv_obj_set_style_border_width(resetFlightBtn_, 1, 0);

    if (phaseResetRequested_) {
      lv_label_set_text(resetFlightLabel_, "RESETTING...");
    } else if (resetFlightArmed_) {
      lv_label_set_text(resetFlightLabel_, "HOLD TO CONFIRM");
    } else {
      lv_label_set_text(resetFlightLabel_, "HOLD: RESET FLIGHT");
    }
  }

  const bool sdAnyPending = sdCommandPending_ || sdUtilityCommandPending_;
  if (sdRotateBtn_ != nullptr && sdRotateLabel_ != nullptr) {
    const bool rotateEnabled = sdLoggingEnabled_ && !sdAnyPending;
    if (rotateEnabled) {
      lv_obj_clear_state(sdRotateBtn_, LV_STATE_DISABLED);
      lv_obj_set_style_bg_color(sdRotateBtn_, lv_color_hex(0x1f2a3b), 0);
      lv_obj_set_style_bg_color(sdRotateBtn_, lv_color_hex(0x2d4f86), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(sdRotateBtn_, lv_color_hex(0x4b7dd1), 0);
    } else {
      lv_obj_add_state(sdRotateBtn_, LV_STATE_DISABLED);
      lv_obj_set_style_bg_color(sdRotateBtn_, lv_color_hex(0x4b566d), 0);
      lv_obj_set_style_bg_color(sdRotateBtn_, lv_color_hex(0x4b566d), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(sdRotateBtn_, lv_color_hex(0x6b7792), 0);
    }
    lv_obj_set_style_border_width(sdRotateBtn_, 1, 0);
    if (sdUtilityCommandPending_ && sdUtilityPendingAction_ == "sd_rotate") {
      lv_label_set_text(sdRotateLabel_, "ROTATING...");
    } else {
      lv_label_set_text(sdRotateLabel_, "ROTATE LOGFILE");
    }
  }

  if (sdFormatBtn_ != nullptr && sdFormatLabel_ != nullptr) {
    const bool formatEnabled = !sdLoggingEnabled_ && !sdAnyPending && !commandLockoutActive_;
    if (formatEnabled) {
      lv_obj_clear_state(sdFormatBtn_, LV_STATE_DISABLED);
      lv_obj_set_style_bg_color(sdFormatBtn_, lv_color_hex(0x7a1d1d), 0);
      lv_obj_set_style_bg_color(sdFormatBtn_, lv_color_hex(0xa02828), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(sdFormatBtn_, lv_color_hex(0xdc7070), 0);
    } else {
      lv_obj_add_state(sdFormatBtn_, LV_STATE_DISABLED);
      lv_obj_set_style_bg_color(sdFormatBtn_, lv_color_hex(0x4b566d), 0);
      lv_obj_set_style_bg_color(sdFormatBtn_, lv_color_hex(0x4b566d), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(sdFormatBtn_, commandLockoutActive_ ? lv_color_hex(0xffb34f) : lv_color_hex(0x6b7792),
                                    0);
    }
    lv_obj_set_style_border_width(sdFormatBtn_, 1, 0);

    if (sdUtilityCommandPending_ && sdUtilityPendingAction_ == "sd_format") {
      lv_label_set_text(sdFormatLabel_, "FORMATTING...");
    } else if (sdFormatArmed_ && (nowMs - sdFormatArmSinceMs_) <= kSdFormatArmWindowMs && !sdAnyPending &&
               !sdLoggingEnabled_ && !commandLockoutActive_) {
      lv_label_set_text(sdFormatLabel_, "HOLD TO CONFIRM");
    } else {
      lv_label_set_text(sdFormatLabel_, "HOLD: FORMAT SD");
    }
  }

  if (sdDumpBtn_ != nullptr && sdDumpLabel_ != nullptr) {
    const bool dumpEnabled = !sdLoggingEnabled_ && !sdAnyPending;
    if (dumpEnabled) {
      lv_obj_clear_state(sdDumpBtn_, LV_STATE_DISABLED);
      lv_obj_set_style_bg_color(sdDumpBtn_, lv_color_hex(0x1f2a3b), 0);
      lv_obj_set_style_bg_color(sdDumpBtn_, lv_color_hex(0x2d4f86), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(sdDumpBtn_, lv_color_hex(0x4b7dd1), 0);
    } else {
      lv_obj_add_state(sdDumpBtn_, LV_STATE_DISABLED);
      lv_obj_set_style_bg_color(sdDumpBtn_, lv_color_hex(0x4b566d), 0);
      lv_obj_set_style_bg_color(sdDumpBtn_, lv_color_hex(0x4b566d), LV_STATE_PRESSED);
      lv_obj_set_style_border_color(sdDumpBtn_, lv_color_hex(0x6b7792), 0);
    }
    lv_obj_set_style_border_width(sdDumpBtn_, 1, 0);
    if (sdUtilityCommandPending_ && sdUtilityPendingAction_ == "sd_dump_sample") {
      lv_label_set_text(sdDumpLabel_, "DUMPING...");
    } else {
      lv_label_set_text(sdDumpLabel_, "DUMP SAMPLE");
    }
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

  if (state_.hasSdCardAckState) {
    const uint32_t ackToken = state_.sdCardAckToken;
    if (ackToken != 0 && ackToken != lastSdCardAckToken_) {
      lastSdCardAckToken_ = ackToken;

      const String ackAction = state_.sdCardLastCommand;
      const bool ackOk = state_.sdCardAckOk;
      String ackDetail = state_.sdCardDetail;
      ackDetail.trim();

      if (ackAction == "sd_rotate" || ackAction == "sd_format" || ackAction == "sd_dump_sample") {
        if (sdUtilityCommandPending_ && ackAction == sdUtilityPendingAction_) {
          sdUtilityCommandPending_ = false;
          sdUtilityPendingAction_ = "";
          sdUtilityPendingSinceMs_ = 0;
          changed = true;
        }

        String statusPrefix;
        if (ackAction == "sd_rotate") {
          statusPrefix = "SD rotate";
        } else if (ackAction == "sd_format") {
          statusPrefix = "SD format";
          if (ackOk) {
            sdFormatArmed_ = false;
          }
        } else {
          statusPrefix = "SD dump sample";
        }

        String statusMsg = statusPrefix;
        statusMsg += ackOk ? " OK" : " failed";
        if (ackDetail.length() > 0) {
          statusMsg += ": ";
          statusMsg += ackDetail;
        }
        setCommandStatus(statusMsg, ackOk);

        if (ackAction == "sd_dump_sample") {
          if (ackDetail.length() == 0) {
            ackDetail = ackOk ? "No sample text returned." : "Dump sample failed.";
          }
          setSdDumpOverlayText(ackDetail);
          setSdDumpOverlayVisible(true);
        }
      }
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

  bool reportedLockout = phaseIndicatesInFlight(state_.flight.phase);
  if (state_.hasCommandLockoutState) {
    reportedLockout = state_.commandLockoutActive;
  }
  if (commandLockoutActive_ != reportedLockout) {
    commandLockoutActive_ = reportedLockout;
    changed = true;
  }

  if (state_.hasTelemetryTxPowerState) {
    const uint8_t reportedPowerDbm = state_.telemetryTxPowerDbm;
    if (!hasTxPowerReadback_ || txPowerActiveDbm_ != reportedPowerDbm) {
      hasTxPowerReadback_ = true;
      txPowerActiveDbm_ = reportedPowerDbm;
      changed = true;
    }

    if (!txPowerDefaultSynced_) {
      uint8_t desiredPowerDbm = reportedPowerDbm;
      if (desiredPowerDbm < kTelemetryTxPowerMinDbm) {
        desiredPowerDbm = kTelemetryTxPowerMinDbm;
      } else if (desiredPowerDbm > kTelemetryTxPowerMaxDbm) {
        desiredPowerDbm = kTelemetryTxPowerMaxDbm;
      }

      txPowerDbm_ = desiredPowerDbm;
      if (txPowerSlider_ != nullptr) {
        lv_slider_set_value(txPowerSlider_, txPowerDbm_, LV_ANIM_OFF);
      }
      if (txPowerLabel_ != nullptr) {
        lv_label_set_text_fmt(txPowerLabel_, "%udBm", static_cast<unsigned>(txPowerDbm_));
      }
      txPowerDefaultSynced_ = true;
      changed = true;
    }
  } else if (hasTxPowerReadback_) {
    hasTxPowerReadback_ = false;
    txPowerActiveDbm_ = 0;
    changed = true;
  }

  if (txPowerActiveLabel_ != nullptr) {
    if (hasTxPowerReadback_) {
      lv_label_set_text_fmt(txPowerActiveLabel_, "Active: %udBm", static_cast<unsigned>(txPowerActiveDbm_));
    } else {
      lv_label_set_text(txPowerActiveLabel_, "Active: -- dBm");
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

  if (sdUtilityCommandPending_ && (now - sdUtilityPendingSinceMs_) > kCommandConfirmTimeoutMs) {
    sdUtilityCommandPending_ = false;
    sdUtilityPendingAction_ = "";
    sdUtilityPendingSinceMs_ = 0;
    setCommandStatus("SD utility command failed (timeout)", false);
    changed = true;
  }

  if (sdFormatArmed_ && (now - sdFormatArmSinceMs_) > kSdFormatArmWindowMs) {
    sdFormatArmed_ = false;
    changed = true;
  }

  if (launchPrepArmed_ && (now - launchPrepArmSinceMs_) > kLaunchPrepArmWindowMs) {
    launchPrepArmed_ = false;
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

  if (!enable && commandLockoutActive_) {
    setCommandStatus("SD stop locked until landing", false);
    refreshUi();
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

  if (!enable && commandLockoutActive_) {
    setCommandStatus("TX disable locked until landing", false);
    refreshUi();
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

bool LvglController::requestArmLaunch() {
  if (!state_.recoveryGpsFix3d && !allowArmWithoutGpsFix_) {
    setCommandStatus("ARM blocked: waiting for GPS 3D fix", false);
    return false;
  }
  if (state_.recoveryLaunchArmed) {
    setCommandStatus("Launch detect already armed", true);
    return false;
  }
  if (commandLockoutActive_) {
    setCommandStatus("Arm command locked in flight", false);
    return false;
  }
  if (!sendAction("launch_arm", allowArmWithoutGpsFix_ ? 1 : 0)) {
    return false;
  }

  launchPrepArmed_ = false;
  panelCollapsed_ = true;
  settingsCollapsed_ = true;
  setSoundSettingsVisible(false);
  setSdFunctionsVisible(false);
  if (actionPanel_ != nullptr) {
    lv_obj_add_flag(actionPanel_, LV_OBJ_FLAG_HIDDEN);
  }
  if (settingsBody_ != nullptr) {
    lv_obj_add_flag(settingsBody_, LV_OBJ_FLAG_HIDDEN);
  }
  if (telemetryPanel_ != nullptr) {
    lv_obj_update_layout(telemetryPanel_);
  }
  return true;
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
  } else if (action == "telemetry_tx_power") {
    pretty = "TELEM TX POWER " + String(durationS) + "dBm";
  } else if (action == "phase_reset") {
    pretty = "RESET FLIGHT";
  } else if (action == "launch_arm" && durationS != 0) {
    pretty = "LAUNCH ARM (NO GPS)";
  } else if (action == "sd_rotate") {
    pretty = "SD ROTATE";
  } else if (action == "sd_format") {
    pretty = "SD FORMAT";
  } else if (action == "sd_dump_sample") {
    pretty = "SD DUMP SAMPLE";
  } else if (action == "wifi_ap_toggle") {
    pretty = "WIFI AP TOGGLE";
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
    setSoundSettingsVisible(false);
    setSdFunctionsVisible(false);
    lv_obj_add_flag(settingsBody_, LV_OBJ_FLAG_HIDDEN);
  }
  lv_obj_update_layout(telemetryPanel_);
}

void LvglController::setAllowArmWithoutGpsFix(bool enabled) {
  if (allowArmWithoutGpsFix_ == enabled) {
    if (armNoGpsCheckbox_ != nullptr) {
      if (allowArmWithoutGpsFix_) {
        lv_obj_add_state(armNoGpsCheckbox_, LV_STATE_CHECKED);
      } else {
        lv_obj_clear_state(armNoGpsCheckbox_, LV_STATE_CHECKED);
      }
    }
    return;
  }

  allowArmWithoutGpsFix_ = enabled;
  prefs_.putBool("arm_no_gps", allowArmWithoutGpsFix_);

  if (allowArmWithoutGpsFix_ && soundQueueCount_ > 0) {
    SoundCue filteredQueue[kSoundQueueCapacity] = {};
    uint8_t filteredCount = 0;
    for (uint8_t i = 0; i < soundQueueCount_; ++i) {
      const uint8_t idx = static_cast<uint8_t>((soundQueueHead_ + i) % kSoundQueueCapacity);
      const SoundCue queued = soundQueue_[idx];
      if (queued == SoundCue::kWaitingForLocationFix) {
        continue;
      }
      filteredQueue[filteredCount++] = queued;
    }
    for (uint8_t i = 0; i < filteredCount; ++i) {
      soundQueue_[i] = filteredQueue[i];
    }
    soundQueueHead_ = 0;
    soundQueueTail_ = static_cast<uint8_t>(filteredCount % kSoundQueueCapacity);
    soundQueueCount_ = filteredCount;
  }

  if (armNoGpsCheckbox_ != nullptr) {
    if (allowArmWithoutGpsFix_) {
      lv_obj_add_state(armNoGpsCheckbox_, LV_STATE_CHECKED);
    } else {
      lv_obj_clear_state(armNoGpsCheckbox_, LV_STATE_CHECKED);
    }
  }

  setCommandStatus(allowArmWithoutGpsFix_ ? "Launch arm override enabled"
                                         : "Launch arm now requires GPS 3D fix",
                   true);
  updateDashboardActionButtons();
}

void LvglController::setTouchDebugVisible(bool visible) {
  touchDebugVisible_ = visible;
  if (touchDebugLabel_ == nullptr) {
    return;
  }

  if (touchDebugVisible_) {
    lv_obj_clear_flag(touchDebugLabel_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(touchDebugLabel_, LV_OBJ_FLAG_HIDDEN);
  }
}

void LvglController::toggleSettings() {
  settingsCollapsed_ = !settingsCollapsed_;
  if (settingsCollapsed_) {
    setSoundSettingsVisible(false);
    setSdFunctionsVisible(false);
    lv_obj_add_flag(settingsBody_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_clear_flag(settingsBody_, LV_OBJ_FLAG_HIDDEN);
    if (hasTxPowerReadback_) {
      uint8_t desiredPowerDbm = txPowerActiveDbm_;
      if (desiredPowerDbm < kTelemetryTxPowerMinDbm) {
        desiredPowerDbm = kTelemetryTxPowerMinDbm;
      } else if (desiredPowerDbm > kTelemetryTxPowerMaxDbm) {
        desiredPowerDbm = kTelemetryTxPowerMaxDbm;
      }
      txPowerDbm_ = desiredPowerDbm;
      if (txPowerSlider_ != nullptr) {
        lv_slider_set_value(txPowerSlider_, txPowerDbm_, LV_ANIM_OFF);
      }
      if (txPowerLabel_ != nullptr) {
        lv_label_set_text_fmt(txPowerLabel_, "%udBm", static_cast<unsigned>(txPowerDbm_));
      }
    }
    setSoundSettingsVisible(false);
    setSdFunctionsVisible(false);
    if (settingsActions_ != nullptr) {
      lv_obj_scroll_to_y(settingsActions_, 0, LV_ANIM_OFF);
    }
    panelCollapsed_ = true;
    lv_obj_add_flag(actionPanel_, LV_OBJ_FLAG_HIDDEN);
  }
  lv_obj_update_layout(telemetryPanel_);
}

void LvglController::setSoundEnabled(bool enabled) {
  if (soundEnabled_ == enabled) {
    if (soundEnabledCheckbox_ != nullptr) {
      if (soundEnabled_) {
        lv_obj_add_state(soundEnabledCheckbox_, LV_STATE_CHECKED);
      } else {
        lv_obj_clear_state(soundEnabledCheckbox_, LV_STATE_CHECKED);
      }
    }
    return;
  }

  soundEnabled_ = enabled;
  prefs_.putBool("sound_enabled", soundEnabled_);

  if (!soundEnabled_) {
    soundQueueHead_ = 0;
    soundQueueTail_ = 0;
    soundQueueCount_ = 0;
  }

  if (soundEnabledCheckbox_ != nullptr) {
    if (soundEnabled_) {
      lv_obj_add_state(soundEnabledCheckbox_, LV_STATE_CHECKED);
    } else {
      lv_obj_clear_state(soundEnabledCheckbox_, LV_STATE_CHECKED);
    }
  }

  setCommandStatus(soundEnabled_ ? "Sound cues enabled" : "Sound cues disabled", true);
}

void LvglController::setSoundSettingsVisible(bool visible) {
  soundSettingsVisible_ = visible;
  if (settingsActions_ == nullptr || soundSettingsPanel_ == nullptr || sdFunctionsPanel_ == nullptr) {
    return;
  }

  if (soundSettingsVisible_) {
    sdFunctionsVisible_ = false;
    lv_obj_add_flag(settingsActions_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(soundSettingsPanel_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(sdFunctionsPanel_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_scroll_to_y(soundSettingsPanel_, 0, LV_ANIM_OFF);
  } else {
    if (!sdFunctionsVisible_) {
      lv_obj_clear_flag(settingsActions_, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_add_flag(soundSettingsPanel_, LV_OBJ_FLAG_HIDDEN);
    if (!sdFunctionsVisible_) {
      lv_obj_scroll_to_y(settingsActions_, 0, LV_ANIM_OFF);
    }
  }

  if (settingsBody_ != nullptr) {
    lv_obj_update_layout(settingsBody_);
  }
}

void LvglController::setSdFunctionsVisible(bool visible) {
  sdFunctionsVisible_ = visible;
  if (settingsActions_ == nullptr || soundSettingsPanel_ == nullptr || sdFunctionsPanel_ == nullptr) {
    return;
  }

  if (sdFunctionsVisible_) {
    soundSettingsVisible_ = false;
    lv_obj_add_flag(settingsActions_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(soundSettingsPanel_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(sdFunctionsPanel_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_scroll_to_y(sdFunctionsPanel_, 0, LV_ANIM_OFF);
  } else {
    if (!soundSettingsVisible_) {
      lv_obj_clear_flag(settingsActions_, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_add_flag(sdFunctionsPanel_, LV_OBJ_FLAG_HIDDEN);
    if (!soundSettingsVisible_) {
      lv_obj_scroll_to_y(settingsActions_, 0, LV_ANIM_OFF);
    }
  }

  if (settingsBody_ != nullptr) {
    lv_obj_update_layout(settingsBody_);
  }
}

void LvglController::setSdDumpOverlayVisible(bool visible) {
  if (sdDumpOverlay_ == nullptr) {
    return;
  }
  if (visible) {
    lv_obj_clear_flag(sdDumpOverlay_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(sdDumpOverlay_, LV_OBJ_FLAG_HIDDEN);
  }
}

void LvglController::setSdDumpOverlayText(const String& text) {
  if (sdDumpTextLabel_ == nullptr) {
    return;
  }
  lv_label_set_text(sdDumpTextLabel_, text.c_str());
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

  if (calibrationCancelBtn_ != nullptr) {
    int screenX = 0;
    int screenY = 0;
    if (mapTouchToScreen(rawX, rawY, screenX, screenY)) {
      const int btnX = lv_obj_get_x(calibrationCancelBtn_);
      const int btnY = lv_obj_get_y(calibrationCancelBtn_);
      const int btnW = lv_obj_get_width(calibrationCancelBtn_);
      const int btnH = lv_obj_get_height(calibrationCancelBtn_);
      if (screenX >= btnX && screenX < (btnX + btnW) &&
          screenY >= btnY && screenY < (btnY + btnH)) {
        cancelCalibration();
        setCommandStatus("Calibration cancelled", true);
        refreshUi();
        return;
      }
    }
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
  updateDashboardActionButtons();
  bool groundStationAppOffline = false;
#if COMPANION_LINK_UART
  groundStationAppOffline = (lastRxMs_ == 0) && (now > kGroundStationOfflineDetectMs);
#else
  groundStationAppOffline = !sseConnected_;
#endif

  const bool linkLive = state_.link.connected && !state_.stale;

  lv_label_set_text(linkLabel_, groundStationAppOffline
                                    ? "GS APP OFFLINE"
                                    : (linkLive ? "LINK LIVE" : (state_.stale ? "LINK STALE" : "NO LINK")));
  lv_obj_set_style_text_color(
      linkLabel_,
      groundStationAppOffline
          ? lv_color_hex(0xff6b6b)
          : (linkLive ? lv_color_hex(0x6cff95)
                      : (state_.stale ? lv_color_hex(0xffc369) : lv_color_hex(0xff6b6b))),
      0);

  if (wifiIndicator_ != nullptr && wifiIndicatorIcon_ != nullptr && wifiIndicatorState_ != nullptr) {
    const bool wifiApActive = state_.hasWifiApState && state_.wifiApActive;
#if (!COMPANION_LINK_UART) || OTA_ENABLE
    const bool wifiConnected = (WiFi.status() == WL_CONNECTED);
    lv_label_set_text(wifiIndicatorIcon_, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(wifiIndicatorIcon_, lv_color_hex(wifiConnected ? 0xe6f2ff : 0xa7b4c9), 0);

    if (wifiApActive) {
      lv_label_set_text(wifiIndicatorState_, "AP");
      lv_obj_set_style_text_color(wifiIndicatorState_, lv_color_hex(0x102436), 0);
      lv_obj_set_style_bg_color(wifiIndicatorState_, lv_color_hex(0x7ec8ff), 0);
      lv_obj_set_style_border_color(wifiIndicatorState_, lv_color_hex(0xb5e4ff), 0);
    } else {
      lv_label_set_text(wifiIndicatorState_, wifiConnected ? LV_SYMBOL_OK : LV_SYMBOL_CLOSE);
      lv_obj_set_style_text_color(wifiIndicatorState_,
                                  lv_color_hex(wifiConnected ? 0x10351f : 0x3a1111),
                                  0);
      lv_obj_set_style_bg_color(wifiIndicatorState_,
                                lv_color_hex(wifiConnected ? 0x6cff95 : 0xff8e8e),
                                0);
      lv_obj_set_style_border_color(wifiIndicatorState_,
                                    lv_color_hex(wifiConnected ? 0xb4ffd0 : 0xffc3c3),
                                    0);
    }
#else
    lv_label_set_text(wifiIndicatorIcon_, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(wifiIndicatorIcon_, lv_color_hex(0x637287), 0);

    if (wifiApActive) {
      lv_label_set_text(wifiIndicatorState_, "AP");
      lv_obj_set_style_text_color(wifiIndicatorState_, lv_color_hex(0x102436), 0);
      lv_obj_set_style_bg_color(wifiIndicatorState_, lv_color_hex(0x7ec8ff), 0);
      lv_obj_set_style_border_color(wifiIndicatorState_, lv_color_hex(0xb5e4ff), 0);
    } else {
      lv_label_set_text(wifiIndicatorState_, "-");
      lv_obj_set_style_text_color(wifiIndicatorState_, lv_color_hex(0x93a3bc), 0);
      lv_obj_set_style_bg_color(wifiIndicatorState_, lv_color_hex(0x2d394d), 0);
      lv_obj_set_style_border_color(wifiIndicatorState_, lv_color_hex(0x4d5f79), 0);
    }
#endif
  }

  const String snrText = formatFloat(state_.link.snr, 1, "--.-");
  lv_label_set_text_fmt(linkMetaLabel_,
                        "RSSI %d   SNR %s   AGE %d   Packets: %lu",
                        state_.link.rssi,
                        snrText.c_str(),
                        state_.link.lastPacketAgeMs,
                        static_cast<unsigned long>(state_.flight.packetCount));

  lv_label_set_text_fmt(phaseLabel_, "PHASE: %s", state_.flight.phase.length() ? state_.flight.phase.c_str() : "unknown");

  const int8_t checklistIndex = phaseChecklistCleared_ ? -1 : phaseChecklistIndex(state_.flight.phase);
  lv_label_set_text_fmt(phaseChecklistLabel_,
                        "%s Idle\n\n%s Boost\n\n%s Coast\n\n%s Descent\n\n%s Landed",
                        checklistMarkerForStage(checklistIndex, 0),
                        checklistMarkerForStage(checklistIndex, 1),
                        checklistMarkerForStage(checklistIndex, 2),
                        checklistMarkerForStage(checklistIndex, 3),
                        checklistMarkerForStage(checklistIndex, 4));

  const String baroAltText = formatFloat(state_.alt.altitudeAglM, 1);
  const String gpsAltText = formatFloat(state_.alt.gpsAltitudeM, 1);
  float preferredVerticalSpeedMps = state_.alt.verticalSpeedMps;
  if (isnan(preferredVerticalSpeedMps)) {
    preferredVerticalSpeedMps = state_.alt.baroVerticalSpeedMps;
  }
  if (isnan(preferredVerticalSpeedMps)) {
    preferredVerticalSpeedMps = state_.alt.gpsVerticalSpeedMps;
  }
  const String vsText = formatFloat(preferredVerticalSpeedMps, 1);
  const String svsText = state_.hasGpsSvsState
                             ? (String(state_.gpsSvsUsed) + "/" + String(state_.gpsSvsTotal))
                             : String("--/--");
  const String hdopText = state_.hasGpsHdopState ? formatFloat(state_.gpsHdop, 2, "--") : String("--");
  const char* fixText = state_.recoveryGpsFix3d ? "3D" : "NO";

  lv_label_set_text_fmt(altitudeLabel_,
                        "BARO %s m\nGPS  %s m\nVS   %s m/s\nSAT  %s\nHDOP %s\nFIX  %s",
                        baroAltText.c_str(),
                        gpsAltText.c_str(),
                        vsText.c_str(),
                        svsText.c_str(),
                        hdopText.c_str(),
                        fixText);

  lv_label_set_text(callsignLabel_,
                    state_.flight.callsign.length() ? state_.flight.callsign.c_str() : "No Callsign");

  const String txVbat = formatFloat(state_.battery.telemetryVbatV, 2, "--.--");
  const String groundVbat = formatFloat(state_.battery.groundVbatV, 2, "--.--");
  const String clockText = formatClockText(now);
  const String flightText = (flightDurationMs_ > 0) ? formatDurationText(flightDurationMs_) : String("--:--:--");
  lv_label_set_text_fmt(batteryLabel_,
                        "TX: %sV / GS: %sV\nDT %s  FLT %s",
                        txVbat.c_str(),
                        groundVbat.c_str(),
                        clockText.c_str(),
                        flightText.c_str());

  String cmdStatus = "";
  if (cmdMsg_.length() > 0 && (now - cmdTs_) <= kCommandStatusShowMs) {
    cmdStatus = cmdMsg_;
    lv_obj_set_style_text_color(cmdStatusLabel_, cmdOk_ ? lv_color_hex(0x87f0ae) : lv_color_hex(0xff8e8e), 0);
  }
  lv_label_set_text(cmdStatusLabel_, cmdStatus.c_str());

  String alert;
  if (groundStationAppOffline) {
    alert = "Start ground_station_server.py on Pi";
  } else if (state_.primaryAlert.length() > 0) {
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
  if (touchDebugVisible_) {
    lv_obj_clear_flag(touchDebugLabel_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(touchDebugLabel_, LV_OBJ_FLAG_HIDDEN);
  }

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
#if COMPANION_LINK_UART
    const bool loraPacketAdvanced =
        !hasLastLoraPacketCount_ || (state_.flight.packetCount != lastLoraPacketCount_);
    if (loraPacketAdvanced) {
      hasLastLoraPacketCount_ = true;
      lastLoraPacketCount_ = state_.flight.packetCount;
      loraAgeBaseMs_ = (state_.link.lastPacketAgeMs >= 0) ? state_.link.lastPacketAgeMs : 0;
      loraAgeBaseTickMs_ = now;
    }
#else
    if (state_.link.lastPacketAgeMs >= 0) {
      loraAgeBaseMs_ = state_.link.lastPacketAgeMs;
      loraAgeBaseTickMs_ = now;
    }
#endif
    const String previousPhase = lastPhase_;
    const String reportedPhase = state_.flight.phase;
    String effectivePhase = reportedPhase;

    const int8_t previousPhaseStep = phaseProgressStep(previousPhase);
    const int8_t reportedPhaseStep = phaseProgressStep(reportedPhase);

    bool allowPhaseResetToIdle = false;
    if (phaseResetRequested_) {
      if (reportedPhaseStep == 0) {
        allowPhaseResetToIdle = true;
      } else if (reportedPhaseStep > 0) {
        phaseResetRequested_ = false;
      }
    }

    if (previousPhaseStep >= 0) {
      if (reportedPhaseStep < 0) {
        effectivePhase = previousPhase;
      } else if (reportedPhaseStep < previousPhaseStep) {
        if (!allowPhaseResetToIdle) {
          effectivePhase = previousPhase;
        }
      } else if (reportedPhaseStep > (previousPhaseStep + 1)) {
        if (previousPhaseStep == 0) {
          effectivePhase = "boost";
        } else if (previousPhaseStep == 1) {
          effectivePhase = "descent";
        } else if (previousPhaseStep == 2) {
          effectivePhase = "landed";
        } else {
          effectivePhase = previousPhase;
        }
      }
    }

    if (allowPhaseResetToIdle) {
      phaseResetRequested_ = false;
    }

    state_.flight.phase = effectivePhase;

    if (phaseChecklistCleared_) {
      const bool previousIdleOrPad = phaseEquals(previousPhase, "idle") || phaseEquals(previousPhase, "pad");
      const int8_t effectiveStep = phaseProgressStep(state_.flight.phase);
      if (previousIdleOrPad && effectiveStep > 0) {
        phaseChecklistCleared_ = false;
      }
    }

    syncCommandStateFromTelemetry();

    const bool phaseChanged = (state_.flight.phase != previousPhase && state_.flight.phase.length() > 0);
    handleEventSoundTriggers(previousPhase, state_.flight.phase, phaseChanged);

    if (phaseChanged) {
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
  updateFlightTimerState(now);
  playNextQueuedSound();
  if (apogeeCalloutPending_ && soundEnabled_ && soundQueueCount_ == 0 &&
      apogeeCalloutReadyAtMs_ != 0 && now >= apogeeCalloutReadyAtMs_) {
    (void)playApogeeAltitudeCallout();
    apogeeCalloutPending_ = false;
    apogeeCalloutReadyAtMs_ = 0;
  }

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
    data->point.x = static_cast<lv_coord_t>(self->lastTouchScreenX_);
    data->point.y = static_cast<lv_coord_t>(self->lastTouchScreenY_);
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
    data->point.x = static_cast<lv_coord_t>(self->lastTouchScreenX_);
    data->point.y = static_cast<lv_coord_t>(self->lastTouchScreenY_);
    return;
  }

  self->touchDebugMapOk_ = true;
  self->touchDebugMapX_ = x;
  self->touchDebugMapY_ = y;
  self->lastTouchScreenX_ = x;
  self->lastTouchScreenY_ = y;

  data->state = LV_INDEV_STATE_PR;
  data->point.x = x;
  data->point.y = y;
}

void LvglController::onPanelToggleEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  self->togglePanel();
}

void LvglController::onSettingsToggleEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  self->toggleSettings();
}

void LvglController::onTouchDebugToggleEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  lv_obj_t* target = static_cast<lv_obj_t*>(lv_event_get_target(e));
  if (self == nullptr || target == nullptr) {
    return;
  }
  self->setTouchDebugVisible(lv_obj_has_state(target, LV_STATE_CHECKED));
}

void LvglController::onArmNoGpsToggleEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  lv_obj_t* target = static_cast<lv_obj_t*>(lv_event_get_target(e));
  if (self == nullptr || target == nullptr) {
    return;
  }
  self->setAllowArmWithoutGpsFix(lv_obj_has_state(target, LV_STATE_CHECKED));
  self->refreshUi();
}

void LvglController::onWifiApToggleEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  self->sendAction("wifi_ap_toggle", 0);
  self->refreshUi();
}

void LvglController::onSdFunctionsOpenEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  self->setSdFunctionsVisible(true);
}

void LvglController::onSdFunctionsBackEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  self->setSdFunctionsVisible(false);
}

void LvglController::onCalibrateEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  self->startCalibration();
}

void LvglController::onCalibrationCancelEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  self->cancelCalibration();
  self->setCommandStatus("Calibration cancelled", true);
  self->refreshUi();
}

void LvglController::onPhaseResetEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }

  const lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_PRESSED) {
    self->resetFlightArmed_ = true;
    self->resetFlightArmSinceMs_ = millis();
    self->setCommandStatus("Hold RESET FLIGHT 4s to confirm", true);
    self->refreshUi();
    return;
  }

  if (code == LV_EVENT_RELEASED) {
    if (self->resetFlightArmed_) {
      self->resetFlightArmed_ = false;
      self->setCommandStatus("RESET FLIGHT canceled", true);
      self->refreshUi();
    }
    return;
  }

  if (code != LV_EVENT_PRESSING || !self->resetFlightArmed_) {
    return;
  }

  const uint32_t now = millis();
  if ((now - self->resetFlightArmSinceMs_) < kResetFlightArmWindowMs) {
    return;
  }

  self->resetFlightArmed_ = false;

  if (self->sendAction("phase_reset", 0)) {
    self->soundQueueHead_ = 0;
    self->soundQueueTail_ = 0;
    self->soundQueueCount_ = 0;
    self->apogeeCalloutPending_ = false;
    self->phaseResetRequested_ = true;
    self->phaseChecklistCleared_ = true;
    self->flightTimerActive_ = false;
    self->flightTimerStartMs_ = 0;
    self->flightDurationMs_ = 0;
    self->flightTimerInitialized_ = true;
    self->lastFlightLaunchDetected_ = false;
    self->lastFlightLandingDetected_ = false;
    self->queueSoundCue(SoundCue::kPowerOn);
    self->queueSoundCue(SoundCue::kCalibrating);
    self->queueSoundCue(SoundCue::kSensorsReady);

    self->hasRecoveryDeployHistory_ = true;
    self->lastRecoveryDrogueDeployed_ = self->state_.recoveryDrogueDeployed;
    self->lastRecoveryMainDeployed_ = self->state_.recoveryMainDeployed;
    self->hasRecoveryEventHistory_ = true;
    self->lastRecoveryLaunchDetected_ = self->state_.recoveryLaunchDetected;
    self->lastRecoveryApogee_ = self->state_.recoveryApogee;
    self->lastRecoveryLandingDetected_ = self->state_.recoveryLandingDetected;
  }

  self->panelCollapsed_ = true;
  self->settingsCollapsed_ = true;
  self->setSoundSettingsVisible(false);
  self->setSdFunctionsVisible(false);
  if (self->actionPanel_ != nullptr) {
    lv_obj_add_flag(self->actionPanel_, LV_OBJ_FLAG_HIDDEN);
  }
  if (self->settingsBody_ != nullptr) {
    lv_obj_add_flag(self->settingsBody_, LV_OBJ_FLAG_HIDDEN);
  }
  if (self->telemetryPanel_ != nullptr) {
    lv_obj_update_layout(self->telemetryPanel_);
  }

  self->refreshUi();
}

void LvglController::onImuCalibrateEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  if (self->sendAction("imu_calibrate", 0)) {
    self->queueSoundCue(SoundCue::kCalibrating);
    self->queueSoundCue(SoundCue::kSensorsReady);
  }

  self->panelCollapsed_ = true;
  self->settingsCollapsed_ = true;
  self->setSoundSettingsVisible(false);
  self->setSdFunctionsVisible(false);
  if (self->actionPanel_ != nullptr) {
    lv_obj_add_flag(self->actionPanel_, LV_OBJ_FLAG_HIDDEN);
  }
  if (self->settingsBody_ != nullptr) {
    lv_obj_add_flag(self->settingsBody_, LV_OBJ_FLAG_HIDDEN);
  }
  if (self->telemetryPanel_ != nullptr) {
    lv_obj_update_layout(self->telemetryPanel_);
  }

  self->refreshUi();
}

void LvglController::onShutdownEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  if (self->commandLockoutActive_) {
    self->setCommandStatus("Shutdown locked until landing", false);
    self->refreshUi();
    return;
  }
  self->sendAction("shutdown", 0);
  self->refreshUi();
}

void LvglController::onRebootEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  if (self->commandLockoutActive_) {
    self->setCommandStatus("Reboot locked until landing", false);
    self->refreshUi();
    return;
  }
  self->sendAction("reboot", 0);
  self->refreshUi();
}

void LvglController::onBuzzerToggleEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  self->setBuzzerConfigVisible(!self->buzzerConfigVisible_);
}

void LvglController::onBuzzerDurationChangedEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr || self->buzzerDurationSlider_ == nullptr) {
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
  if (self == nullptr) {
    return;
  }
  self->sendAction("buzzer", self->buzzerDurationS_);
  self->refreshUi();
}

void LvglController::onTxPowerChangedEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr || self->txPowerSlider_ == nullptr) {
    return;
  }
  int value = lv_slider_get_value(self->txPowerSlider_);
  if (value < kTelemetryTxPowerMinDbm) {
    value = kTelemetryTxPowerMinDbm;
  } else if (value > kTelemetryTxPowerMaxDbm) {
    value = kTelemetryTxPowerMaxDbm;
  }
  self->txPowerDbm_ = static_cast<uint8_t>(value);
  if (self->txPowerLabel_ != nullptr) {
    lv_label_set_text_fmt(self->txPowerLabel_, "%udBm", static_cast<unsigned>(self->txPowerDbm_));
  }
}

void LvglController::onTxPowerSendEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  self->sendAction("telemetry_tx_power", self->txPowerDbm_);
  self->refreshUi();
}

void LvglController::onArmEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  self->requestArmLaunch();
  self->refreshUi();
}

void LvglController::onSoundSettingsOpenEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  self->setSdFunctionsVisible(false);
  self->setSoundSettingsVisible(true);
}

void LvglController::onSoundSettingsBackEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  self->setSoundSettingsVisible(false);
}

void LvglController::onSdRotateEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  self->requestSdRotate();
  self->refreshUi();
}

void LvglController::onSdFormatEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }

  const lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_PRESSED) {
    if (self->sdLoggingEnabled_) {
      self->setCommandStatus("Stop SD logging before format", false);
    } else if (self->commandLockoutActive_) {
      self->setCommandStatus("SD format locked until landing", false);
    } else {
      self->sdFormatArmed_ = true;
      self->sdFormatArmSinceMs_ = millis();
      self->setCommandStatus("Hold FORMAT to confirm", true);
    }
    self->updateDashboardActionButtons();
    self->refreshUi();
    return;
  }

  if (code == LV_EVENT_LONG_PRESSED) {
    const uint32_t now = millis();
    if (!self->sdFormatArmed_ || (now - self->sdFormatArmSinceMs_) > kSdFormatArmWindowMs) {
      self->sdFormatArmed_ = false;
      self->setCommandStatus("Tap then hold FORMAT to confirm", false);
      self->updateDashboardActionButtons();
      self->refreshUi();
      return;
    }
    self->requestSdFormat();
    self->refreshUi();
  }
}

void LvglController::onSdDumpSampleEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  self->requestSdDumpSample();
  self->refreshUi();
}

void LvglController::onSdDumpCloseEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  self->setSdDumpOverlayVisible(false);
  self->refreshUi();
}

void LvglController::onLaunchPrepEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }

  const lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_PRESSED) {
    if (self->sdCommandPending_ || self->txCommandPending_) {
      self->setCommandStatus("Wait for pending TX/SD command", false);
      self->refreshUi();
      return;
    }
    if (!self->state_.recoveryGpsFix3d && !self->allowArmWithoutGpsFix_) {
      self->setCommandStatus("PREP blocked: waiting for GPS 3D fix", false);
      self->refreshUi();
      return;
    }
    if (self->state_.recoveryLaunchArmed) {
      self->setCommandStatus("Launch detect already armed", true);
      self->refreshUi();
      return;
    }
    if (self->commandLockoutActive_) {
      self->setCommandStatus("Prep command locked in flight", false);
      self->refreshUi();
      return;
    }

    self->launchPrepArmed_ = true;
    self->launchPrepArmSinceMs_ = millis();
    self->setCommandStatus("Hold PREP+ARM to confirm", true);
    self->updateDashboardActionButtons();
    self->refreshUi();
    return;
  }

  if (code != LV_EVENT_LONG_PRESSED) {
    return;
  }

  const uint32_t now = millis();
  if (!self->launchPrepArmed_ || (now - self->launchPrepArmSinceMs_) > kLaunchPrepArmWindowMs) {
    self->launchPrepArmed_ = false;
    self->setCommandStatus("Tap then hold PREP+ARM to confirm", false);
    self->updateDashboardActionButtons();
    self->refreshUi();
    return;
  }

  self->launchPrepArmed_ = false;
  bool requestedTxOn = false;
  bool requestedSdOn = false;

  if (!self->telemetryTxEnabled_) {
    self->requestTxToggle(true);
    requestedTxOn = true;
  }
  if (!self->sdLoggingEnabled_) {
    self->requestSdToggle(true);
    requestedSdOn = true;
  }
  const bool armRequested = self->requestArmLaunch();

  if (armRequested && (requestedTxOn || requestedSdOn)) {
    self->setCommandStatus("PREP requested: TX ON + SD ON + ARM", true);
  } else if (armRequested) {
    self->setCommandStatus("ARM requested (TX/SD already ON)", true);
  } else if (requestedTxOn || requestedSdOn) {
    self->setCommandStatus("PREP partial: TX/SD requested, ARM failed", false);
  }

  self->updateDashboardActionButtons();
  self->refreshUi();
}

void LvglController::onSoundEnableToggleEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  lv_obj_t* target = static_cast<lv_obj_t*>(lv_event_get_target(e));
  if (self == nullptr || target == nullptr) {
    return;
  }
  self->setSoundEnabled(lv_obj_has_state(target, LV_STATE_CHECKED));
  self->refreshUi();
}

void LvglController::onSoundTestEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
  const char* failReason = nullptr;
  const bool ok = self->playWavFromSd(COMPANION_SOUND_FILE_AUDIO_TEST, &failReason);
  if (ok) {
    self->setCommandStatus("AUDIO TEST played", true);
  } else {
    String msg = "AUDIO TEST failed";
    if (failReason != nullptr && failReason[0] != '\0') {
      msg += " (";
      msg += failReason;
      msg += ")";
    }
    self->setCommandStatus(msg, false);
    Serial.printf("[audio-test] failed: %s | path=%s | sd_ready=%d | audio_ready=%d\n",
                  (failReason != nullptr) ? failReason : "unknown",
                  COMPANION_SOUND_FILE_AUDIO_TEST,
                  self->sdStorageReady_ ? 1 : 0,
                  self->audioOutputReady_ ? 1 : 0);
  }
  self->refreshUi();
}

void LvglController::onSoundVolumeChangedEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr || self->soundVolumeSlider_ == nullptr) {
    return;
  }
  int value = lv_slider_get_value(self->soundVolumeSlider_);
  if (value < 0) {
    value = 0;
  } else if (value > 100) {
    value = 100;
  }
  self->audioVolumePercent_ = static_cast<uint8_t>(value);
  self->prefs_.putUChar("sound_volume", self->audioVolumePercent_);
  if (self->soundVolumeLabel_ != nullptr) {
    lv_label_set_text_fmt(self->soundVolumeLabel_, "%u%%", static_cast<unsigned>(self->audioVolumePercent_));
  }
}

void LvglController::onSdToggleEvent(lv_event_t* e) {
  LvglController* self = static_cast<LvglController*>(lv_event_get_user_data(e));
  if (self == nullptr) {
    return;
  }
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
  if (self == nullptr) {
    return;
  }
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
