#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <lvgl.h>

#include "config.h"

#include "../model/telemetry_state.h"
#include "../net/api_client.h"
#include "../serial/uart_link.h"

class LvglController {
 public:
  LvglController(TFT_eSPI& tft, const String& host, uint16_t port);
  void begin();
  void tick();

 private:
  struct TouchCalibration {
    int32_t xMin = TOUCH_X_MIN;
    int32_t xMax = TOUCH_X_MAX;
    int32_t yMin = TOUCH_Y_MIN;
    int32_t yMax = TOUCH_Y_MAX;
    bool swapXY = TOUCH_SWAP_XY;
  };

  static constexpr int kScreenWidth = 480;
  static constexpr int kScreenHeight = 320;
  static constexpr int kDrawBufferLines = 28;
  static constexpr int kActionPanelWidth = 172;

  ApiClient api_;
  UartLink uart_;
  CompanionState state_;
  TFT_eSPI& tft_;
  XPT2046_Touchscreen touch_;
  Preferences prefs_;

  bool sdLoggingEnabled_ = false;
  bool telemetryTxEnabled_ = true;

  bool sseConnected_ = false;
  uint32_t lastRxMs_ = 0;
  uint32_t lastReconnectAttemptMs_ = 0;
  uint32_t lastCompanionBatSampleMs_ = 0;
  uint32_t lastLvTickMs_ = 0;
  uint32_t lastUiRefreshMs_ = 0;

  String lastPhase_;
  bool lastConnected_ = false;
  bool lastStale_ = true;
  String lastPrimaryAlert_;

  lv_disp_draw_buf_t drawBuf_{};
  lv_color_t* drawBufPixels_ = nullptr;
  lv_disp_drv_t dispDrv_{};
  lv_indev_drv_t indevDrv_{};
  lv_indev_t* touchIndev_ = nullptr;

  TouchCalibration touchCal_{};

  bool panelCollapsed_ = false;
  bool settingsCollapsed_ = true;

  bool calibrationActive_ = false;
  bool calibrationTouchLatch_ = false;
  uint8_t calibrationStep_ = 0;
  int32_t calibrationRawX_[4] = {0, 0, 0, 0};
  int32_t calibrationRawY_[4] = {0, 0, 0, 0};

  String cmdMsg_;
  bool cmdOk_ = true;
  uint32_t cmdTs_ = 0;

  lv_obj_t* root_ = nullptr;
  lv_obj_t* telemetryPanel_ = nullptr;
  lv_obj_t* actionPanel_ = nullptr;
  lv_obj_t* actionContent_ = nullptr;
  lv_obj_t* settingsBody_ = nullptr;
  lv_obj_t* settingsToggleLabel_ = nullptr;
  lv_obj_t* panelToggleLabel_ = nullptr;
  lv_obj_t* panelQuickToggle_ = nullptr;
  lv_obj_t* panelQuickToggleLabel_ = nullptr;

  lv_obj_t* linkLabel_ = nullptr;
  lv_obj_t* linkMetaLabel_ = nullptr;
  lv_obj_t* phaseLabel_ = nullptr;
  lv_obj_t* altitudeLabel_ = nullptr;
  lv_obj_t* vsLabel_ = nullptr;
  lv_obj_t* packetLabel_ = nullptr;
  lv_obj_t* callsignLabel_ = nullptr;
  lv_obj_t* batteryLabel_ = nullptr;
  lv_obj_t* companionBatteryLabel_ = nullptr;
  lv_obj_t* cmdStatusLabel_ = nullptr;
  lv_obj_t* alertLabel_ = nullptr;

  lv_obj_t* calibrationOverlay_ = nullptr;
  lv_obj_t* calibrationInstrLabel_ = nullptr;
  lv_obj_t* calibrationRawLabel_ = nullptr;
  lv_obj_t* calibrationTarget_ = nullptr;

  void initLvgl();
  void buildUi();
  void refreshUi();
  void setCommandStatus(const String& msg, bool ok);

  void updateStaleness();
  void updateCompanionBattery();
  bool ensureConnected();

  void sendAction(const String& action, int durationS = 0);
  void togglePanel();
  void toggleSettings();

  void loadTouchCalibration();
  void saveTouchCalibration();
  bool mapTouchToScreen(int rawX, int rawY, int& outX, int& outY) const;

  void startCalibration();
  void cancelCalibration();
  void handleCalibrationTouch();
  void advanceCalibrationTarget();
  bool completeCalibration();

  static void flushDisplayCb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* colorP);
  static void readTouchCb(lv_indev_drv_t* indev, lv_indev_data_t* data);

  static void onPanelToggleEvent(lv_event_t* e);
  static void onSettingsToggleEvent(lv_event_t* e);
  static void onCalibrateEvent(lv_event_t* e);
  static void onBuzz1Event(lv_event_t* e);
  static void onBuzz5Event(lv_event_t* e);
  static void onSdStartEvent(lv_event_t* e);
  static void onSdStopEvent(lv_event_t* e);
  static void onTelemEnableEvent(lv_event_t* e);
  static void onTelemDisableEvent(lv_event_t* e);
};
