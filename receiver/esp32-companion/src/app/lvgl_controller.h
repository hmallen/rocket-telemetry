#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <TFT_eSPI.h>
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
  enum class SoundCue : uint8_t {
    kArmed = 0,
    kCalibrating,
    kSensorsReady,
    kWaitingForLocationFix,
    kLocationFixAcquired,
    kLaunchDetectMode,
    kLaunchDetected,
    kApogee,
    kDrogueDeploy,
    kMainDeploy,
    KLandingDetected,
    kHomePointSet,
  };

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
  static constexpr int kActionPanelWidth = 210;
  static constexpr int kSettingsPanelWidth = 252;

  ApiClient api_;
  UartLink uart_;
  CompanionState state_;
  TFT_eSPI& tft_;
  Preferences prefs_;
#if defined(ESP32)
  SPIClass sdSpi_{HSPI};
#endif

  bool sdLoggingEnabled_ = false;
  bool telemetryTxEnabled_ = false;
  bool sdCommandPending_ = false;
  bool sdPendingTargetEnabled_ = false;
  bool sdPendingPreviousEnabled_ = false;
  uint32_t sdPendingSinceMs_ = 0;
  bool txCommandPending_ = false;
  bool txPendingTargetEnabled_ = false;
  bool txPendingPreviousEnabled_ = false;
  uint32_t txPendingSinceMs_ = 0;
  uint8_t txPowerDbm_ = 17;
  bool hasTxPowerReadback_ = false;
  uint8_t txPowerActiveDbm_ = 0;
  bool commandLockoutActive_ = false;
  bool sdStorageReady_ = false;
  uint64_t sdStorageTotalBytes_ = 0;
  uint64_t sdStorageUsedBytes_ = 0;
  bool audioOutputReady_ = false;
  uint16_t audioPwmMaxDuty_ = 255;
  bool soundEnabled_ = true;
  bool hasRecoveryDeployHistory_ = false;
  bool lastRecoveryDrogueDeployed_ = false;
  bool lastRecoveryMainDeployed_ = false;
  bool recoveryLaunchArmed_ = false;
  bool recoveryGpsFix3d_ = false;
  bool allowArmWithoutGpsFix_ = false;
  bool phaseResetRequested_ = false;
  float maxObservedAglM_ = NAN;
  bool apogeeCalloutPending_ = false;
  static constexpr uint8_t kSoundQueueCapacity = 16;
  SoundCue soundQueue_[kSoundQueueCapacity] = {};
  uint8_t soundQueueHead_ = 0;
  uint8_t soundQueueTail_ = 0;
  uint8_t soundQueueCount_ = 0;

  uint8_t buzzerDurationS_ = 3;
  bool buzzerConfigVisible_ = false;

  bool sseConnected_ = false;
  uint32_t lastRxMs_ = 0;
  int32_t loraAgeBaseMs_ = -1;
  uint32_t loraAgeBaseTickMs_ = 0;
  bool hasLastLoraPacketCount_ = false;
  uint32_t lastLoraPacketCount_ = 0;
  uint32_t lastReconnectAttemptMs_ = 0;
  uint32_t lastCompanionBatSampleMs_ = 0;
  uint32_t lastCompanionBatRawMv_ = 0;
  bool lastCompanionBatRawValid_ = false;
  uint32_t lastLvTickMs_ = 0;
  uint32_t lastUiRefreshMs_ = 0;

  String lastPhase_;
  bool lastConnected_ = false;
  bool lastStale_ = true;
  String lastPrimaryAlert_;

  uint8_t* drawBufPixels_ = nullptr;
  lv_display_t* display_ = nullptr;
  lv_indev_t* touchIndev_ = nullptr;

  TouchCalibration touchCal_{};

  bool panelCollapsed_ = true;
  bool settingsCollapsed_ = true;
  bool soundSettingsVisible_ = false;

  bool calibrationActive_ = false;
  bool calibrationTouchLatch_ = false;
  uint8_t calibrationStep_ = 0;
  int32_t calibrationRawX_[4] = {0, 0, 0, 0};
  int32_t calibrationRawY_[4] = {0, 0, 0, 0};
  int32_t calibrationLastRawX_ = -1;
  int32_t calibrationLastRawY_ = -1;
  uint32_t calibrationLastSampleMs_ = 0;

  bool touchDebugPressed_ = false;
  bool touchDebugIrqPressed_ = false;
  bool touchDebugMapOk_ = false;
  bool touchDebugVisible_ = false;
  int32_t touchDebugRawX_ = -1;
  int32_t touchDebugRawY_ = -1;
  int32_t touchDebugRawZ_ = -1;
  int32_t touchDebugMapX_ = -1;
  int32_t touchDebugMapY_ = -1;
  uint32_t touchDebugTs_ = 0;
  bool touchPressed_ = false;
  int32_t lastTouchScreenX_ = 0;
  int32_t lastTouchScreenY_ = 0;

  String cmdMsg_;
  bool cmdOk_ = true;
  uint32_t cmdTs_ = 0;

  lv_obj_t* root_ = nullptr;
  lv_obj_t* telemetryPanel_ = nullptr;
  lv_obj_t* sdToggleBtn_ = nullptr;
  lv_obj_t* sdToggleLabel_ = nullptr;
  lv_obj_t* txToggleBtn_ = nullptr;
  lv_obj_t* txToggleLabel_ = nullptr;
  lv_obj_t* actionPanel_ = nullptr;
  lv_obj_t* actionContent_ = nullptr;
  lv_obj_t* buzzerConfigRow_ = nullptr;
  lv_obj_t* buzzerDurationSlider_ = nullptr;
  lv_obj_t* buzzerDurationLabel_ = nullptr;
  lv_obj_t* txPowerConfigRow_ = nullptr;
  lv_obj_t* txPowerSlider_ = nullptr;
  lv_obj_t* txPowerLabel_ = nullptr;
  lv_obj_t* txPowerActiveLabel_ = nullptr;
  lv_obj_t* armBtn_ = nullptr;
  lv_obj_t* armLabel_ = nullptr;
  lv_obj_t* shutdownBtn_ = nullptr;
  lv_obj_t* settingsBody_ = nullptr;
  lv_obj_t* settingsActions_ = nullptr;
  lv_obj_t* soundSettingsPanel_ = nullptr;
  lv_obj_t* armNoGpsCheckbox_ = nullptr;
  lv_obj_t* soundEnabledCheckbox_ = nullptr;
  lv_obj_t* soundVolumeSlider_ = nullptr;
  lv_obj_t* soundVolumeLabel_ = nullptr;
  uint8_t audioVolumePercent_ = 100;

  lv_obj_t* linkLabel_ = nullptr;
  lv_obj_t* linkMetaLabel_ = nullptr;
  lv_obj_t* phaseLabel_ = nullptr;
  lv_obj_t* altitudeLabel_ = nullptr;
  lv_obj_t* vsLabel_ = nullptr;
  lv_obj_t* packetLabel_ = nullptr;
  lv_obj_t* callsignLabel_ = nullptr;
  lv_obj_t* batteryLabel_ = nullptr;
  lv_obj_t* companionBatteryLabel_ = nullptr;
  lv_obj_t* companionBatteryDebugLabel_ = nullptr;
  lv_obj_t* cmdStatusLabel_ = nullptr;
  lv_obj_t* alertLabel_ = nullptr;
  lv_obj_t* touchDebugLabel_ = nullptr;

  lv_obj_t* calibrationOverlay_ = nullptr;
  lv_obj_t* calibrationInstrLabel_ = nullptr;
  lv_obj_t* calibrationRawLabel_ = nullptr;
  lv_obj_t* calibrationTarget_ = nullptr;

  void initLvgl();
  void buildUi();
  void refreshUi();
  void setCommandStatus(const String& msg, bool ok);
  void updateDashboardActionButtons();
  void setBuzzerConfigVisible(bool visible);
  void syncCommandStateFromTelemetry();
  void updatePendingCommandTimeouts(uint32_t now);
  void requestSdToggle(bool enable);
  void requestTxToggle(bool enable);

  void updateStaleness();
  void updateCompanionBattery();
  void initSdStorage();
  void initSoundOutput();
  void queueSoundCue(SoundCue cue);
  void handleEventSoundTriggers(const String& previousPhase,
                                const String& currentPhase,
                                bool phaseChanged);
  void playNextQueuedSound();
  bool playNumberCue(int value);
  bool playApogeeAltitudeCallout();
  const char* cueFilePath(SoundCue cue) const;
  bool playWavFromSd(const char* path, const char** failReason = nullptr);
  bool ensureConnected();

  bool sendAction(const String& action, int durationS = 0);
  void togglePanel();
  void toggleSettings();
  void setAllowArmWithoutGpsFix(bool enabled);
  void setSoundEnabled(bool enabled);
  void setSoundSettingsVisible(bool visible);
  void setTouchDebugVisible(bool visible);

  void loadTouchCalibration();
  void saveTouchCalibration();
  bool readTouchRaw(int32_t& rawX, int32_t& rawY, int32_t& rawZ);
  bool mapTouchToScreen(int rawX, int rawY, int& outX, int& outY) const;

  void startCalibration();
  void cancelCalibration();
  void handleCalibrationTouch();
  void advanceCalibrationTarget();
  bool completeCalibration();

  static void flushDisplayCb(lv_display_t* disp, const lv_area_t* area, uint8_t* pxMap);
  static void readTouchCb(lv_indev_t* indev, lv_indev_data_t* data);

  static void onPanelToggleEvent(lv_event_t* e);
  static void onSettingsToggleEvent(lv_event_t* e);
  static void onTouchDebugToggleEvent(lv_event_t* e);
  static void onCalibrateEvent(lv_event_t* e);
  static void onAltCalibrateEvent(lv_event_t* e);
  static void onImuCalibrateEvent(lv_event_t* e);
  static void onShutdownEvent(lv_event_t* e);
  static void onBuzzerToggleEvent(lv_event_t* e);
  static void onBuzzerDurationChangedEvent(lv_event_t* e);
  static void onBuzzerSendEvent(lv_event_t* e);
  static void onTxPowerChangedEvent(lv_event_t* e);
  static void onTxPowerSendEvent(lv_event_t* e);
  static void onArmEvent(lv_event_t* e);
  static void onArmNoGpsToggleEvent(lv_event_t* e);
  static void onSoundSettingsOpenEvent(lv_event_t* e);
  static void onSoundSettingsBackEvent(lv_event_t* e);
  static void onSoundEnableToggleEvent(lv_event_t* e);
  static void onSoundTestEvent(lv_event_t* e);
  static void onSoundVolumeChangedEvent(lv_event_t* e);
  static void onSdToggleEvent(lv_event_t* e);
  static void onTxToggleEvent(lv_event_t* e);
};
