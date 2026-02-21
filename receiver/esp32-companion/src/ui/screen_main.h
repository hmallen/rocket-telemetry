#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <lvgl.h>

#include "../model/telemetry_state.h"

/**
 * Command callback type. Called by UI button handlers when the user presses
 * a command button (buzz, SD toggle). Returns true if the command was sent
 * successfully.
 */
typedef bool (*CmdCallback)(const char* action, int durationS);

/**
 * LVGL-based main screen for the CrowPanel 3.5" 480x320 companion display.
 *
 * Manages:
 *   - LVGL display and touch driver registration
 *   - Telemetry dashboard (header + data area + battery strip + alert strip)
 *   - Collapsible actions panel (BUZZ / SD buttons, slides in from right)
 *   - Settings overlay (brightness slider, touch calibration, info)
 */
class MainScreen {
 public:
  explicit MainScreen(TFT_eSPI& tft);

  /** Init TFT, LVGL drivers, and build the complete UI. */
  void begin();

  /** Push fresh telemetry state into LVGL labels. Call every tick. */
  void update(const CompanionState& state);

  /** Append a message to the cycling alert strip. */
  void pushAlert(const String& msg);

  /** Show a command result message briefly in the battery strip. */
  void setCommandStatus(const String& msg, bool ok);

  /** Register the callback that fires when a command button is pressed. */
  void setCommandCallback(CmdCallback cb) { cmdCallback_ = cb; }

  /** Returns whether SD logging is currently toggled on. */
  bool isSdLoggingEnabled() const { return sdLoggingEnabled_; }

 private:
  // ── Layout constants (480×320 landscape) ────────────────────────────────
  static constexpr int kW = 480;
  static constexpr int kH = 320;
  static constexpr int kHeaderH = 28;
  static constexpr int kBatH = 22;
  static constexpr int kAlertH = 22;
  static constexpr int kMainH = kH - kHeaderH - kBatH - kAlertH;  // 248
  static constexpr int kMainY = kHeaderH;
  static constexpr int kBatY = kMainY + kMainH;   // 276
  static constexpr int kAlertY = kBatY + kBatH;   // 298

  // Actions panel (right-side overlay when visible)
  static constexpr int kActPanelW = 190;
  static constexpr int kActPanelX = kW - kActPanelW;  // 290

  // Toggle tab: a small handle always visible at right edge of main area
  static constexpr int kToggleW = 24;
  static constexpr int kToggleH = 60;
  static constexpr int kToggleX = kW - kToggleW;    // 456
  static constexpr int kToggleY = kMainY + (kMainH - kToggleH) / 2;  // centred

  // ── TFT + LVGL ──────────────────────────────────────────────────────────
  TFT_eSPI& tft_;

  static lv_disp_draw_buf_t drawBuf_;
  // ~10 lines of 480px × 2 bytes each = 9600 bytes
  static lv_color_t lvBuf_[480 * 10];
  static lv_disp_drv_t dispDrv_;
  static lv_indev_drv_t indevDrv_;

  // ── Dashboard objects ────────────────────────────────────────────────────
  lv_obj_t* dashScreen_ = nullptr;

  // Header
  lv_obj_t* headerBar_ = nullptr;
  lv_obj_t* lblLinkDot_ = nullptr;   // "●" coloured by link state
  lv_obj_t* lblRssi_ = nullptr;
  lv_obj_t* lblSnr_ = nullptr;
  lv_obj_t* lblAge_ = nullptr;
  lv_obj_t* lblPkt_ = nullptr;
  lv_obj_t* btnGear_ = nullptr;      // ⚙ opens settings

  // Main data area
  lv_obj_t* lblPhase_ = nullptr;
  lv_obj_t* lblAlt_ = nullptr;       // altitude AGL — very large
  lv_obj_t* lblVs_ = nullptr;        // vertical speed
  lv_obj_t* lblMaxAlt_ = nullptr;    // max altitude tracker

  // Battery / status strip
  lv_obj_t* batStrip_ = nullptr;
  lv_obj_t* lblTxBat_ = nullptr;
  lv_obj_t* lblCompBat_ = nullptr;
  lv_obj_t* lblCmdStatus_ = nullptr;

  // Alert strip
  lv_obj_t* alertStrip_ = nullptr;
  lv_obj_t* lblAlert_ = nullptr;

  // Actions panel (collapsible)
  lv_obj_t* actionsPanel_ = nullptr;
  lv_obj_t* btnActionsToggle_ = nullptr;  // "> / <" tab
  lv_obj_t* lblActToggle_ = nullptr;
  lv_obj_t* btnBuzz1_ = nullptr;
  lv_obj_t* btnBuzz5_ = nullptr;
  lv_obj_t* btnSd_ = nullptr;
  lv_obj_t* lblSdState_ = nullptr;
  bool actionsVisible_ = false;

  // Settings overlay (full-screen)
  lv_obj_t* settingsOverlay_ = nullptr;
  lv_obj_t* sliderBrightness_ = nullptr;
  lv_obj_t* lblBrightnessVal_ = nullptr;
  lv_obj_t* lblCalStatus_ = nullptr;
  bool settingsVisible_ = false;

  // ── State ────────────────────────────────────────────────────────────────
  CmdCallback cmdCallback_ = nullptr;
  bool sdLoggingEnabled_ = false;
  float maxAltAgl_ = 0.0f;

  // Alert ring buffer
  String alerts_[4];
  int alertCount_ = 0;

  // Command status
  String cmdMsg_;
  bool cmdOk_ = true;
  uint32_t cmdTs_ = 0;

  // ── Helpers ──────────────────────────────────────────────────────────────
  void initStyles();
  void createDashboard();
  void createHeader(lv_obj_t* parent);
  void createDataArea(lv_obj_t* parent);
  void createBatteryStrip(lv_obj_t* parent);
  void createAlertStrip(lv_obj_t* parent);
  void createActionsPanel(lv_obj_t* parent);
  void createSettingsOverlay(lv_obj_t* parent);

  void showActions(bool visible);
  void showSettings(bool visible);
  void refreshCmdStatus();
  void updateAlertLabel();

  lv_obj_t* makeBtn(lv_obj_t* parent, const char* label,
                    int x, int y, int w, int h,
                    lv_color_t bgColor, lv_event_cb_t cb);

  // ── Singleton for static LVGL callbacks ─────────────────────────────────
  static MainScreen* instance_;
  static TFT_eSPI* tftInstance_;

  // LVGL driver callbacks
  static void flushCb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p);
  static void touchReadCb(lv_indev_drv_t* indev, lv_indev_data_t* data);

  // Button event callbacks
  static void onBuzz1(lv_event_t* e);
  static void onBuzz5(lv_event_t* e);
  static void onSdToggle(lv_event_t* e);
  static void onActionsToggle(lv_event_t* e);
  static void onSettingsOpen(lv_event_t* e);
  static void onSettingsClose(lv_event_t* e);
  static void onBrightness(lv_event_t* e);
  static void onCalibrate(lv_event_t* e);
};
