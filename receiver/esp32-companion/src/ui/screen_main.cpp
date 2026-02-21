#include "screen_main.h"

#include <math.h>
#include <Preferences.h>

#include "config.h"

// ── Static member definitions ─────────────────────────────────────────────
MainScreen* MainScreen::instance_       = nullptr;
TFT_eSPI*   MainScreen::tftInstance_    = nullptr;

lv_disp_draw_buf_t MainScreen::drawBuf_;
lv_color_t         MainScreen::lvBuf_[480 * 10];
lv_disp_drv_t      MainScreen::dispDrv_;
lv_indev_drv_t     MainScreen::indevDrv_;

// ── Construction ──────────────────────────────────────────────────────────
MainScreen::MainScreen(TFT_eSPI& tft) : tft_(tft) {}

// ── LVGL driver callbacks ─────────────────────────────────────────────────

void MainScreen::flushCb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
  if (!tftInstance_) { lv_disp_flush_ready(disp); return; }
  uint32_t w = static_cast<uint32_t>(area->x2 - area->x1 + 1);
  uint32_t h = static_cast<uint32_t>(area->y2 - area->y1 + 1);
  tftInstance_->startWrite();
  tftInstance_->setAddrWindow(area->x1, area->y1, w, h);
  // LV_COLOR_16_SWAP=1 means bytes are already swapped; pass swap=false to TFT_eSPI
  tftInstance_->pushColors(reinterpret_cast<uint16_t*>(&color_p->full), w * h, false);
  tftInstance_->endWrite();
  lv_disp_flush_ready(disp);
}

void MainScreen::touchReadCb(lv_indev_drv_t* /*indev*/, lv_indev_data_t* data) {
  if (!tftInstance_) { data->state = LV_INDEV_STATE_REL; return; }
  uint16_t x = 0, y = 0;
  bool touched = tftInstance_->getTouch(&x, &y, 600);
  if (!touched) {
    data->state = LV_INDEV_STATE_REL;
  } else {
    data->state   = LV_INDEV_STATE_PR;
    data->point.x = static_cast<lv_coord_t>(x);
    data->point.y = static_cast<lv_coord_t>(y);
  }
}

// ── begin() ───────────────────────────────────────────────────────────────

void MainScreen::begin() {
  instance_    = this;
  tftInstance_ = &tft_;

  // TFT init
  tft_.init();
  tft_.setRotation(1);   // landscape 480×320
  tft_.fillScreen(TFT_BLACK);

  // Backlight on (GPIO 27, HIGH = on)
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);

  // Load touch calibration from NVS if saved
  {
    Preferences prefs;
    prefs.begin("touch_cal", /*readOnly=*/true);
    if (prefs.isKey("cal")) {
      uint16_t calData[5];
      prefs.getBytes("cal", calData, sizeof(calData));
      tft_.setTouch(calData);
    }
    prefs.end();
  }

  // LVGL init
  lv_init();

  lv_disp_draw_buf_init(&drawBuf_, lvBuf_, nullptr, 480 * 10);

  lv_disp_drv_init(&dispDrv_);
  dispDrv_.hor_res  = kW;
  dispDrv_.ver_res  = kH;
  dispDrv_.flush_cb = flushCb;
  dispDrv_.draw_buf = &drawBuf_;
  lv_disp_drv_register(&dispDrv_);

  // Dark default theme
  lv_theme_t* theme = lv_theme_default_init(
      lv_disp_get_default(),
      lv_palette_main(LV_PALETTE_BLUE),
      lv_palette_main(LV_PALETTE_CYAN),
      /*dark=*/true,
      &lv_font_montserrat_14);
  lv_disp_set_theme(lv_disp_get_default(), theme);

  // Touch input
  lv_indev_drv_init(&indevDrv_);
  indevDrv_.type    = LV_INDEV_TYPE_POINTER;
  indevDrv_.read_cb = touchReadCb;
  lv_indev_drv_register(&indevDrv_);

  // Build UI
  createDashboard();
}

// ── UI creation ───────────────────────────────────────────────────────────

lv_obj_t* MainScreen::makeBtn(lv_obj_t* parent, const char* label,
                               int x, int y, int w, int h,
                               lv_color_t bgColor, lv_event_cb_t cb) {
  lv_obj_t* btn = lv_btn_create(parent);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_size(btn, w, h);
  lv_obj_set_style_bg_color(btn, bgColor, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_color(btn, lv_color_darken(bgColor, LV_OPA_30),
                             LV_PART_MAIN | LV_STATE_PRESSED);
  lv_obj_set_style_radius(btn, 6, LV_PART_MAIN);
  lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(btn, 0, LV_PART_MAIN);
  if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* lbl = lv_label_create(btn);
  lv_label_set_text(lbl, label);
  lv_obj_set_style_text_color(lbl, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_center(lbl);

  return btn;
}

void MainScreen::createDashboard() {
  dashScreen_ = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(dashScreen_, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(dashScreen_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_pad_all(dashScreen_, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(dashScreen_, 0, LV_PART_MAIN);
  lv_obj_clear_flag(dashScreen_, LV_OBJ_FLAG_SCROLLABLE);

  createHeader(dashScreen_);
  createDataArea(dashScreen_);
  createBatteryStrip(dashScreen_);
  createAlertStrip(dashScreen_);
  createActionsPanel(dashScreen_);
  createSettingsOverlay(dashScreen_);

  lv_scr_load(dashScreen_);
}

void MainScreen::createHeader(lv_obj_t* parent) {
  headerBar_ = lv_obj_create(parent);
  lv_obj_set_pos(headerBar_, 0, 0);
  lv_obj_set_size(headerBar_, kW, kHeaderH);
  lv_obj_set_style_bg_color(headerBar_, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(headerBar_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(headerBar_, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(headerBar_, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(headerBar_, 0, LV_PART_MAIN);
  lv_obj_clear_flag(headerBar_, LV_OBJ_FLAG_SCROLLABLE);

  // Link status dot (using LV_SYMBOL_STOP as a filled square)
  lblLinkDot_ = lv_label_create(headerBar_);
  lv_label_set_text(lblLinkDot_, LV_SYMBOL_STOP);
  lv_obj_set_style_text_color(lblLinkDot_, lv_color_hex(0xFF2222), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblLinkDot_, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_pos(lblLinkDot_, 4, 7);

  lblRssi_ = lv_label_create(headerBar_);
  lv_label_set_text(lblRssi_, "RSSI --");
  lv_obj_set_style_text_color(lblRssi_, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblRssi_, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(lblRssi_, 26, 8);

  lblSnr_ = lv_label_create(headerBar_);
  lv_label_set_text(lblSnr_, "SNR --");
  lv_obj_set_style_text_color(lblSnr_, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblSnr_, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(lblSnr_, 118, 8);

  lblAge_ = lv_label_create(headerBar_);
  lv_label_set_text(lblAge_, "AGE ---ms");
  lv_obj_set_style_text_color(lblAge_, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblAge_, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(lblAge_, 202, 8);

  lblPkt_ = lv_label_create(headerBar_);
  lv_label_set_text(lblPkt_, "PKT 0");
  lv_obj_set_style_text_color(lblPkt_, lv_color_hex(0xAAAAAA), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblPkt_, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(lblPkt_, 316, 8);

  // Gear / settings button (top-right corner)
  btnGear_ = lv_btn_create(headerBar_);
  lv_obj_set_pos(btnGear_, kW - 36, 1);
  lv_obj_set_size(btnGear_, 34, 26);
  lv_obj_set_style_bg_color(btnGear_, lv_color_hex(0x444444), LV_PART_MAIN);
  lv_obj_set_style_bg_color(btnGear_, lv_color_hex(0x666666),
                             LV_PART_MAIN | LV_STATE_PRESSED);
  lv_obj_set_style_radius(btnGear_, 4, LV_PART_MAIN);
  lv_obj_set_style_border_width(btnGear_, 0, LV_PART_MAIN);
  lv_obj_add_event_cb(btnGear_, onSettingsOpen, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* gearLbl = lv_label_create(btnGear_);
  lv_label_set_text(gearLbl, LV_SYMBOL_SETTINGS);
  lv_obj_set_style_text_color(gearLbl, lv_color_white(), LV_PART_MAIN);
  lv_obj_center(gearLbl);
}

void MainScreen::createDataArea(lv_obj_t* parent) {
  lv_obj_t* area = lv_obj_create(parent);
  lv_obj_set_pos(area, 0, kMainY);
  lv_obj_set_size(area, kW, kMainH);
  lv_obj_set_style_bg_opa(area, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(area, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(area, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(area, 0, LV_PART_MAIN);
  lv_obj_clear_flag(area, LV_OBJ_FLAG_SCROLLABLE);

  // Flight phase — cyan, medium
  lblPhase_ = lv_label_create(area);
  lv_label_set_text(lblPhase_, "PHASE: ---");
  lv_obj_set_style_text_color(lblPhase_, lv_color_hex(0x00FFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblPhase_, &lv_font_montserrat_20, LV_PART_MAIN);
  lv_obj_set_pos(lblPhase_, 10, 8);

  // Altitude AGL — large amber, dominant
  lblAlt_ = lv_label_create(area);
  lv_label_set_text(lblAlt_, "ALT ---.-- m");
  lv_obj_set_style_text_color(lblAlt_, lv_color_hex(0xFFCC00), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblAlt_, &lv_font_montserrat_40, LV_PART_MAIN);
  lv_obj_set_pos(lblAlt_, 10, 40);

  // Vertical speed — lime green
  lblVs_ = lv_label_create(area);
  lv_label_set_text(lblVs_, "VS  ---.- m/s");
  lv_obj_set_style_text_color(lblVs_, lv_color_hex(0x88FF44), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblVs_, &lv_font_montserrat_24, LV_PART_MAIN);
  lv_obj_set_pos(lblVs_, 10, 148);

  // Max altitude — subtle white
  lblMaxAlt_ = lv_label_create(area);
  lv_label_set_text(lblMaxAlt_, "MAX ---.- m");
  lv_obj_set_style_text_color(lblMaxAlt_, lv_color_hex(0xCCCCCC), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblMaxAlt_, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_set_pos(lblMaxAlt_, 10, 200);
}

void MainScreen::createBatteryStrip(lv_obj_t* parent) {
  batStrip_ = lv_obj_create(parent);
  lv_obj_set_pos(batStrip_, 0, kBatY);
  lv_obj_set_size(batStrip_, kW, kBatH);
  lv_obj_set_style_bg_color(batStrip_, lv_color_hex(0x1E1E1E), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(batStrip_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(batStrip_, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(batStrip_, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(batStrip_, 0, LV_PART_MAIN);
  lv_obj_clear_flag(batStrip_, LV_OBJ_FLAG_SCROLLABLE);

  lblTxBat_ = lv_label_create(batStrip_);
  lv_label_set_text(lblTxBat_, "TX --.-V");
  lv_obj_set_style_text_color(lblTxBat_, lv_color_hex(0xAAAAFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblTxBat_, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(lblTxBat_, 6, 5);

  lblCompBat_ = lv_label_create(batStrip_);
  lv_label_set_text(lblCompBat_, "DISP --.-V");
  lv_obj_set_style_text_color(lblCompBat_, lv_color_hex(0xAAAAFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblCompBat_, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(lblCompBat_, 110, 5);

  lblCmdStatus_ = lv_label_create(batStrip_);
  lv_label_set_text(lblCmdStatus_, "");
  lv_obj_set_style_text_color(lblCmdStatus_, lv_color_hex(0x88FF44), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblCmdStatus_, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(lblCmdStatus_, 240, 5);
  lv_obj_set_width(lblCmdStatus_, 230);
  lv_label_set_long_mode(lblCmdStatus_, LV_LABEL_LONG_CLIP);
}

void MainScreen::createAlertStrip(lv_obj_t* parent) {
  alertStrip_ = lv_obj_create(parent);
  lv_obj_set_pos(alertStrip_, 0, kAlertY);
  lv_obj_set_size(alertStrip_, kW, kAlertH);
  lv_obj_set_style_bg_color(alertStrip_, lv_color_hex(0x3A0000), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(alertStrip_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(alertStrip_, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(alertStrip_, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(alertStrip_, 0, LV_PART_MAIN);
  lv_obj_clear_flag(alertStrip_, LV_OBJ_FLAG_SCROLLABLE);

  lblAlert_ = lv_label_create(alertStrip_);
  lv_label_set_text(lblAlert_, "Nominal");
  lv_obj_set_style_text_color(lblAlert_, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblAlert_, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_pos(lblAlert_, 6, 4);
  lv_obj_set_width(lblAlert_, kW - 12);
  lv_label_set_long_mode(lblAlert_, LV_LABEL_LONG_CLIP);
}

void MainScreen::createActionsPanel(lv_obj_t* parent) {
  // Toggle tab — always visible at right edge
  btnActionsToggle_ = lv_btn_create(parent);
  lv_obj_set_pos(btnActionsToggle_, kToggleX, kToggleY);
  lv_obj_set_size(btnActionsToggle_, kToggleW, kToggleH);
  lv_obj_set_style_bg_color(btnActionsToggle_, lv_color_hex(0x444444), LV_PART_MAIN);
  lv_obj_set_style_bg_color(btnActionsToggle_, lv_color_hex(0x666666),
                             LV_PART_MAIN | LV_STATE_PRESSED);
  lv_obj_set_style_radius(btnActionsToggle_, 4, LV_PART_MAIN);
  lv_obj_set_style_border_width(btnActionsToggle_, 0, LV_PART_MAIN);
  lv_obj_add_event_cb(btnActionsToggle_, onActionsToggle, LV_EVENT_CLICKED, nullptr);
  lblActToggle_ = lv_label_create(btnActionsToggle_);
  lv_label_set_text(lblActToggle_, ">");
  lv_obj_set_style_text_color(lblActToggle_, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblActToggle_, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_center(lblActToggle_);

  // Actions panel — hidden by default, overlays right portion of main area
  actionsPanel_ = lv_obj_create(parent);
  lv_obj_set_pos(actionsPanel_, kActPanelX, kMainY);
  lv_obj_set_size(actionsPanel_, kActPanelW, kMainH);
  lv_obj_set_style_bg_color(actionsPanel_, lv_color_hex(0x1A1A2E), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(actionsPanel_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_color(actionsPanel_, lv_color_hex(0x444466), LV_PART_MAIN);
  lv_obj_set_style_border_width(actionsPanel_, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(actionsPanel_, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(actionsPanel_, 8, LV_PART_MAIN);
  lv_obj_clear_flag(actionsPanel_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(actionsPanel_, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t* panelTitle = lv_label_create(actionsPanel_);
  lv_label_set_text(panelTitle, "ACTIONS");
  lv_obj_set_style_text_color(panelTitle, lv_color_hex(0xCCCCCC), LV_PART_MAIN);
  lv_obj_set_style_text_font(panelTitle, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_pos(panelTitle, 0, 2);

  int btnW = kActPanelW - 18;   // panel width minus 2×pad
  int btnH = 44;

  btnBuzz1_ = makeBtn(actionsPanel_, "BUZZ 1s",
                      0, 28, btnW, btnH,
                      lv_color_hex(0xFF6600), onBuzz1);

  btnBuzz5_ = makeBtn(actionsPanel_, "BUZZ 5s",
                      0, 80, btnW, btnH,
                      lv_color_hex(0xFF6600), onBuzz5);

  btnSd_ = makeBtn(actionsPanel_, "SD LOG",
                   0, 132, btnW, btnH,
                   lv_color_hex(0x008888), onSdToggle);

  lblSdState_ = lv_label_create(actionsPanel_);
  lv_label_set_text(lblSdState_, "OFF");
  lv_obj_set_style_text_color(lblSdState_, lv_color_hex(0xAAAAAA), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblSdState_, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(lblSdState_, btnW / 2 - 8, 182);
}

void MainScreen::createSettingsOverlay(lv_obj_t* parent) {
  settingsOverlay_ = lv_obj_create(parent);
  lv_obj_set_pos(settingsOverlay_, 0, 0);
  lv_obj_set_size(settingsOverlay_, kW, kH);
  lv_obj_set_style_bg_color(settingsOverlay_, lv_color_hex(0x0D0D1A), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(settingsOverlay_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(settingsOverlay_, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(settingsOverlay_, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(settingsOverlay_, 16, LV_PART_MAIN);
  lv_obj_clear_flag(settingsOverlay_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(settingsOverlay_, LV_OBJ_FLAG_HIDDEN);

  // Title
  lv_obj_t* title = lv_label_create(settingsOverlay_);
  lv_label_set_text(title, "SETTINGS");
  lv_obj_set_style_text_color(title, lv_color_hex(0x00FFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_20, LV_PART_MAIN);
  lv_obj_set_pos(title, 0, 0);

  // Close button (X) in top-right
  lv_obj_t* btnClose = lv_btn_create(settingsOverlay_);
  lv_obj_set_pos(btnClose, kW - 60, 0);
  lv_obj_set_size(btnClose, 44, 30);
  lv_obj_set_style_bg_color(btnClose, lv_color_hex(0x882222), LV_PART_MAIN);
  lv_obj_set_style_radius(btnClose, 4, LV_PART_MAIN);
  lv_obj_set_style_border_width(btnClose, 0, LV_PART_MAIN);
  lv_obj_add_event_cb(btnClose, onSettingsClose, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* closeLbl = lv_label_create(btnClose);
  lv_label_set_text(closeLbl, LV_SYMBOL_CLOSE);
  lv_obj_set_style_text_color(closeLbl, lv_color_white(), LV_PART_MAIN);
  lv_obj_center(closeLbl);

  int y = 46;

  // Brightness section
  lv_obj_t* lblBrTitle = lv_label_create(settingsOverlay_);
  lv_label_set_text(lblBrTitle, "Backlight:");
  lv_obj_set_style_text_color(lblBrTitle, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblBrTitle, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_set_pos(lblBrTitle, 0, y);

  lblBrightnessVal_ = lv_label_create(settingsOverlay_);
  lv_label_set_text(lblBrightnessVal_, "100%");
  lv_obj_set_style_text_color(lblBrightnessVal_, lv_color_hex(0xFFCC00), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblBrightnessVal_, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_set_pos(lblBrightnessVal_, 140, y);

  y += 26;
  sliderBrightness_ = lv_slider_create(settingsOverlay_);
  lv_obj_set_pos(sliderBrightness_, 0, y);
  lv_obj_set_size(sliderBrightness_, kW - 48, 20);
  lv_slider_set_range(sliderBrightness_, 10, 255);
  lv_slider_set_value(sliderBrightness_, 255, LV_ANIM_OFF);
  lv_obj_add_event_cb(sliderBrightness_, onBrightness, LV_EVENT_VALUE_CHANGED, nullptr);

  y += 42;

  // Touch calibration section
  lv_obj_t* calTitle = lv_label_create(settingsOverlay_);
  lv_label_set_text(calTitle, "Touch Calibration:");
  lv_obj_set_style_text_color(calTitle, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(calTitle, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_set_pos(calTitle, 0, y);

  y += 26;
  lv_obj_t* btnCal = lv_btn_create(settingsOverlay_);
  lv_obj_set_pos(btnCal, 0, y);
  lv_obj_set_size(btnCal, 190, 36);
  lv_obj_set_style_bg_color(btnCal, lv_color_hex(0x335599), LV_PART_MAIN);
  lv_obj_set_style_radius(btnCal, 6, LV_PART_MAIN);
  lv_obj_set_style_border_width(btnCal, 0, LV_PART_MAIN);
  lv_obj_add_event_cb(btnCal, onCalibrate, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* calBtnLbl = lv_label_create(btnCal);
  lv_label_set_text(calBtnLbl, "Run 4-Point Cal");
  lv_obj_set_style_text_color(calBtnLbl, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(calBtnLbl, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_center(calBtnLbl);

  lblCalStatus_ = lv_label_create(settingsOverlay_);
  lv_label_set_text(lblCalStatus_, "Tap button to run touch calibration.");
  lv_obj_set_style_text_color(lblCalStatus_, lv_color_hex(0xAAAAAA), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblCalStatus_, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(lblCalStatus_, 0, y + 42);

  y += 92;

  // Info / about
  char infoBuf[80];
  snprintf(infoBuf, sizeof(infoBuf), "Mode: %s   Baud: %d",
           COMPANION_LINK_UART ? "UART" : "WiFi",
           static_cast<int>(UART_BAUD));
  lv_obj_t* lblInfo = lv_label_create(settingsOverlay_);
  lv_label_set_text(lblInfo, infoBuf);
  lv_obj_set_style_text_color(lblInfo, lv_color_hex(0x888888), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblInfo, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(lblInfo, 0, y);

  y += 20;
  lv_obj_t* lblVer = lv_label_create(settingsOverlay_);
  lv_label_set_text(lblVer, "ESP32 Companion v2.0  (LVGL 8.3)");
  lv_obj_set_style_text_color(lblVer, lv_color_hex(0x555555), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblVer, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(lblVer, 0, y);
}

// ── Panel visibility helpers ──────────────────────────────────────────────

void MainScreen::showActions(bool visible) {
  actionsVisible_ = visible;
  if (visible) {
    lv_obj_clear_flag(actionsPanel_, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(lblActToggle_, "<");
  } else {
    lv_obj_add_flag(actionsPanel_, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(lblActToggle_, ">");
  }
}

void MainScreen::showSettings(bool visible) {
  settingsVisible_ = visible;
  if (visible) {
    lv_obj_clear_flag(settingsOverlay_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(settingsOverlay_);
  } else {
    lv_obj_add_flag(settingsOverlay_, LV_OBJ_FLAG_HIDDEN);
  }
}

// ── update() ─────────────────────────────────────────────────────────────

void MainScreen::update(const CompanionState& state) {
  char buf[64];

  // Header: link status colour
  lv_color_t linkColor;
  if (state.stale) {
    linkColor = lv_color_hex(0xFF8800);  // orange = stale
  } else if (state.link.connected) {
    linkColor = lv_color_hex(0x00CC44);  // green = connected
  } else {
    linkColor = lv_color_hex(0xFF2222);  // red = no link
  }
  lv_obj_set_style_text_color(lblLinkDot_, linkColor, LV_PART_MAIN);

  snprintf(buf, sizeof(buf), "RSSI %d", state.link.rssi);
  lv_label_set_text(lblRssi_, buf);

  snprintf(buf, sizeof(buf), "SNR %.1f", static_cast<double>(state.link.snr));
  lv_label_set_text(lblSnr_, buf);

  if (state.link.lastPacketAgeMs >= 0) {
    snprintf(buf, sizeof(buf), "AGE %dms", state.link.lastPacketAgeMs);
  } else {
    snprintf(buf, sizeof(buf), "AGE ---ms");
  }
  lv_label_set_text(lblAge_, buf);

  snprintf(buf, sizeof(buf), "PKT %lu",
           static_cast<unsigned long>(state.flight.packetCount));
  lv_label_set_text(lblPkt_, buf);

  // Phase
  if (state.flight.phase.length()) {
    snprintf(buf, sizeof(buf), "PHASE: %s", state.flight.phase.c_str());
  } else {
    snprintf(buf, sizeof(buf), "PHASE: ---");
  }
  lv_label_set_text(lblPhase_, buf);

  // Altitude AGL
  if (isnan(state.alt.altitudeAglM)) {
    lv_label_set_text(lblAlt_, "ALT ---.-- m");
  } else {
    if (state.alt.altitudeAglM > maxAltAgl_) {
      maxAltAgl_ = state.alt.altitudeAglM;
    }
    snprintf(buf, sizeof(buf), "ALT %.1f m",
             static_cast<double>(state.alt.altitudeAglM));
    lv_label_set_text(lblAlt_, buf);
  }

  // Vertical speed
  if (isnan(state.alt.verticalSpeedMps)) {
    lv_label_set_text(lblVs_, "VS  ---.- m/s");
  } else {
    snprintf(buf, sizeof(buf), "VS  %+.1f m/s",
             static_cast<double>(state.alt.verticalSpeedMps));
    lv_label_set_text(lblVs_, buf);
  }

  // Max altitude
  if (maxAltAgl_ > 0.0f) {
    snprintf(buf, sizeof(buf), "MAX %.1f m", static_cast<double>(maxAltAgl_));
  } else {
    snprintf(buf, sizeof(buf), "MAX ---.- m");
  }
  lv_label_set_text(lblMaxAlt_, buf);

  // Battery voltages
  if (isnan(state.battery.telemetryVbatV)) {
    lv_label_set_text(lblTxBat_, "TX --.-V");
  } else {
    snprintf(buf, sizeof(buf), "TX %.2fV",
             static_cast<double>(state.battery.telemetryVbatV));
    lv_label_set_text(lblTxBat_, buf);
  }

  if (isnan(state.battery.companionVbatV)) {
    lv_label_set_text(lblCompBat_, "DISP --.-V");
  } else {
    snprintf(buf, sizeof(buf), "DISP %.2fV",
             static_cast<double>(state.battery.companionVbatV));
    lv_label_set_text(lblCompBat_, buf);
  }

  // Command status: auto-clear after 3 s
  if (cmdMsg_.length() && (millis() - cmdTs_ > 3000)) {
    cmdMsg_ = "";
    lv_label_set_text(lblCmdStatus_, "");
  }

  // Alert strip: show latest alert or link state
  if (alertCount_ == 0) {
    lv_label_set_text(lblAlert_,
                      state.stale ? "LINK STALE" : "Nominal");
  }
  // (pushAlert() already sets the label directly when a new alert arrives)
}

// ── pushAlert / setCommandStatus ─────────────────────────────────────────

void MainScreen::pushAlert(const String& msg) {
  if (msg.length() == 0) return;
  for (int i = 3; i > 0; --i) alerts_[i] = alerts_[i - 1];
  alerts_[0] = msg;
  if (alertCount_ < 4) alertCount_++;
  lv_label_set_text(lblAlert_, msg.c_str());
}

void MainScreen::setCommandStatus(const String& msg, bool ok) {
  cmdMsg_ = msg;
  cmdOk_  = ok;
  cmdTs_  = millis();
  lv_color_t color = ok ? lv_color_hex(0x88FF44) : lv_color_hex(0xFF4444);
  lv_obj_set_style_text_color(lblCmdStatus_, color, LV_PART_MAIN);
  lv_label_set_text(lblCmdStatus_, msg.c_str());
}

// ── Button callbacks ──────────────────────────────────────────────────────

void MainScreen::onBuzz1(lv_event_t* /*e*/) {
  if (!instance_ || !instance_->cmdCallback_) return;
  bool ok = instance_->cmdCallback_("buzzer", 1);
  instance_->setCommandStatus(ok ? "BUZZ 1s sent" : "BUZZ 1s fail", ok);
}

void MainScreen::onBuzz5(lv_event_t* /*e*/) {
  if (!instance_ || !instance_->cmdCallback_) return;
  bool ok = instance_->cmdCallback_("buzzer", 5);
  instance_->setCommandStatus(ok ? "BUZZ 5s sent" : "BUZZ 5s fail", ok);
}

void MainScreen::onSdToggle(lv_event_t* /*e*/) {
  if (!instance_ || !instance_->cmdCallback_) return;
  const char* cmd = instance_->sdLoggingEnabled_ ? "sd_stop" : "sd_start";
  bool ok = instance_->cmdCallback_(cmd, 0);
  if (ok) {
    instance_->sdLoggingEnabled_ = !instance_->sdLoggingEnabled_;
    lv_label_set_text(instance_->lblSdState_,
                      instance_->sdLoggingEnabled_ ? "REC" : "OFF");
    lv_color_t sdColor = instance_->sdLoggingEnabled_
                           ? lv_color_hex(0xFF4444)
                           : lv_color_hex(0x008888);
    lv_obj_set_style_bg_color(instance_->btnSd_, sdColor, LV_PART_MAIN);
  }
  const char* statusMsg = ok
    ? (instance_->sdLoggingEnabled_ ? "SD start sent" : "SD stop sent")
    : "SD cmd failed";
  instance_->setCommandStatus(statusMsg, ok);
}

void MainScreen::onActionsToggle(lv_event_t* /*e*/) {
  if (!instance_) return;
  instance_->showActions(!instance_->actionsVisible_);
}

void MainScreen::onSettingsOpen(lv_event_t* /*e*/) {
  if (!instance_) return;
  instance_->showSettings(true);
}

void MainScreen::onSettingsClose(lv_event_t* /*e*/) {
  if (!instance_) return;
  instance_->showSettings(false);
}

void MainScreen::onBrightness(lv_event_t* e) {
  lv_obj_t* slider = lv_event_get_target(e);
  int val = static_cast<int>(lv_slider_get_value(slider));
  analogWrite(27, val);
  if (instance_ && instance_->lblBrightnessVal_) {
    char buf[10];
    int pct = (val * 100) / 255;
    snprintf(buf, sizeof(buf), "%d%%", pct);
    lv_label_set_text(instance_->lblBrightnessVal_, buf);
  }
}

void MainScreen::onCalibrate(lv_event_t* /*e*/) {
  if (!instance_ || !tftInstance_) return;

  lv_label_set_text(instance_->lblCalStatus_, "Calibrating — tap the targets...");
  // Flush LVGL before blocking
  lv_timer_handler();

  // TFT_eSPI 4-point calibration routine (blocks until all points tapped)
  uint16_t calData[5];
  tftInstance_->calibrateTouch(calData, TFT_WHITE, TFT_BLACK, 15);

  // Persist to NVS under "touch_cal" namespace
  Preferences prefs;
  prefs.begin("touch_cal", /*readOnly=*/false);
  prefs.putBytes("cal", calData, sizeof(calData));
  prefs.end();

  tftInstance_->setTouch(calData);

  lv_label_set_text(instance_->lblCalStatus_, "Calibration saved.");
  lv_obj_invalidate(lv_scr_act());
}
