#pragma once

#include <Arduino.h>
#include <cstdio>
#include "RadioTftSetup.h"
#include <TFT_eSPI.h>

namespace RadioDisplay {
class DisplayHardware {
 public:
  bool begin() {
    if (ready_) return true;
    // The library is compiled separately. Check its actual build settings
    // before touching hardware, including when the IDE reused a cached build.
    setup_t actual;
    panel_.getSetup(actual);
    bool configured = actual.setup_id == USER_SETUP_ID &&
        actual.tft_driver == TFT_DRIVER && actual.pin_tft_cs == TFT_CS &&
        actual.pin_tft_dc == TFT_DC && actual.pin_tft_rst == TFT_RST &&
        actual.tft_width == ScreenHeight && actual.tft_height == ScreenWidth;
#if RADIO_DISPLAY == RADIO_DISPLAY_TFT_SHIELD_24
    configured = configured && !actual.serial && actual.pin_tft_rd == TFT_RD &&
        actual.pin_tft_wr == TFT_WR && actual.pin_tft_d0 == TFT_D0 &&
        actual.pin_tft_d1 == TFT_D1 && actual.pin_tft_d2 == TFT_D2 &&
        actual.pin_tft_d3 == TFT_D3 && actual.pin_tft_d4 == TFT_D4 &&
        actual.pin_tft_d5 == TFT_D5 && actual.pin_tft_d6 == TFT_D6 && actual.pin_tft_d7 == TFT_D7;
#else
    configured = configured && actual.serial && actual.pin_tft_mosi == TFT_MOSI &&
        actual.pin_tft_miso == TFT_MISO && actual.pin_tft_clk == TFT_SCLK;
#endif
    if (!configured) {
      std::snprintf(error_, sizeof(error_), "TFT_eSPI setup mismatch; rebuild with project build_opt.h");
      return false;
    }
#if RADIO_DISPLAY == RADIO_DISPLAY_TFT_SHIELD_24
    // The wired SD card shares neither display nor audio pins. Keep it
    // deselected; this feature does not mount or otherwise use the card.
    pinMode(RADIO_SD_CS, OUTPUT); digitalWrite(RADIO_SD_CS, HIGH);
#endif
    // TFT_eSPI initializes only the configured parallel or SPI interface.
    panel_.init();
    panel_.setRotation(1);
    if (panel_.width() != ScreenWidth || panel_.height() != ScreenHeight) {
      std::snprintf(error_, sizeof(error_), "Unexpected TFT dimensions; check selected display profile");
      return false;
    }
    // GFXcanvas16 stores native RGB565 words; TFT_eSPI's RAM image path needs
    // byte swapping enabled to send these words in the display's wire order.
    panel_.setSwapBytes(true);
    panel_.setTextWrap(false);
    panel_.fillScreen(0);
    error_[0] = 0;
    ready_ = true;
    return true;
  }
  void present(const uint16_t* buffer) {
    if (!ready_ || !buffer) return;
    // TFT_eSPI's RAM overload accepts a mutable pointer, but pushImage and the
    // non-DMA pushPixels path read the source without changing the canvas.
    panel_.pushImage(0, 0, ScreenWidth, ScreenHeight, const_cast<uint16_t*>(buffer));
  }
  void showMessage(const char* title, const char* detail) {
    if (!ready_) return;
    panel_.fillScreen(0);
    panel_.setTextWrap(true);
    panel_.setTextFont(1);
    panel_.setTextSize(LargeDisplay ? 2 : 1);
    panel_.setTextColor(0xffff);
    panel_.setCursor(ContentLeft, LargeDisplay ? 18 : 8);
    panel_.println(title ? title : "");
    panel_.setTextColor(0xffe0);
    panel_.setCursor(ContentLeft, LargeDisplay ? 62 : 32);
    panel_.println(detail ? detail : "");
    panel_.setTextWrap(false);
  }
  bool ready() const { return ready_; }
  const char* error() const { return error_; }
  const char* driverName() const {
#if RADIO_DISPLAY == RADIO_DISPLAY_TFT_SHIELD_24
    return "TFT_eSPI / 2.4-inch parallel ILI9341";
#else
    return "TFT_eSPI / 1.8-inch SPI ST7735";
#endif
  }
 private:
  TFT_eSPI panel_;
  bool ready_ = false;
  char error_[80] = {};
};
} // namespace RadioDisplay
