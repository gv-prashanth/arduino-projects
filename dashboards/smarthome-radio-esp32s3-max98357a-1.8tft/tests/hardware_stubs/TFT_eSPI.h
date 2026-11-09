#pragma once
#include "Arduino.h"

#ifdef ILI9341_DRIVER
#define TFT_DRIVER 0x9341
#else
#define TFT_DRIVER 0x7735
#endif

struct setup_t {
  uint32_t setup_id = 0;
  uint16_t tft_driver = 0;
  int tft_width = 0, tft_height = 0;
  bool serial = false;
  int pin_tft_cs = -1, pin_tft_dc = -1, pin_tft_rst = -1, pin_tft_rd = -1, pin_tft_wr = -1;
  int pin_tft_d0 = -1, pin_tft_d1 = -1, pin_tft_d2 = -1, pin_tft_d3 = -1;
  int pin_tft_d4 = -1, pin_tft_d5 = -1, pin_tft_d6 = -1, pin_tft_d7 = -1;
  int pin_tft_mosi = -1, pin_tft_miso = -1, pin_tft_clk = -1;
};
class TFT_eSPI {
 public:
  TFT_eSPI() {
    auto& state = FakeHardware::state();
    if (!state.width) state.width = RadioDisplay::ScreenWidth;
    if (!state.height) state.height = RadioDisplay::ScreenHeight;
  }
  void getSetup(setup_t& result) {
    auto& state = FakeHardware::state();
    ++state.setupCalls;
    result.setup_id = state.setupMismatch ? 0 : USER_SETUP_ID;
    result.tft_driver = state.wrongController ? 0 : TFT_DRIVER;
    result.tft_width = state.wrongGeometry ? 1 : RadioDisplay::ScreenHeight;
    result.tft_height = RadioDisplay::ScreenWidth;
    result.pin_tft_cs = state.pinMismatch ? -1 : TFT_CS;
    result.pin_tft_dc = TFT_DC; result.pin_tft_rst = TFT_RST;
#if RADIO_DISPLAY == RADIO_DISPLAY_TFT_SHIELD_24
    result.pin_tft_rd = TFT_RD; result.pin_tft_wr = TFT_WR;
    result.pin_tft_d0 = TFT_D0; result.pin_tft_d1 = TFT_D1;
    result.pin_tft_d2 = TFT_D2; result.pin_tft_d3 = TFT_D3;
    result.pin_tft_d4 = TFT_D4; result.pin_tft_d5 = TFT_D5;
    result.pin_tft_d6 = TFT_D6; result.pin_tft_d7 = TFT_D7;
#else
    result.serial = true;
    result.pin_tft_mosi = TFT_MOSI; result.pin_tft_miso = TFT_MISO; result.pin_tft_clk = TFT_SCLK;
#endif
  }
  void init() {
    ++FakeHardware::state().initCalls;
#ifndef TFT_PARALLEL_8_BIT
    ++FakeHardware::state().spiCalls;
#endif
  }
  void setRotation(int rotation) { FakeHardware::state().rotation = rotation; }
  int width() { return FakeHardware::state().width; }
  int height() { return FakeHardware::state().height; }
  void setSwapBytes(bool value) { FakeHardware::state().byteSwap = value; }
  void fillScreen(uint16_t) { ++FakeHardware::state().fills; }
  void setTextWrap(bool) {}
  void setTextFont(int) {}
  void setTextSize(int) {}
  void setTextColor(uint16_t) {}
  void setCursor(int, int) {}
  void println(const char*) { ++FakeHardware::state().messages; }
  void pushImage(int, int, int width, int height, uint16_t*) {
    assert(FakeHardware::state().byteSwap);
    ++FakeHardware::state().bulkFrames;
    FakeHardware::state().frameWidth = width; FakeHardware::state().frameHeight = height;
  }
};
