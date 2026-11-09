#include <cassert>
#include <cstring>
#include <cstdio>
#include "../DisplayHardware.h"

int main() {
  using namespace RadioDisplay;
  FakeHardware::reset();
  DisplayHardware display;
  uint16_t pixel = 0xabcd;
  display.present(&pixel); display.showMessage("Before", "begin");
  assert(FakeHardware::state().fills == 0);
  assert(display.begin() && display.ready() && !display.error()[0]);
  assert(display.begin()); // Repeated setup cannot reinitialize a live bus.
  auto& state = FakeHardware::state();
  assert(state.setupCalls == 1 && state.initCalls == 1 && state.rotation == 1 && state.byteSwap);
#if RADIO_DISPLAY == RADIO_DISPLAY_TFT_SHIELD_24
  const int pins[] = {TFT_DC, TFT_CS, TFT_WR, TFT_RD, TFT_D0, TFT_D1, TFT_D2, TFT_D3, TFT_D4, TFT_D5, TFT_D6, TFT_D7};
  const int expected[] = {15, 16, 7, 21, 8, 18, 14, 13, 12, 11, 10, 9};
  for (unsigned index = 0; index < 12; ++index) assert(pins[index] == expected[index]);
  assert(state.spiCalls == 0 && TFT_RST == 17);
  assert(state.modes[38] == OUTPUT && state.levels[38] == HIGH);
  assert(RADIO_SD_MOSI == 39 && RADIO_SD_MISO == 40 && RADIO_SD_SCLK == 41);
#else
  assert(state.spiCalls == 1 && TFT_CS == 10 && TFT_DC == 13 && TFT_RST == 14);
  assert(TFT_SCLK == 12 && TFT_MOSI == 11 && TFT_MISO == -1);
  assert(SPI_FREQUENCY == 32000000);
  assert(TFT_RGB_ORDER == 1);
#endif
  assert(state.readBits == 0);
  display.present(&pixel);
  display.present(nullptr);
  assert(state.frameWidth == ScreenWidth && state.frameHeight == ScreenHeight);
  assert(pixel == 0xabcd);
  assert(state.bulkFrames == 1);
  display.showMessage("Title", "Detail");
  assert(state.messages == 2 && std::strlen(display.driverName()) > 0);

  FakeHardware::reset(); FakeHardware::state().setupMismatch = true;
  DisplayHardware stale;
  assert(!stale.begin() && !stale.ready() && std::strstr(stale.error(), "setup mismatch"));
  assert(FakeHardware::state().initCalls == 0 && FakeHardware::state().readBits == 0);
  FakeHardware::reset(); FakeHardware::state().pinMismatch = true;
  DisplayHardware wrongPins;
  assert(!wrongPins.begin() && FakeHardware::state().initCalls == 0);
  FakeHardware::reset(); FakeHardware::state().wrongController = true;
  DisplayHardware wrongController;
  assert(!wrongController.begin() && FakeHardware::state().initCalls == 0);
  FakeHardware::reset(); FakeHardware::state().wrongGeometry = true;
  DisplayHardware wrongGeometry;
  assert(!wrongGeometry.begin() && FakeHardware::state().initCalls == 0);
  FakeHardware::reset(); FakeHardware::state().width = ScreenHeight;
  DisplayHardware dimensions;
  assert(!dimensions.begin() && std::strstr(dimensions.error(), "dimensions"));
  std::puts("Display hardware adapter tests passed");
}
