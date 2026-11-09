#include "../DisplayProfile.h"

#include <cassert>
#include <cstdio>

int main() {
  // This test is deliberately compiled without a profile override: the new
  // shield must remain the default shipped firmware configuration.
  assert(RADIO_DISPLAY == RADIO_DISPLAY_TFT_SHIELD_24);
  assert(RadioDisplay::LargeDisplay);
  assert(RadioDisplay::ScreenWidth == 320 && RadioDisplay::ScreenHeight == 240);
  assert(RadioDisplay::FirstStationTop + RadioDisplay::VisibleStationRows *
         RadioDisplay::StationRowHeight <= RadioDisplay::FooterTop);
  assert(RadioDisplay::FooterControlTop + RadioDisplay::FooterControlHeight <=
         RadioDisplay::ScreenHeight);
  std::puts("Default shield display profile tests passed");
}
