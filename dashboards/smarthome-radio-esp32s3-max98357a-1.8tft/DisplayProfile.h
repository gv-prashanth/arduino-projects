#pragma once

// Select the attached display here before uploading. The parallel 2.4-inch
// shield is the default; choose RADIO_DISPLAY_ST7735_18 for the original TFT.
// A build flag such as -DRADIO_DISPLAY=2 overrides this selection for testing.
#define RADIO_DISPLAY_TFT_SHIELD_24 1
#define RADIO_DISPLAY_ST7735_18 2
#ifndef RADIO_DISPLAY
#define RADIO_DISPLAY RADIO_DISPLAY_TFT_SHIELD_24
#endif

#if RADIO_DISPLAY != RADIO_DISPLAY_TFT_SHIELD_24 && RADIO_DISPLAY != RADIO_DISPLAY_ST7735_18
#error "Unsupported RADIO_DISPLAY: select RADIO_DISPLAY_TFT_SHIELD_24 or RADIO_DISPLAY_ST7735_18"
#endif

#ifdef __cplusplus
namespace RadioDisplay {
constexpr bool LargeDisplay = RADIO_DISPLAY == RADIO_DISPLAY_TFT_SHIELD_24;
constexpr int ScreenWidth = LargeDisplay ? 320 : 160;
constexpr int ScreenHeight = LargeDisplay ? 240 : 128;
constexpr int HeaderHeight = LargeDisplay ? 32 : 18;
constexpr int ContentLeft = LargeDisplay ? 12 : 4;
constexpr int ContentWidth = ScreenWidth - ContentLeft * 2;
constexpr int FilterTop = LargeDisplay ? 41 : 21;
constexpr int FilterFocusTop = LargeDisplay ? 36 : 19;
constexpr int FilterHeight = LargeDisplay ? 26 : 14;
constexpr int FirstStationTop = LargeDisplay ? 74 : 34;
constexpr int StationRowHeight = LargeDisplay ? 26 : 15;
constexpr int VisibleStationRows = 5;
constexpr int FooterTop = LargeDisplay ? 212 : 110;
constexpr int FooterLabelTop = LargeDisplay ? 220 : 114;
constexpr int FooterControlTop = LargeDisplay ? 216 : 112;
constexpr int FooterControlHeight = LargeDisplay ? 22 : 14;
constexpr int PlayerTitleTop = LargeDisplay ? 77 : 39;
constexpr int PlayerStatusTop = LargeDisplay ? 158 : 80;
constexpr int PlayerMessageTop = LargeDisplay ? 187 : 97;
} // namespace RadioDisplay
#endif
