#pragma once

#include "DisplayProfile.h"
#include "fonts/RadioSansBody.h"
#include "fonts/RadioSansTitle.h"
#include "fonts/RadioSansSmall.h"
#include "fonts/RadioSansChrome.h"
#include "fonts/RadioSansLargeChrome.h"
#include "fonts/RadioSansLargeRow.h"
#include "fonts/RadioSansLargeTitle.h"
#include "fonts/RadioSansLargeDetail.h"

namespace RadioFonts {
constexpr const GFXfont* Body = &RadioSansBody;
constexpr const GFXfont* Title = &RadioSansTitle;
constexpr const GFXfont* Small = &RadioSansSmall;
constexpr const GFXfont* Chrome = &RadioSansChrome;
constexpr const GFXfont* LargeChrome = &RadioSansLargeChrome;
constexpr const GFXfont* LargeRow = &RadioSansLargeRow;
constexpr const GFXfont* LargeTitle = &RadioSansLargeTitle;
constexpr const GFXfont* LargeDetail = &RadioSansLargeDetail;
// Ink bounds relative to the GFX cursor baseline, across printable ASCII.
// A row's top maps to baseline = top + Ascent; Descent includes baseline y=0.
constexpr int16_t BodyAscent = 10, BodyDescent = 4, BodyHeight = 14;
constexpr int16_t TitleAscent = 14, TitleDescent = 5, TitleHeight = 19;
constexpr int16_t SmallAscent = 9, SmallDescent = 4, SmallHeight = 13;
constexpr int16_t ChromeAscent = 7, ChromeDescent = 3, ChromeHeight = 10;
constexpr int16_t LargeChromeAscent = 12, LargeChromeDescent = 4, LargeChromeHeight = 16;
constexpr int16_t LargeRowAscent = 16, LargeRowDescent = 6, LargeRowHeight = 22;
constexpr int16_t LargeTitleAscent = 22, LargeTitleDescent = 8, LargeTitleHeight = 30;
constexpr int16_t LargeDetailAscent = 13, LargeDetailDescent = 5, LargeDetailHeight = 18;
} // namespace RadioFonts
