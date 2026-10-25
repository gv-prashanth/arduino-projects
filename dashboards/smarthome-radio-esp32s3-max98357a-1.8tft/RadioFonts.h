#pragma once

#include "fonts/RadioSansBody.h"
#include "fonts/RadioSansTitle.h"
#include "fonts/RadioSansSmall.h"

namespace RadioFonts {
constexpr const GFXfont* Body = &RadioSansBody;
constexpr const GFXfont* Title = &RadioSansTitle;
constexpr const GFXfont* Small = &RadioSansSmall;
// Ink bounds relative to the GFX cursor baseline, across printable ASCII.
// A row's top maps to baseline = top + Ascent; Descent includes baseline y=0.
constexpr int16_t BodyAscent = 10, BodyDescent = 4, BodyHeight = 14;
constexpr int16_t TitleAscent = 14, TitleDescent = 5, TitleHeight = 19;
constexpr int16_t SmallAscent = 9, SmallDescent = 4, SmallHeight = 13;
} // namespace RadioFonts
