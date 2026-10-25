#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include "RadioFonts.h"
#include "RadioModel.h"
#include "ProportionalText.h"

// Shared 160 x 128 chrome. All coordinates are ink-box tops, not GFX baselines.
// Keeping primitives here makes the host pixel checks exercise device drawing.
namespace RadioDisplay {
constexpr uint16_t Black = 0x0000, White = 0xf7bf, Green = 0x7f75;
constexpr uint16_t Yellow = 0xfeeF, Red = 0xfcd2, Cyan = 0x76fd;
constexpr uint16_t MutedInk = 0xadf8, DisabledInk = 0x63af;
constexpr uint16_t HeaderBackground = 0x1926, FocusBackground = 0x19e6, RuleColor = 0x3229;
constexpr int ScreenWidth = 160, ScreenHeight = 128, HeaderHeight = 18;
constexpr int FilterTop = 21, FirstStationTop = 34, StationRowHeight = 15;
constexpr int FooterTop = 110, FooterLabelTop = 114;
constexpr const GFXfont* ChromeFont = RadioFonts::Chrome;
constexpr const GFXfont* RowFont = RadioFonts::Small;

enum class Icon : uint8_t {
  Play, Muted, Volume, Spinner, Warning, Offline, Back, Prev, Next, Stations, Settings,
  Check, Filters, Wifi, Retry, Source, Location, Genre, Codec, Bitrate,
  ChevronDown, ChevronUpDown
};

inline const char* compactLocationLabel(const char* location) {
  if (!location || !*location) return "All";
  const auto equals = [location](const char* value) {
    const char* left = location;
    while (*left && *value) {
      const char a = *left >= 'A' && *left <= 'Z' ? static_cast<char>(*left + 'a' - 'A') : *left;
      const char b = *value >= 'A' && *value <= 'Z' ? static_cast<char>(*value + 'a' - 'A') : *value;
      if (a != b) return false;
      ++left; ++value;
    }
    return *left == *value;
  };
  if (equals("NATIONAL")) return "National";
  if (equals("Andhra Pradesh")) return "AP";
  if (equals("Telangana")) return "TS";
  if (equals("India")) return "IN";
  if (equals("United Kingdom")) return "UK";
  if (equals("United States")) return "US";
  return location;
}

template<class Canvas>
inline void drawIcon(Canvas& canvas, Icon icon, int x, int y, int size = 11,
                     uint16_t color = Cyan, uint32_t now = 0) {
  if (size <= 0) return;
  const auto px = [=](int value) { return x + (value * (size - 1) + 6) / 12; };
  const auto py = [=](int value) { return y + (value * (size - 1) + 6) / 12; };
  const auto line = [&](int ax, int ay, int bx, int by) {
    canvas.drawLine(px(ax), py(ay), px(bx), py(by), color);
  };
  switch (icon) {
    case Icon::Play:
      canvas.fillTriangle(px(2), py(1), px(2), py(11), px(11), py(6), color); break;
    case Icon::Muted:
      canvas.fillRect(px(0), py(4), px(3) - px(0) + 1, py(8) - py(4) + 1, color);
      canvas.fillTriangle(px(3), py(4), px(6), py(1), px(6), py(11), color);
      line(8, 4, 12, 8); line(8, 8, 12, 4); break;
    case Icon::Volume:
      canvas.fillRect(px(0), py(4), px(3) - px(0) + 1, py(8) - py(4) + 1, color);
      canvas.fillTriangle(px(3), py(4), px(6), py(1), px(6), py(11), color);
      line(8, 4, 9, 6); line(9, 6, 8, 8);
      line(10, 2, 12, 6); line(12, 6, 10, 10); break;
    case Icon::Spinner: {
      static constexpr int8_t points[8][2] = {{0,-6},{4,-4},{6,0},{4,4},{0,6},{-4,4},{-6,0},{-4,-4}};
      const unsigned active = (now / 100) % 8;
      for (unsigned index = 0; index < 8; ++index) {
        const int dx = points[index][0], dy = points[index][1];
        canvas.drawLine(px(6 + dx / 2), py(6 + dy / 2), px(6 + dx), py(6 + dy),
                        index == active ? color : DisabledInk);
      }
      break;
    }
    case Icon::Warning:
      line(6,0,0,11); line(0,11,12,11); line(12,11,6,0);
      line(6,4,6,7); line(6,9,6,9); break;
    case Icon::Offline:
      line(1,3,4,1); line(4,1,8,1); line(8,1,11,3);
      line(3,6,5,4); line(5,4,8,5); line(6,9,6,10); line(0,0,12,12); break;
    case Icon::Back:
      line(0,6,12,6); line(0,6,5,1); line(0,6,5,11); break;
    case Icon::Prev:
      line(8,1,3,6); line(3,6,8,11); break;
    case Icon::Next:
      line(4,1,9,6); line(9,6,4,11); break;
    case Icon::Check:
      line(1,6,4,10); line(4,10,11,2); break;
    case Icon::Stations:
      line(0,2,8,2); line(0,6,8,6); line(0,10,5,10); line(10,3,10,11); line(8,11,10,11); break;
    case Icon::Filters:
    case Icon::Bitrate:
      line(0,2,12,2); line(0,6,12,6); line(0,10,12,10);
      line(3,0,3,4); line(9,4,9,8); line(5,8,5,12); break;
    case Icon::Settings:
      line(3,1,9,1); line(9,1,12,6); line(12,6,9,11); line(9,11,3,11); line(3,11,0,6); line(0,6,3,1);
      line(4,4,8,4); line(8,4,8,8); line(8,8,4,8); line(4,8,4,4); break;
    case Icon::Wifi:
      line(0,3,3,1); line(3,1,9,1); line(9,1,12,3);
      line(2,6,5,4); line(5,4,7,4); line(7,4,10,6); line(4,8,6,7); line(6,7,8,8); line(6,10,6,11); break;
    case Icon::Retry:
      line(2,4,3,1); line(3,1,9,1); line(9,1,12,5); line(12,5,10,10); line(10,10,4,11); line(4,11,1,8);
      line(0,1,2,4); line(2,4,5,3); break;
    case Icon::Source:
      line(1,4,11,4); line(11,4,11,11); line(11,11,1,11); line(1,11,1,4);
      line(2,3,9,0); line(4,7,4,8); line(7,7,9,7); line(7,9,9,9); break;
    case Icon::Location:
      line(2,2,5,0); line(5,0,8,0); line(8,0,10,2); line(10,2,10,6); line(10,6,6,12);
      line(6,12,2,6); line(2,6,2,2); line(5,3,7,3); line(7,3,7,5); line(7,5,5,5); line(5,5,5,3); break;
    case Icon::Genre:
      line(0,0,6,0); line(6,0,12,6); line(12,6,6,12); line(6,12,0,6); line(0,6,0,0); line(3,3,3,3); break;
    case Icon::Codec:
      line(0,5,0,7); line(3,2,3,10); line(6,0,6,12); line(9,3,9,9); line(12,5,12,7); break;
    case Icon::ChevronDown:
      line(1,4,6,9); line(6,9,11,4); break;
    case Icon::ChevronUpDown:
      line(2,4,6,0); line(6,0,10,4); line(2,8,6,12); line(6,12,10,8); break;
  }
}

template<class Canvas>
inline void drawMuteIcon(Canvas& canvas, int x, int y, uint16_t color) {
  drawIcon(canvas, Icon::Muted, x, y, 12, color);
}

namespace chrome_detail {
template<class Canvas>
struct FittedCaption {
  Canvas& canvas;
  int x, width, logicalWidth;
  void drawPixel(int px, int py, uint16_t color) {
    canvas.drawPixel(x + (px - x) * width / logicalWidth, py, color);
  }
};
} // namespace chrome_detail

template<class Canvas>
inline void brandHeader(Canvas& canvas, const RadioModel::Settings& settings, const char* context = "") {
  canvas.fillRect(0, 0, ScreenWidth, HeaderHeight, HeaderBackground);
  drawText(canvas, *ChromeFont, 4, 4, 61, "GTS RADIO", White, false, 0);
  // Compact captions use a stable 40px slot before the separate mute badge.
  if (context && !std::strcmp(context, "Max bitrate")) context = "Bitrate";
  const int captionWidth = textWidth(*ChromeFont, context);
  if (captionWidth > 40) {
    chrome_detail::FittedCaption<Canvas> fitted{canvas, 67, 40, captionWidth};
    drawText(fitted, *ChromeFont, 67, 4, captionWidth, context, MutedInk, false, 0);
  } else drawText(canvas, *ChromeFont, 67, 4, 40, context, MutedInk, false, 0);
  const bool silent = settings.muted || settings.volume == 0;
  if (silent) drawIcon(canvas, Icon::Muted, 109, 3, 10, Yellow);
  char volume[12];
  std::snprintf(volume, sizeof(volume), "Vol %u", settings.volume);
  drawText(canvas, *ChromeFont, 121, 4, 36, volume, silent ? Yellow : White, false, 0);
}

template<class Canvas>
inline void focusRow(Canvas& canvas, int y, int height = StationRowHeight) {
  canvas.fillRect(3, y, 154, height, FocusBackground);
  canvas.fillRect(3, y, 2, height, Green);
}

template<class Canvas>
inline void footer(Canvas& canvas, Icon leftIcon, const char* leftLabel,
                   Icon rightIcon, const char* rightLabel) {
  canvas.drawFastHLine(4, FooterTop, 152, RuleColor);
  drawIcon(canvas, leftIcon, 5, 113, 11, Cyan);
  drawIcon(canvas, rightIcon, 83, 113, 11, Cyan);
  char label[64];
  std::snprintf(label, sizeof(label), "Click %s", leftLabel ? leftLabel : "");
  drawText(canvas, *ChromeFont, 18, FooterLabelTop, 64, label, Cyan, false, 0);
  std::snprintf(label, sizeof(label), "Hold %s", rightLabel ? rightLabel : "");
  drawText(canvas, *ChromeFont, 96, FooterLabelTop, 63, label, Cyan, false, 0);
}

template<class Canvas>
inline void navButton(Canvas& canvas, bool right, Icon symbol, const char* label,
                      bool focused, bool disabled) {
  const int x = right ? 110 : 4;
  if (focused && !disabled) canvas.fillRect(x, 112, 46, 14, FocusBackground);
  const uint16_t color = disabled ? DisabledInk : focused ? Green : White;
  drawIcon(canvas, symbol, right ? x + 33 : x + 2, 114, 10, color);
  drawText(canvas, *ChromeFont, right ? x + 3 : x + 15, 114, 29, label, color, false, 0);
}
} // namespace RadioDisplay
