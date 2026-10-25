#include "../ProportionalText.h"

#include <array>
#include <cassert>
#include <cstdio>
#include <limits>

namespace {

std::uint8_t bitmap[] = {
  0xf7, 0x80, // A: 111 / 101 / 111 (negative left bearing)
  0xec,       // B: 11 / 10 / 11
  0xdd,       // g: 11 / 01 / 11 / 01 (descender)
  0xa0,       // ~: 101
  0xd4,       // ?: 11 / 01 / 01
  0xff, 0xf0  // X: 1111 / 1111 / 1111 (right overhang)
};
std::array<GFXglyph, 95> glyphs;
GFXfont font = {bitmap, glyphs.data(), 32, 126, 8};

void initFont() {
  glyphs.fill(GFXglyph{5, 2, 3, 3, 0, -3});
  glyphs['A' - 32] = GFXglyph{0, 3, 3, 4, -1, -3};
  glyphs['B' - 32] = GFXglyph{2, 2, 3, 3, 0, -3};
  glyphs['g' - 32] = GFXglyph{3, 2, 4, 3, 0, -2};
  glyphs['~' - 32] = GFXglyph{4, 3, 1, 4, 0, -2};
  glyphs['?' - 32] = GFXglyph{5, 2, 3, 3, 0, -3};
  glyphs['X' - 32] = GFXglyph{6, 4, 3, 2, 1, -3};
  glyphs[' ' - 32] = GFXglyph{0, 0, 0, 2, 0, 0};
}

struct Canvas {
  static constexpr int Width = 32, Height = 16;
  std::array<std::uint16_t, Width * Height> pixels{};
  void drawPixel(int x, int y, std::uint16_t color) {
    assert(x >= 0 && x < Width && y >= 0 && y < Height);
    pixels[static_cast<unsigned>(y * Width + x)] = color;
  }
  std::uint16_t at(int x, int y) const { return pixels[static_cast<unsigned>(y * Width + x)]; }
  unsigned lit() const {
    unsigned count = 0;
    for (std::uint16_t pixel : pixels) if (pixel) ++count;
    return count;
  }
  void clipped(int x, int y, int width, int height) const {
    for (int py = 0; py < Height; ++py) for (int px = 0; px < Width; ++px)
      if (at(px, py)) assert(px >= x && px < x + width && py >= y && py < y + height);
  }
};

void testMetrics() {
  assert(RadioDisplay::fontHeight(font) == 5); // Ink box [-3, 2), not yAdvance=8.
  assert(RadioDisplay::textWidth(font, nullptr) == 0);
  assert(RadioDisplay::textWidth(font, "") == 0);
  assert(RadioDisplay::textWidth(font, " ") == 2);
  assert(RadioDisplay::textWidth(font, "A") == 5);
  assert(RadioDisplay::textWidth(font, "B") == 3);
  assert(RadioDisplay::textWidth(font, "g") == 3);
  assert(RadioDisplay::textWidth(font, "AB") == 8);
  assert(RadioDisplay::textWidth(font, "A B") == 10);
  assert(RadioDisplay::textWidth(font, "X") == 5);
  assert(RadioDisplay::textWidth(font, "XX") == 7);
}

void testBearingsAndDescenders() {
  Canvas a;
  assert(!RadioDisplay::drawText(a, font, 3, 2, 5, "A", 7, true, 0));
  assert(a.lit() == 8 && a.at(3, 2) == 7 && a.at(5, 4) == 7);
  assert(!a.at(4, 3));
  a.clipped(3, 2, 5, 5);
  Canvas g;
  assert(!RadioDisplay::drawText(g, font, 3, 2, 3, "g", 5, false, 0));
  assert(g.lit() == 6 && g.at(4, 6) == 5); // Descender reaches the common font bottom.
  assert(!g.at(3, 2));
  g.clipped(3, 2, 3, 5);
  Canvas x;
  assert(!RadioDisplay::drawText(x, font, 3, 2, 5, "X", 2, false, 0));
  assert(x.lit() == 12 && x.at(7, 2) == 2);
  x.clipped(3, 2, 5, 5);
}

void testShortTextNeverMoves() {
  Canvas first;
  assert(!RadioDisplay::drawText(first, font, 3, 2, 10, "A B", 1, true, 0));
  for (std::uint32_t now : {1u, 40u, 800u, 3600000u, std::numeric_limits<std::uint32_t>::max()}) {
    Canvas later;
    assert(!RadioDisplay::drawText(later, font, 3, 2, 10, "A B", 1, true, now));
    assert(later.pixels == first.pixels);
  }
  Canvas spaces;
  assert(!RadioDisplay::drawText(spaces, font, 3, 2, 4, "  ", 1, true, 1000));
  assert(!spaces.lit());
}

void testTruncation() {
  Canvas clipped, expected;
  assert(!RadioDisplay::drawText(clipped, font, 3, 2, 9, "ABgABg", 1, false, 0));
  assert(!RadioDisplay::drawText(expected, font, 3, 2, 9, "A~", 1, false, 0));
  assert(clipped.pixels == expected.pixels);
  clipped.clipped(3, 2, 9, 5);
  Canvas marker, tilde;
  assert(!RadioDisplay::drawText(marker, font, 3, 2, 4, "ABgABg", 1, false, 0));
  assert(!RadioDisplay::drawText(tilde, font, 3, 2, 4, "~", 1, false, 0));
  assert(marker.pixels == tilde.pixels);
  Canvas tiny;
  assert(!RadioDisplay::drawText(tiny, font, 3, 2, 2, "ABgABg", 1, false, 0));
  assert(tiny.lit() && tiny.at(3, 2) && tiny.at(4, 2));
  tiny.clipped(3, 2, 2, 5);
}

void testPixelMarquee() {
  Canvas initial, beforeTick, onePixel, loop;
  assert(RadioDisplay::drawText(initial, font, 5, 4, 5, "AB", 1, true, 0));
  assert(RadioDisplay::drawText(beforeTick, font, 5, 4, 5, "AB", 1, true, 39));
  assert(RadioDisplay::drawText(onePixel, font, 5, 4, 5, "AB", 1, true, 40));
  assert(RadioDisplay::drawText(loop, font, 5, 4, 5, "AB", 1, true, 960));
  assert(initial.pixels == beforeTick.pixels && initial.pixels == loop.pixels);
  assert(initial.pixels != onePixel.pixels);
  for (std::uint32_t now = 0; now < 1000; now += 13) {
    Canvas frame;
    assert(RadioDisplay::drawText(frame, font, 5, 4, 5, "AB", 1, true, now));
    frame.clipped(5, 4, 5, 5);
  }
  Canvas gap;
  assert(RadioDisplay::drawText(gap, font, 5, 4, 5, "AB", 1, true, 400));
  assert(!gap.lit()); // The 16-pixel ring gap is actually empty.
}

void testFallbackAndEmpty() {
  Canvas unsupported, question;
  const char bytes[] = {1, static_cast<char>(0x80), static_cast<char>(0xff), '\0'};
  assert(RadioDisplay::textWidth(font, bytes) == RadioDisplay::textWidth(font, "???"));
  assert(!RadioDisplay::drawText(unsupported, font, 3, 2, 12, bytes, 1, true, 70));
  assert(!RadioDisplay::drawText(question, font, 3, 2, 12, "???", 1, true, 70));
  assert(unsupported.pixels == question.pixels);
  Canvas empty;
  assert(!RadioDisplay::drawText(empty, font, 3, 2, 12, nullptr, 1, true, 0));
  assert(!RadioDisplay::drawText(empty, font, 3, 2, 12, "", 1, true, 0));
  assert(!RadioDisplay::drawText(empty, font, 3, 2, 0, "A", 1, true, 0));
  assert(!RadioDisplay::drawText(empty, font, 3, 2, -4, "A", 1, true, 0));
  GFXfont invalid = {nullptr, nullptr, 32, 126, 8};
  assert(!RadioDisplay::fontHeight(invalid) && !RadioDisplay::textWidth(invalid, "A"));
  assert(!RadioDisplay::drawText(empty, invalid, 3, 2, 12, "A", 1, true, 0));
  assert(!empty.lit());
}

}  // namespace

int main() {
  initFont(); testMetrics(); testBearingsAndDescenders(); testShortTextNeverMoves();
  testTruncation(); testPixelMarquee(); testFallbackAndEmpty();
  std::puts("Proportional text tests passed");
}
