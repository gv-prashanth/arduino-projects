#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <gfxfont.h>

// ESP32 maps PROGMEM into its address space, so the font and bitmap pointers
// can be read directly. This renderer also works with ordinary host arrays.
namespace RadioDisplay {
namespace proportional_detail {

struct FontBounds {
  int top = 0;
  int bottom = 0;
};

inline FontBounds fontBounds(const GFXfont& font) {
  FontBounds result;
  if (!font.glyph || font.last < font.first) return result;
  const unsigned first = font.first < 32 ? 32 : font.first;
  const unsigned last = font.last > 126 ? 126 : font.last;
  bool ink = false;
  for (unsigned c = first; c <= last; ++c) {
    const GFXglyph& glyph = font.glyph[c - font.first];
    if (!glyph.width || !glyph.height) continue;
    const int bottom = static_cast<int>(glyph.yOffset) + glyph.height;
    if (!ink || glyph.yOffset < result.top) result.top = glyph.yOffset;
    if (!ink || bottom > result.bottom) result.bottom = bottom;
    ink = true;
  }
  return result;
}

inline const GFXglyph* glyphFor(const GFXfont& font, unsigned char character) {
  if (!font.glyph || font.last < font.first) return nullptr;
  if (character < 32 || character > 126 || character < font.first || character > font.last) {
    if (font.first <= '?' && font.last >= '?') character = '?';
    else if (font.first <= ' ' && font.last >= ' ') character = ' ';
    else return nullptr;
  }
  return font.glyph + character - font.first;
}

struct TextBounds {
  std::int64_t advance = 0;
  std::int64_t left = 0;
  std::int64_t right = 0;
  std::size_t length = 0;
  std::int64_t width() const { return right - left; }
  void append(const GFXglyph* glyph) {
    if (glyph) {
      if (glyph->width && glyph->height) {
        const std::int64_t begin = advance + glyph->xOffset;
        const std::int64_t end = begin + glyph->width;
        if (begin < left) left = begin;
        if (end > right) right = end;
      }
      advance += glyph->xAdvance;
      if (advance > right) right = advance;
    }
    ++length;
  }
};

inline TextBounds measure(const GFXfont& font, const char* text) {
  TextBounds result;
  if (!text) return result;
  while (*text) result.append(glyphFor(font, static_cast<unsigned char>(*text++)));
  return result;
}

template <class Canvas>
inline void drawGlyph(Canvas& canvas, const GFXfont& font, const GFXglyph* glyph,
                      std::int64_t pen, std::int64_t baseline,
                      std::int64_t left, std::int64_t top,
                      std::int64_t right, std::int64_t bottom,
                      std::uint16_t color) {
  if (!glyph || !glyph->width || !glyph->height) return;
  const std::int64_t glyphLeft = pen + glyph->xOffset;
  const std::int64_t glyphTop = baseline + glyph->yOffset;
  if (glyphLeft >= right || glyphLeft + glyph->width <= left ||
      glyphTop >= bottom || glyphTop + glyph->height <= top) return;
  for (unsigned row = 0; row < glyph->height; ++row) {
    const std::int64_t y = glyphTop + row;
    if (y < top || y >= bottom || y < std::numeric_limits<int>::min() ||
        y > std::numeric_limits<int>::max()) continue;
    for (unsigned column = 0; column < glyph->width; ++column) {
      const std::int64_t x = glyphLeft + column;
      if (x < left || x >= right || x < std::numeric_limits<int>::min() ||
          x > std::numeric_limits<int>::max()) continue;
      const unsigned bit = row * glyph->width + column;
      const std::uint8_t byte = font.bitmap[glyph->bitmapOffset + bit / 8];
      if (byte & (0x80u >> (bit % 8))) canvas.drawPixel(static_cast<int>(x), static_cast<int>(y), color);
    }
  }
}

template <class Canvas>
inline void drawRun(Canvas& canvas, const GFXfont& font, const char* text,
                    std::size_t length, bool marker, std::int64_t pen,
                    std::int64_t baseline, std::int64_t left, std::int64_t top,
                    std::int64_t right, std::int64_t bottom, std::uint16_t color) {
  for (std::size_t i = 0; i < length; ++i) {
    const GFXglyph* glyph = glyphFor(font, static_cast<unsigned char>(text[i]));
    drawGlyph(canvas, font, glyph, pen, baseline, left, top, right, bottom, color);
    if (glyph) pen += glyph->xAdvance;
  }
  if (marker) drawGlyph(canvas, font, glyphFor(font, '~'), pen, baseline, left, top, right, bottom, color);
}

}  // namespace proportional_detail

// Height is the shared ASCII ink box, including ascenders and descenders.
// All drawText calls place that box at top, regardless of their actual letters.
inline int fontHeight(const GFXfont& font) {
  const proportional_detail::FontBounds bounds = proportional_detail::fontBounds(font);
  return bounds.bottom - bounds.top;
}

// Logical advances retain spaces; extra left/right ink overhang is included.
inline int textWidth(const GFXfont& font, const char* text) {
  const std::int64_t width = proportional_detail::measure(font, text).width();
  return width > std::numeric_limits<int>::max() ? std::numeric_limits<int>::max() : static_cast<int>(width);
}

// Draws foreground pixels only. The caller clears the rectangle when repainting.
// Input has already been transliterated to ASCII; unsupported bytes use '?'.
// Returns true only when overflow actually activates the pixel marquee.
template <class Canvas>
inline bool drawText(Canvas& canvas, const GFXfont& font, int x, int top,
                     int width, const char* ascii, std::uint16_t color,
                     bool scroll, std::uint32_t now) {
  using namespace proportional_detail;
  if (!ascii || !*ascii || width <= 0 || !font.bitmap || !font.glyph) return false;
  const FontBounds vertical = fontBounds(font);
  if (vertical.bottom <= vertical.top) return false;
  const std::int64_t left = x, right = left + width;
  const std::int64_t baseline = static_cast<std::int64_t>(top) - vertical.top;
  const std::int64_t bottom = static_cast<std::int64_t>(top) + vertical.bottom - vertical.top;
  const TextBounds full = measure(font, ascii);
  if (full.width() <= width) {
    drawRun(canvas, font, ascii, full.length, false, left - full.left,
            baseline, left, top, right, bottom, color);
    return false;
  }
  if (scroll) {
    constexpr std::int64_t gap = 16;
    constexpr std::uint64_t pixelsPerSecond = 25;
    const std::int64_t period = full.width() + gap;
    const std::int64_t offset = static_cast<std::int64_t>((static_cast<std::uint64_t>(now) * pixelsPerSecond / 1000) % period);
    const std::int64_t pen = left - full.left - offset;
    drawRun(canvas, font, ascii, full.length, false, pen,
            baseline, left, top, right, bottom, color);
    drawRun(canvas, font, ascii, full.length, false, pen + period,
            baseline, left, top, right, bottom, color);
    return true;
  }
  // Reserve a proportional '~', then select the longest prefix that fits.
  // If even the marker cannot fit, clipping still shows the start of the text.
  const GFXglyph* marker = font.first <= '~' && font.last >= '~' ? glyphFor(font, '~') : nullptr;
  TextBounds marked;
  marked.append(marker);
  if (!marker || !marker->width || !marker->height || marked.width() > width) {
    drawRun(canvas, font, ascii, full.length, false, left - full.left,
            baseline, left, top, right, bottom, color);
    return false;
  }
  TextBounds prefix, selected = marked;
  std::size_t length = 0;
  for (std::size_t i = 0; i < full.length; ++i) {
    prefix.append(glyphFor(font, static_cast<unsigned char>(ascii[i])));
    marked = prefix; marked.append(marker);
    if (marked.width() > width) break;
    selected = marked; length = i + 1;
  }
  drawRun(canvas, font, ascii, length, true, left - selected.left,
          baseline, left, top, right, bottom, color);
  return false;
}

}  // namespace RadioDisplay
