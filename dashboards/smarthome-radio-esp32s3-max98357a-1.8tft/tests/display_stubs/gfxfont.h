#ifndef _GFXFONT_H_
#define _GFXFONT_H_

#include <cstdint>

// Match Adafruit GFX's public font layout, including mutable pointer types.
typedef struct {
  std::uint16_t bitmapOffset;
  std::uint8_t width;
  std::uint8_t height;
  std::uint8_t xAdvance;
  std::int8_t xOffset;
  std::int8_t yOffset;
} GFXglyph;

typedef struct {
  std::uint8_t* bitmap;
  GFXglyph* glyph;
  std::uint16_t first;
  std::uint16_t last;
  std::uint8_t yAdvance;
} GFXfont;

#endif
