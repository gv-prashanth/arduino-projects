#ifndef GTS_RADIO_TOUCH_CALIBRATION_H
#define GTS_RADIO_TOUCH_CALIBRATION_H

#include <stdint.h>
#include <limits.h>

namespace RadioTouch {
struct RawPoint { int32_t x, y; };
struct ScreenPoint { int16_t x, y; };

// Fixed-width, value-only data can be persisted independently of radio settings.
// The signature identifies the selected reader/wiring so a hardware change
// cannot silently reuse another panel's calibration. All coordinates are for
// the display's current landscape rotation.
struct Calibration {
  uint32_t magic = 0, signature = 0, width = 0, height = 0;
  int32_t xx = 0, xy = 0, x0 = 0, yx = 0, yy = 0, y0 = 0;
  uint32_t checksum = 0;

  uint32_t digest() const {
    const uint32_t words[] = {magic, signature, width, height, uint32_t(xx), uint32_t(xy),
        uint32_t(x0), uint32_t(yx), uint32_t(yy), uint32_t(y0)};
    uint32_t value = 2166136261UL;
    for (unsigned index = 0; index < sizeof(words) / sizeof(words[0]); ++index)
      for (unsigned shift = 0; shift < 32; shift += 8) {
        value ^= (words[index] >> shift) & 255U;
        value *= 16777619UL;
      }
    return value;
  }
  bool valid(uint32_t expectedSignature, uint32_t expectedWidth, uint32_t expectedHeight) const {
    return magic == 0x47545331UL && signature == expectedSignature &&
        width == expectedWidth && height == expectedHeight && width >= 96 && height >= 96 &&
        width <= 1024 && height <= 1024 && checksum == digest();
  }
  bool map(const RawPoint& raw, ScreenPoint& point) const {
    if (!valid(signature, width, height) || raw.x < 0 || raw.x > 4095 || raw.y < 0 || raw.y > 4095)
      return false;
    const int64_t mappedX = (int64_t(xx) * raw.x + int64_t(xy) * raw.y + x0) / 65536;
    const int64_t mappedY = (int64_t(yx) * raw.x + int64_t(yy) * raw.y + y0) / 65536;
    // Allow a small calibration margin at the panel edge; reject extrapolated
    // noise instead of turning it into an edge button press.
    if (mappedX < -12 || mappedY < -12 || mappedX >= int64_t(width) + 12 ||
        mappedY >= int64_t(height) + 12) return false;
    point.x = int16_t(mappedX < 0 ? 0 : mappedX >= width ? width - 1 : mappedX);
    point.y = int16_t(mappedY < 0 ? 0 : mappedY >= height ? height - 1 : mappedY);
    return true;
  }
};

inline ScreenPoint calibrationTarget(unsigned index, int width, int height) {
  const int margin = 24;
  const int vertical = height >= 160 ? 48 : 24; // Leave branded header/footer clear.
  const ScreenPoint targets[] = {{int16_t(margin), int16_t(vertical)},
      {int16_t(width - 1 - margin), int16_t(vertical)},
      {int16_t(margin), int16_t(height - 1 - vertical)},
      {int16_t(width - 1 - margin), int16_t(height - 1 - vertical)},
      {int16_t((width - 1) / 2), int16_t((height - 1) / 2)}};
  return targets[index < 5 ? index : 4];
}

// Fit three corners, then verify the fourth corner and centre independently.
// This handles exchanged/reversed axes without assuming the panel wiring or
// folding malformed calibration data into a valid-looking stored record.
inline bool fitCalibration(const RawPoint (&raw)[5], uint32_t signature,
                           int width, int height, Calibration& result) {
  if (width < 96 || height < 96 || width > 1024 || height > 1024) return false;
  for (unsigned index = 0; index < 5; ++index)
    if (raw[index].x < 0 || raw[index].x > 4095 || raw[index].y < 0 || raw[index].y > 4095) return false;
  const int64_t ax = raw[1].x - raw[0].x, ay = raw[1].y - raw[0].y;
  const int64_t bx = raw[2].x - raw[0].x, by = raw[2].y - raw[0].y;
  const int64_t determinant = ax * by - ay * bx;
  if (determinant > -65536 && determinant < 65536) return false;
  const ScreenPoint p0 = calibrationTarget(0, width, height);
  const ScreenPoint p1 = calibrationTarget(1, width, height);
  const ScreenPoint p2 = calibrationTarget(2, width, height);
  const int64_t xx = int64_t(p1.x - p0.x) * by * 65536 / determinant;
  const int64_t xy = -int64_t(p1.x - p0.x) * bx * 65536 / determinant;
  const int64_t yx = -int64_t(p2.y - p0.y) * ay * 65536 / determinant;
  const int64_t yy = int64_t(p2.y - p0.y) * ax * 65536 / determinant;
  const int64_t x0 = int64_t(p0.x) * 65536 - xx * raw[0].x - xy * raw[0].y;
  const int64_t y0 = int64_t(p0.y) * 65536 - yx * raw[0].x - yy * raw[0].y;
  const int64_t coefficients[] = {xx, xy, x0, yx, yy, y0};
  for (unsigned index = 0; index < 6; ++index)
    if (coefficients[index] < INT32_MIN || coefficients[index] > INT32_MAX) return false;
  Calibration candidate;
  candidate.magic = 0x47545331UL; candidate.signature = signature;
  candidate.width = uint32_t(width); candidate.height = uint32_t(height);
  candidate.xx = int32_t(xx); candidate.xy = int32_t(xy); candidate.x0 = int32_t(x0);
  candidate.yx = int32_t(yx); candidate.yy = int32_t(yy); candidate.y0 = int32_t(y0);
  candidate.checksum = candidate.digest();
  for (unsigned index = 0; index < 5; ++index) {
    ScreenPoint projected;
    if (!candidate.map(raw[index], projected)) return false;
    const ScreenPoint expected = calibrationTarget(index, width, height);
    const int dx = projected.x - expected.x, dy = projected.y - expected.y;
    if (dx * dx + dy * dy > 12 * 12) return false;
  }
  result = candidate;
  return true;
}
} // namespace RadioTouch

#endif // GTS_RADIO_TOUCH_CALIBRATION_H
