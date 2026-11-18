#include "../TouchCalibration.h"
#include <cassert>
#include <cstring>
#include <type_traits>
#include <cstdio>

using namespace RadioTouch;
static_assert(std::is_trivially_copyable<Calibration>::value, "Calibration storage must contain values only");

int main() {
  const unsigned signature = 0x12345678;
  for (unsigned swapped = 0; swapped < 2; ++swapped)
    for (unsigned reverseX = 0; reverseX < 2; ++reverseX)
      for (unsigned reverseY = 0; reverseY < 2; ++reverseY) {
        RawPoint samples[5];
        for (unsigned index = 0; index < 5; ++index) {
          const ScreenPoint target = calibrationTarget(index, 320, 240);
          int x = 300 + target.x * 10, y = 400 + target.y * 13;
          if (reverseX) x = 4095 - x;
          if (reverseY) y = 4095 - y;
          samples[index] = swapped ? RawPoint{y, x} : RawPoint{x, y};
        }
        Calibration fitted;
        assert(fitCalibration(samples, signature, 320, 240, fitted));
        assert(fitted.valid(signature, 320, 240));
        assert(!fitted.valid(signature + 1, 320, 240));
        assert(!fitted.valid(signature, 160, 128));
        for (unsigned index = 0; index < 5; ++index) {
          ScreenPoint projected;
          assert(fitted.map(samples[index], projected));
          const ScreenPoint target = calibrationTarget(index, 320, 240);
          assert(projected.x >= target.x - 1 && projected.x <= target.x + 1);
          assert(projected.y >= target.y - 1 && projected.y <= target.y + 1);
        }
        ScreenPoint point;
        assert(!fitted.map(RawPoint{-1, 100}, point));
        assert(!fitted.map(RawPoint{100, 4096}, point));
        Calibration restored;
        std::memcpy(&restored, &fitted, sizeof(restored));
        assert(restored.valid(signature, 320, 240));
        ++restored.xx;
        assert(!restored.valid(signature, 320, 240));
        assert(!restored.map(samples[0], point));
      }
  RawPoint samples[5] = {{500, 500}, {3500, 500}, {500, 3500}, {3500, 3500}, {1995, 1995}};
  Calibration accepted;
  assert(fitCalibration(samples, signature, 320, 240, accepted));
  const Calibration original = accepted;
  samples[3] = RawPoint{500, 500}; // A wrong fourth corner cannot be accepted.
  assert(!fitCalibration(samples, signature, 320, 240, accepted));
  assert(accepted.checksum == original.checksum);
  samples[3] = RawPoint{3500, 3500}; samples[4] = RawPoint{3000, 1000};
  assert(!fitCalibration(samples, signature, 320, 240, accepted));
  samples[0] = samples[1] = samples[2] = RawPoint{2000, 2000};
  assert(!fitCalibration(samples, signature, 320, 240, accepted));
  assert(!fitCalibration(samples, signature, 0, 240, accepted));
  Calibration empty;
  assert(!empty.valid(signature, 320, 240));
  std::puts("Touch calibration tests passed");
}
