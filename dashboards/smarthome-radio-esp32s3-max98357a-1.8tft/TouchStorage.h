#ifndef GTS_RADIO_TOUCH_STORAGE_H
#define GTS_RADIO_TOUCH_STORAGE_H

#include "TouchConfig.h"
#include "TouchCalibration.h"
#if RADIO_TOUCH_ENABLED
#include <Preferences.h>
#endif

namespace RadioTouch {
class CalibrationStore {
 public:
  bool load(uint32_t signature, unsigned width, unsigned height, Calibration& calibration) {
#if RADIO_TOUCH_ENABLED
    Preferences preferences;
    if (!preferences.begin("gts-touch", true)) return false;
    Calibration candidate;
    const bool read = preferences.getBytesLength("cal") == sizeof(candidate) &&
        preferences.getBytes("cal", &candidate, sizeof(candidate)) == sizeof(candidate);
    preferences.end();
    if (read && candidate.valid(signature, width, height)) { calibration = candidate; return true; }
#else
    (void)signature; (void)width; (void)height; (void)calibration;
#endif
    return false;
  }
  bool save(const Calibration& calibration) {
#if RADIO_TOUCH_ENABLED
    if (!calibration.valid(calibration.signature, calibration.width, calibration.height)) return false;
    Preferences preferences;
    if (!preferences.begin("gts-touch", false)) return false;
    const bool written = preferences.putBytes("cal", &calibration, sizeof(calibration)) == sizeof(calibration);
    preferences.end();
    return written;
#else
    (void)calibration;
    return false;
#endif
  }
};
} // namespace RadioTouch

#endif // GTS_RADIO_TOUCH_STORAGE_H
