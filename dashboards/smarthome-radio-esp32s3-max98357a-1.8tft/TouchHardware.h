#ifndef GTS_RADIO_TOUCH_HARDWARE_H
#define GTS_RADIO_TOUCH_HARDWARE_H

#include "TouchConfig.h"
#include "TouchCalibration.h"

#if RADIO_TOUCH_ENABLED
#if RADIO_TOUCH_BACKEND == RADIO_TOUCH_RESISTIVE
#include "TouchResistive.h"
#else
#include "TouchXpt2046.h"
#endif
#else
namespace RadioTouch {
class TouchHardware {
 public:
  bool begin() { return false; }
  bool read(RawPoint&, bool&) { return false; }
  uint32_t signature() const { return 0; }
  const char* error() const { return "Touch disabled for this display"; }
};
}
#endif

#endif // GTS_RADIO_TOUCH_HARDWARE_H
