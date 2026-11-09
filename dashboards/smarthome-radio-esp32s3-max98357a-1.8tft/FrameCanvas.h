#pragma once

#include <Adafruit_GFX.h>
#include <esp_heap_caps.h>
#include "DisplayProfile.h"

namespace RadioDisplay {
// A shield frame is 150 KiB. Keep it out of the internal RAM used by the
// network/audio tasks, and allocate it only after ESP32 startup enables PSRAM.
// The compact canvas retains the original Adafruit allocation and dimensions.
class FrameCanvas : public GFXcanvas16 {
 public:
  FrameCanvas(uint16_t width, uint16_t height)
      : GFXcanvas16(width, height, !LargeDisplay), bytes_(size_t(width) * height * sizeof(uint16_t)) {}
  ~FrameCanvas() {
    if (LargeDisplay && buffer) {
      heap_caps_free(buffer);
      buffer = nullptr;
    }
  }
  FrameCanvas(const FrameCanvas&) = delete;
  FrameCanvas& operator=(const FrameCanvas&) = delete;
  bool allocate() {
    if (buffer) return true;
    if (!LargeDisplay) return false;
    buffer = static_cast<uint16_t*>(heap_caps_calloc(1, bytes_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    return buffer != nullptr;
  }
 private:
  const size_t bytes_;
};
} // namespace RadioDisplay
