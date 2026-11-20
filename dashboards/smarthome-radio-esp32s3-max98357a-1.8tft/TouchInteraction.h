#ifndef GTS_RADIO_TOUCH_INTERACTION_H
#define GTS_RADIO_TOUCH_INTERACTION_H

#include <stdint.h>

namespace RadioTouch {
enum class Type : uint8_t { Down, Tap, SwipeUp, SwipeDown, Cancel, Move, Release };
struct Event {
  Type type;
  int16_t x, y;
  Event(Type kind = Type::Cancel, int16_t horizontal = 0, int16_t vertical = 0)
      : type(kind), x(horizontal), y(vertical) {}
};

// Feed calibrated screen coordinates at a regular, nonblocking cadence. A
// contact emits one Down, its subsequent Move samples and one release event.
// Moves let the UI retain a button-sized target rather than an 8-pixel circle.
// Holding never emits another Down or a release action until the finger lifts.
class Gesture {
 public:
  static constexpr uint32_t DebounceMs = 20;
  static constexpr int TapSlop = 8, SwipeDistance = 24;

  void reset() {
    contact_ = down_ = releasing_ = blocked_ = false;
    startedAt_ = releasedAt_ = 0;
    firstX_ = firstY_ = lastX_ = lastY_ = 0;
    farthestSquared_ = 0;
  }
  // Suppress an interrupted contact until the finger has been released.
  void cancel() { if (contact_) blocked_ = true; }

  bool sample(bool pressed, int16_t x, int16_t y, uint32_t now, Event& out) {
    if (pressed) {
      if (!contact_) {
        reset(); contact_ = true; startedAt_ = now;
        firstX_ = lastX_ = x; firstY_ = lastY_ = y;
      } else {
        lastX_ = x; lastY_ = y;
        const int32_t dx = int32_t(x) - firstX_, dy = int32_t(y) - firstY_;
        const uint64_t distance = uint64_t(int64_t(dx) * dx + int64_t(dy) * dy);
        if (distance > farthestSquared_) farthestSquared_ = distance;
      }
      releasing_ = false;
      if (!down_ && !blocked_ && uint32_t(now - startedAt_) >= DebounceMs) {
        // Initial pressure can settle after the first raw contact. Anchor the
        // debounced gesture at its current point, not that transient sample.
        firstX_ = lastX_ = x; firstY_ = lastY_ = y;
        farthestSquared_ = 0;
        down_ = true; out = Event(Type::Down, x, y); return true;
      }
      if (down_ && !blocked_) { out = Event(Type::Move, x, y); return true; }
      return false;
    }
    if (!contact_) return false;
    if (!down_ && !blocked_) { reset(); return false; }
    if (!releasing_) { releasing_ = true; releasedAt_ = now; return false; }
    if (uint32_t(now - releasedAt_) < DebounceMs) return false;

    Type type = Type::Cancel;
    int16_t eventX = lastX_, eventY = lastY_;
    if (!blocked_ && down_) {
      const int32_t dx = int32_t(lastX_) - firstX_, dy = int32_t(lastY_) - firstY_;
      const int32_t horizontal = dx < 0 ? -dx : dx, vertical = dy < 0 ? -dy : dy;
      if (vertical >= SwipeDistance && vertical > horizontal) {
        type = dy < 0 ? Type::SwipeUp : Type::SwipeDown;
        eventX = firstX_; eventY = firstY_;
      } else type = farthestSquared_ <= uint64_t(TapSlop * TapSlop) ? Type::Tap : Type::Release;
    }
    reset(); out = Event(type, eventX, eventY); return true;
  }

 private:
  bool contact_ = false, down_ = false, releasing_ = false, blocked_ = false;
  uint32_t startedAt_ = 0, releasedAt_ = 0;
  int16_t firstX_ = 0, firstY_ = 0, lastX_ = 0, lastY_ = 0;
  uint64_t farthestSquared_ = 0;
};
} // namespace RadioTouch

#endif // GTS_RADIO_TOUCH_INTERACTION_H
