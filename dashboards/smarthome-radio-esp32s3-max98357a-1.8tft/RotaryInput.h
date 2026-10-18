#pragma once

#include <cstddef>
#include <cstdint>
#include <climits>

// Most mechanical encoders have four valid quadrature transitions per detent,
// with both contacts HIGH at rest. Set to 2 for encoders that also detent at LOW,
// or 1 for a detent at every transition. Define before including this header.
#ifndef RADIO_INPUT_TICKS_PER_DETENT
#define RADIO_INPUT_TICKS_PER_DETENT 4
#endif
#ifndef RADIO_INPUT_REVERSE
#define RADIO_INPUT_REVERSE 0
#endif

#ifndef RADIO_INPUT_HOST_TEST
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

namespace RadioInput {

enum class EventType : uint8_t {
  Rotate, ButtonDown, ButtonUp, ShortPress, LongPress
};

struct Event {
  EventType type;
  int16_t delta;
  uint32_t atMs;
};

// Hardware-independent model. The adapter below serializes access; host tests
// can sample GPIO states and time directly without mocking Arduino/FreeRTOS.
class InputModel {
 public:
  static constexpr uint8_t kTicksPerDetent = RADIO_INPUT_TICKS_PER_DETENT;
  static constexpr uint32_t kDebounceMs = 25;
  static constexpr uint32_t kLongPressMs = 700;
  static constexpr std::size_t kCapacity = 64;
  static constexpr std::size_t kButtonReserve = 8;

  static_assert(kTicksPerDetent == 1 || kTicksPerDetent == 2 ||
                kTicksPerDetent == 4, "ticks per detent must be 1, 2 or 4");

  void begin(bool clkHigh, bool dtHigh, bool buttonHigh, uint32_t now) {
    head_ = count_ = 0;
    overflow_ = 0;
    previousAB_ = ab(clkHigh, dtHigh);
    synchronized_ = isDetent(previousAB_);
    accumulated_ = 0;
    rawButtonHigh_ = buttonHigh;
    stableButtonHigh_ = true;
    candidateAt_ = now;
    pressedAt_ = now;
    lastActivity_ = now;
    longFired_ = false;
  }

  void sample(bool clkHigh, bool dtHigh, bool buttonHigh, uint32_t now) {
    const uint8_t currentAB = ab(clkHigh, dtHigh);
    if (currentAB != previousAB_) {
      lastActivity_ = now;
      if ((currentAB ^ previousAB_) == 3) {
        // Both bits changed: missed/invalid transition. Resynchronize at a
        // detent so the remainder of a broken cycle cannot become a click.
        accumulated_ = 0;
        synchronized_ = isDetent(currentAB);
      } else if (!synchronized_) {
        if (isDetent(currentAB)) {
          synchronized_ = true;
          accumulated_ = 0;
        }
      } else {
        // Positive follows HIGH/HIGH -> LOW/HIGH, matching CLK falling with
        // DT HIGH. Backtracking contact bounce cancels the partial movement.
        static const int8_t steps[16] = {
           0, -1,  1,  0,  1,  0,  0, -1,
          -1,  0,  0,  1,  0,  1, -1,  0
        };
        accumulated_ += steps[(previousAB_ << 2) | currentAB];
        if (isDetent(currentAB)) {
          int16_t direction = 0;
          if (accumulated_ >= kTicksPerDetent) direction = 1;
          else if (accumulated_ <= -static_cast<int>(kTicksPerDetent)) direction = -1;
          if (direction != 0) {
            if (RADIO_INPUT_REVERSE) direction = -direction;
            push({EventType::Rotate, direction, now});
          }
          accumulated_ = 0;
        }
      }
      previousAB_ = currentAB;
    }

    if (buttonHigh != rawButtonHigh_) {
      rawButtonHigh_ = buttonHigh;
      candidateAt_ = now;
      // Activity starts at the raw edge; a press near the UI inactivity
      // deadline must not time out during the debounce interval.
      lastActivity_ = now;
    }
    if (rawButtonHigh_ != stableButtonHigh_ &&
        elapsed(now, candidateAt_) >= kDebounceMs) {
      stableButtonHigh_ = rawButtonHigh_;
      if (!stableButtonHigh_) {
        pressedAt_ = now;
        longFired_ = false;
        push({EventType::ButtonDown, 0, now});
      } else {
        // Covers a delayed sample that sees release after the long threshold.
        if (!longFired_ && elapsed(now, pressedAt_) >= kLongPressMs) {
          longFired_ = true;
          push({EventType::LongPress, 0, now});
        }
        push({EventType::ButtonUp, 0, now});
        if (!longFired_) push({EventType::ShortPress, 0, now});
      }
    }
    if (buttonDown() && !longFired_ &&
        elapsed(now, pressedAt_) >= kLongPressMs) {
      longFired_ = true;
      push({EventType::LongPress, 0, now});
    }
  }

  bool poll(Event& event) {
    if (count_ == 0) return false;
    event = events_[head_];
    head_ = (head_ + 1) % kCapacity;
    --count_;
    return true;
  }

  bool buttonDown() const { return !stableButtonHigh_; }
  uint32_t pressedAt() const { return pressedAt_; }
  uint32_t lastActivity() const { return lastActivity_; }
  uint32_t overflowCount() const { return overflow_; }

 private:
  static uint32_t elapsed(uint32_t now, uint32_t since) { return now - since; }
  static uint8_t ab(bool clkHigh, bool dtHigh) {
    return static_cast<uint8_t>((clkHigh ? 2 : 0) | (dtHigh ? 1 : 0));
  }
  static bool isDetent(uint8_t state) {
    return kTicksPerDetent == 1 || state == 3 ||
           (kTicksPerDetent == 2 && state == 0);
  }
  Event& at(std::size_t offset) { return events_[(head_ + offset) % kCapacity]; }
  void noteOverflow() { if (overflow_ != UINT32_MAX) ++overflow_; }

  void push(const Event& event) {
    lastActivity_ = event.atMs;
    if (event.type == EventType::Rotate) {
      if (count_ != 0) {
        Event& tail = at(count_ - 1);
        // Preserve reversals: merging +1/-1 changes volume behavior at a
        // clamped endpoint. Never merge across a button event.
        if (tail.type == EventType::Rotate &&
            ((tail.delta > 0) == (event.delta > 0))) {
          const int32_t sum = static_cast<int32_t>(tail.delta) + event.delta;
          if (sum >= INT16_MIN && sum <= INT16_MAX) {
            tail.delta = static_cast<int16_t>(sum);
            tail.atMs = event.atMs;
            return;
          }
        }
      }
      if (count_ >= kCapacity - kButtonReserve) {
        noteOverflow();
        return;
      }
    } else if (count_ == kCapacity) {
      // Give buttons priority by removing the oldest queued rotation. Retain
      // the relative order of every remaining event, including all buttons.
      std::size_t rotation = 0;
      while (rotation < count_ && at(rotation).type != EventType::Rotate) ++rotation;
      if (rotation == count_) {
        noteOverflow();
        return;
      }
      for (std::size_t i = rotation; i + 1 < count_; ++i) at(i) = at(i + 1);
      --count_;
      noteOverflow();
    }
    at(count_++) = event;
  }

  Event events_[kCapacity]{};
  std::size_t head_ = 0;
  std::size_t count_ = 0;
  uint32_t overflow_ = 0;
  uint8_t previousAB_ = 3;
  int8_t accumulated_ = 0;
  bool synchronized_ = true;
  bool rawButtonHigh_ = true;
  bool stableButtonHigh_ = true;
  bool longFired_ = false;
  uint32_t candidateAt_ = 0;
  uint32_t pressedAt_ = 0;
  uint32_t lastActivity_ = 0;
};

#ifndef RADIO_INPUT_HOST_TEST
// Give this object static/global lifetime: its task retains the object address.
// All shared state, including snapshots, is protected by the same port mux.
class RotaryInput {
 public:
  bool begin(int clk, int dt, int sw) {
    if (task_ != nullptr || clk < 0 || dt < 0 || sw < 0 ||
        clk == dt || clk == sw || dt == sw) return false;
    clk_ = clk;
    dt_ = dt;
    sw_ = sw;
    pinMode(clk_, INPUT_PULLUP);
    pinMode(dt_, INPUT_PULLUP);
    pinMode(sw_, INPUT_PULLUP);
    model_.begin(digitalRead(clk_) != LOW, digitalRead(dt_) != LOW,
                 digitalRead(sw_) != LOW, millis());
    // The ring is embedded (no queue allocation). Check the task allocation.
    const BaseType_t result = xTaskCreate(taskEntry, "radio-input", 4096,
                                         this, 2, &task_);
    if (result != pdPASS) {
      task_ = nullptr;
      return false;
    }
    return true;
  }

  bool poll(Event& event) {
    portENTER_CRITICAL(&mux_);
    const bool available = model_.poll(event);
    portEXIT_CRITICAL(&mux_);
    return available;
  }
  bool buttonDown() {
    portENTER_CRITICAL(&mux_);
    const bool value = model_.buttonDown();
    portEXIT_CRITICAL(&mux_);
    return value;
  }
  uint32_t pressedAt() { return snapshot(&InputModel::pressedAt); }
  uint32_t lastActivity() { return snapshot(&InputModel::lastActivity); }
  uint32_t overflowCount() { return snapshot(&InputModel::overflowCount); }

  RotaryInput() = default;
  RotaryInput(const RotaryInput&) = delete;
  RotaryInput& operator=(const RotaryInput&) = delete;

 private:
  uint32_t snapshot(uint32_t (InputModel::*read)() const) {
    portENTER_CRITICAL(&mux_);
    const uint32_t value = (model_.*read)();
    portEXIT_CRITICAL(&mux_);
    return value;
  }
  static void taskEntry(void* context) {
    static_cast<RotaryInput*>(context)->run();
  }
  void run() {
    const TickType_t interval = pdMS_TO_TICKS(1) > 0 ? pdMS_TO_TICKS(1) : 1;
    for (;;) {
      const bool clkHigh = digitalRead(clk_) != LOW;
      const bool dtHigh = digitalRead(dt_) != LOW;
      const bool swHigh = digitalRead(sw_) != LOW;
      const uint32_t now = millis();
      portENTER_CRITICAL(&mux_);
      model_.sample(clkHigh, dtHigh, swHigh, now);
      portEXIT_CRITICAL(&mux_);
      vTaskDelay(interval);
    }
  }

  InputModel model_;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  TaskHandle_t task_ = nullptr;
  int clk_ = -1;
  int dt_ = -1;
  int sw_ = -1;
};
#endif

}  // namespace RadioInput
