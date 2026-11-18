#include <Arduino.h>
#include "../TouchHardware.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <type_traits>

using RadioTouch::TouchHardware;
using RadioTouch::RawPoint;

#if RADIO_TOUCH_ENABLED
static_assert(!std::is_copy_constructible<TouchHardware>::value, "ADC handle must have one owner");
static uint64_t pinMask(int pin) { return uint64_t(1) << pin; }
static void setup(TouchHardware::Pins pins = TouchHardware::Pins()) {
  FakeTouch::reset();
  auto& s = FakeTouch::state();
  s.allowed = pinMask(TFT_D0) | pinMask(TFT_D1) | pinMask(TFT_D2) | pinMask(TFT_D3) | pinMask(TFT_D4) |
      pinMask(TFT_D5) | pinMask(TFT_D6) | pinMask(TFT_D7) | pinMask(TFT_WR) | pinMask(TFT_DC) | pinMask(TFT_RD) | pinMask(TFT_CS);
  s.cs = TFT_CS; s.xp = pins.xp; s.xm = pins.xm; s.yp = pins.yp; s.ym = pins.ym;
  s.pins[TFT_CS].level = s.pins[TFT_RD].level = s.pins[TFT_WR].level = s.pins[TFT_DC].level = 1;
}
static void restored(TouchHardware::Pins pins = TouchHardware::Pins()) {
  auto& s = FakeTouch::state();
  for (const int pin : {pins.xp, pins.xm, pins.yp, pins.ym, TFT_CS, TFT_RD, TFT_WR, TFT_DC}) {
    assert(s.pins[pin].mode == FakeTouch::Output);
    assert(!s.pins[pin].pullup);
  }
  for (const int pin : {TFT_CS, TFT_RD, TFT_WR, TFT_DC}) assert(s.pins[pin].level == 1);
  assert(s.pins[TFT_RST].level == 0); // Unrelated pins never touched.
  for (const int pin : {1, 2, 4, 5, 6, 19, 20, 38, 39, 40, 41, 42})
    assert(s.pins[pin].level == 0);
}
static void contactsAndMedian() {
  setup();
  {
    TouchHardware reader;
    assert(reader.begin() && reader.begin());
    assert(FakeTouch::state().newCalls == 1);
    RawPoint point{11, 22}; bool pressed = true;
    FakeTouch::state().contacts = {false};
    assert(reader.read(point, pressed) && !pressed);
    assert(point.x == 11 && point.y == 22 && FakeTouch::state().readCalls == 0);
    restored();
    FakeTouch::state().contacts = {true, true};
    FakeTouch::state().samples = {0, 1010, 1000, 1020, 4095, 3000, 3020, 3010};
    assert(reader.read(point, pressed) && pressed);
    assert(point.x == 1010 && point.y == 3010);
    assert(FakeTouch::state().configuredPins == std::vector<int>({7, 10}));
    restored();
    assert(reader.read(point, pressed) && pressed);
    assert(FakeTouch::state().newCalls == 1 && FakeTouch::state().configureCalls == 4);
    assert(FakeTouch::state().delayTotal < 4000);
    restored();
    FakeTouch::state().contacts = {true, false};
    assert(reader.read(point, pressed) && !pressed); // Release during sampling.
    restored();
  }
  assert(FakeTouch::state().delCalls == 1);
}
static void samplesFailWithoutFabricatingRelease() {
  for (unsigned failAt = 1; failAt <= 8; ++failAt) {
    setup(); TouchHardware reader; assert(reader.begin());
    auto& s = FakeTouch::state(); s.readFailAt = failAt;
    RawPoint point{23, 24}; bool pressed = true;
    assert(!reader.read(point, pressed) && pressed && point.x == 23 && point.y == 24);
    restored();
    s.readFailAt = 0; assert(reader.read(point, pressed) && pressed);
    assert(s.newCalls == 1); restored();
  }
  for (unsigned failAt = 1; failAt <= 2; ++failAt) {
    setup(); TouchHardware reader; assert(reader.begin());
    FakeTouch::state().configureFailAt = failAt;
    RawPoint point{23, 24}; bool pressed = false;
    assert(!reader.read(point, pressed) && !pressed && point.x == 23);
    restored();
  }
  for (const auto& values : {std::deque<int>{0, 1000, 1200, 1000},
                             std::deque<int>{0, -1, 1000, 1000},
                             std::deque<int>{0, 4096, 1000, 1000}}) {
    setup(); TouchHardware reader; assert(reader.begin());
    FakeTouch::state().samples = values;
    RawPoint point{2, 3}; bool pressed = true;
    assert(!reader.read(point, pressed) && pressed && point.x == 2 && point.y == 3);
    restored();
  }
}
static void gpioFailuresRestoreDisplay() {
  for (unsigned offset = 1; offset <= 10; ++offset) {
    setup(); TouchHardware reader; assert(reader.begin());
    auto& s = FakeTouch::state(); s.gpioFailAt = s.gpioCalls + offset;
    RawPoint point{2, 3}; bool pressed = true;
    assert(!reader.read(point, pressed) && pressed && point.x == 2);
    restored();
  }
  setup();
  TouchHardware reader; assert(reader.begin());
  auto& s = FakeTouch::state();
  s.gpioFailAt = s.gpioCalls + 11; // The restore operation itself fails.
  RawPoint point{2, 3}; bool pressed = true;
  assert(!reader.read(point, pressed) && pressed && point.x == 2);
  const unsigned failedCalls = s.gpioCalls;
  assert(!reader.read(point, pressed) && s.gpioCalls == failedCalls);
  assert(std::strstr(reader.error(), "restore") != nullptr);
  s.gpioFailAt = 0;
  assert(reader.begin() && s.newCalls == 1); // Recover using the same ADC owner.
  restored();
}
static void alternateElectrodesAndInvalidMaps() {
  const TouchHardware::Pins alternate(15, 10, 12, 9);
  setup(alternate);
  {
    TouchHardware reader(alternate); assert(reader.begin());
    RawPoint point{}; bool pressed = false;
    assert(reader.read(point, pressed) && pressed);
    assert(FakeTouch::state().configuredPins == std::vector<int>({9, 10}));
    restored(alternate);
  }
  for (const auto& pins : {TouchHardware::Pins(10, 15, 7, 10), TouchHardware::Pins(16, 15, 7, 9),
                           TouchHardware::Pins(17, 15, 7, 9), TouchHardware::Pins(21, 15, 7, 9),
                           TouchHardware::Pins(4, 15, 7, 9), TouchHardware::Pins(-1, 15, 7, 9),
                           TouchHardware::Pins(15, 14, 7, 9)}) {
    setup(); TouchHardware reader(pins);
    assert(!reader.begin() && reader.error()[0]);
    assert(FakeTouch::state().newCalls == 0 && FakeTouch::state().touchedPins.empty());
  }
  setup();
  {
    FakeTouch::state().denyUnit = true;
    TouchHardware reader;
    RawPoint point{1, 2}; bool pressed = true;
    assert(!reader.begin() && !reader.read(point, pressed) && pressed && point.x == 1);
    assert(FakeTouch::state().touchedPins.empty());
  }
  assert(FakeTouch::state().delCalls == 0);
  TouchHardware a, b, c(alternate);
  assert(a.signature() && a.signature() == b.signature() && a.signature() != c.signature());
}
#endif

int main() {
#if RADIO_TOUCH_ENABLED
  contactsAndMedian(); samplesFailWithoutFabricatingRelease();
  gpioFailuresRestoreDisplay(); alternateElectrodesAndInvalidMaps();
#else
  TouchHardware reader;
  RawPoint point{1, 2}; bool pressed = true;
  assert(!reader.begin() && !reader.read(point, pressed) && pressed && point.x == 1);
  assert(reader.signature() == 0);
  assert(FakeTouch::state().newCalls == 0 && FakeTouch::state().touchedPins.empty());
#endif
  std::cout << "Touch hardware tests passed\n";
}
