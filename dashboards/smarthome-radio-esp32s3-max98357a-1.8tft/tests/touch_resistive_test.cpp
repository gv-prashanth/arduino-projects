#include <Arduino.h>
#include "../TouchHardware.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <type_traits>

using RadioTouch::RawPoint;
using RadioTouch::TouchHardware;

#if RADIO_TOUCH_ENABLED
static_assert(RADIO_TOUCH_BACKEND == RADIO_TOUCH_RESISTIVE, "Exercise default resistive dispatch");
static_assert(!std::is_copy_constructible<TouchHardware>::value, "ADC resources require one owner");

static void restored() {
  const auto& s = FakeResistive::state();
  for (int pin : {7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21}) {
    assert(s.pins[pin].mode == FakeResistive::Output);
    assert(!s.pins[pin].pullup);
  }
  for (int pin : {7, 15, 16, 17, 21}) assert(s.pins[pin].level == 1);
  assert(s.writeEdges == 0); // No command/data write may occur during touch sampling.
  for (int pin : s.changedPins) assert(FakeResistive::allowed(pin));
}

static void verifiedDefaultMappingAndCalibratableCoordinates() {
  FakeResistive::reset();
  {
    TouchHardware panel;
    assert(panel.begin()); restored();
    assert(panel.pins().xp == 8 && panel.pins().xm == 15 && panel.pins().yp == 16 && panel.pins().ym == 18);
    auto& s = FakeResistive::state(); s.displayActive = true;
    const unsigned gpioCalls = s.gpioCalls, configurations = s.configureCalls;
    assert(panel.begin() && s.gpioCalls == gpioCalls && s.configureCalls == configurations);
    RawPoint point{44, 55}; bool pressed = true;
    assert(panel.read(point, pressed) && !pressed && point.x == 44 && point.y == 55); restored();
    s.down = true;
    const unsigned before = s.readCalls;
    assert(panel.read(point, pressed) && pressed); restored();
    assert(point.x >= 500 && point.x <= 3600 && point.y >= 500 && point.y <= 3600);
    assert(s.readCalls - before <= 16); // Bounded median groups, no allocation or retries.
    const RawPoint first = point;
    s.positions[0] = 0.7; s.positions[1] = 0.2;
    assert(panel.read(point, pressed) && pressed); restored();
    assert(std::abs(point.x - first.x) > 500 && std::abs(point.y - first.y) > 500);
    bool adc1 = false, adc2 = false;
    for (int pin : s.sampledPins) { adc1 |= pin <= 10; adc2 |= pin > 10; }
    assert(adc1 && adc2); // Verified CS/D1 axis needs ADC2; no ADC1-only assumption.
    assert(s.newCalls[0] == 1 && s.newCalls[1] == 1);
    s.down = false;
    const RawPoint previous = point;
    assert(panel.read(point, pressed) && !pressed && point.x == previous.x && point.y == previous.y); restored();
    assert(panel.signature() != 0 && panel.signature() != 0xC63E4A7FUL);
    TouchHardware reversed(TouchHardware::Pins(15, 8, 16, 18));
    assert(reversed.signature() != panel.signature());
  }
  assert(FakeResistive::state().delCalls[0] == 1 && FakeResistive::state().delCalls[1] == 1);
}

static void alternativeWriteSharedMappingUsesAdc1WhenAvailable() {
  FakeResistive::reset();
  {
    auto& s = FakeResistive::state();
    s.plates[0] = {10, 15, 300.0}; s.plates[1] = {7, 9, 500.0};
    TouchHardware panel(TouchHardware::Pins(10, 15, 7, 9));
    assert(panel.begin()); restored(); s.displayActive = true; s.down = true;
    RawPoint point{0, 0}; bool pressed = false;
    assert(panel.read(point, pressed) && pressed); restored();
    assert(!s.sampledPins.empty());
    for (int pin : s.sampledPins) assert(pin <= 10);
    assert(s.newCalls[1] == 0);
  }
}

static void invalidConfigurationDoesNotTouchUnrelatedPins() {
  for (const auto& pins : {
      TouchHardware::Pins(-1, 15, 16, 18), TouchHardware::Pins(8, -1, 16, 18),
      TouchHardware::Pins(8, 15, -1, 18), TouchHardware::Pins(8, 15, 16, -1),
      TouchHardware::Pins(8, 15, 16, 8), TouchHardware::Pins(4, 15, 16, 18),
      TouchHardware::Pins(8, 15, 21, 18), TouchHardware::Pins(8, 15, 17, 18),
      TouchHardware::Pins(8, 15, 38, 18), TouchHardware::Pins(8, 7, 16, 18)}) {
    FakeResistive::reset();
    TouchHardware panel(pins);
    RawPoint point{45, 67}; bool pressed = true;
    assert(!panel.begin() && panel.error()[0]);
    assert(!panel.read(point, pressed) && pressed && point.x == 45 && point.y == 67);
    assert(FakeResistive::state().changedPins.empty());
  }
  for (int unit = 0; unit < 2; ++unit) {
    FakeResistive::reset();
    {
      FakeResistive::state().denyUnit[unit] = true;
      TouchHardware panel;
      assert(!panel.begin() && panel.error()[0]);
      RawPoint point{3, 4}; bool pressed = false;
      assert(!panel.read(point, pressed) && !pressed && point.x == 3 && point.y == 4);
    }
    for (int allocated = 0; allocated < 2; ++allocated)
      assert(FakeResistive::state().delCalls[allocated] <= FakeResistive::state().newCalls[allocated]);
  }
}

static unsigned successfulReadCount() {
  FakeResistive::reset();
  TouchHardware panel; assert(panel.begin());
  auto& s = FakeResistive::state(); s.down = true;
  const unsigned before = s.readCalls;
  RawPoint point{0, 0}; bool pressed = false;
  assert(panel.read(point, pressed) && pressed);
  return s.readCalls - before;
}

static void adcTimeoutsNeverFabricateReleaseAndAlwaysRestorePads() {
  const unsigned reads = successfulReadCount();
  assert(reads > 0 && reads <= 16);
  for (unsigned failed = 1; failed <= reads; ++failed) {
    FakeResistive::reset();
    TouchHardware panel; assert(panel.begin());
    auto& s = FakeResistive::state(); s.down = true; s.displayActive = true;
    s.readFailAt = s.readCalls + failed;
    RawPoint point{23, 45}; bool pressed = true;
    assert(!panel.read(point, pressed) && pressed && point.x == 23 && point.y == 45);
    assert(panel.error()[0]); restored();
    s.readFailAt = 0;
    assert(panel.read(point, pressed) && pressed); restored();
    assert(s.newCalls[0] == 1 && s.newCalls[1] == 1);
  }
}

static void failedMuxConfigurationAndInvalidConversionsAreCancelled() {
  for (unsigned failed = 1; failed <= 2; ++failed) {
    FakeResistive::reset(); TouchHardware panel; assert(panel.begin());
    auto& s = FakeResistive::state(); s.down = true; s.displayActive = true;
    s.configureFailAt = s.configureCalls + failed;
    RawPoint point{23, 45}; bool pressed = true;
    assert(!panel.read(point, pressed) && pressed && point.x == 23 && point.y == 45); restored();
  }
  for (int invalid : {-1, 4096}) {
    FakeResistive::reset(); TouchHardware panel; assert(panel.begin());
    auto& s = FakeResistive::state(); s.down = true; s.displayActive = true;
    s.samples = {1000, 1000, invalid, 1000};
    RawPoint point{23, 45}; bool pressed = true;
    assert(!panel.read(point, pressed) && pressed && point.x == 23 && point.y == 45); restored();
  }
}

static void gpioFailuresRestoreSharedPadsBeforeDisplayCanWrite() {
  unsigned calls = 0;
  FakeResistive::reset();
  {
    TouchHardware panel; assert(panel.begin());
    auto& s = FakeResistive::state(); s.down = true;
    const unsigned before = s.gpioCalls;
    RawPoint point{0, 0}; bool pressed = false;
    assert(panel.read(point, pressed) && pressed);
    calls = s.gpioCalls - before;
  }
  assert(calls > 0 && calls < 100);
  for (unsigned failed = 1; failed <= calls; ++failed) {
    FakeResistive::reset(); TouchHardware panel; assert(panel.begin());
    auto& s = FakeResistive::state(); s.down = true; s.displayActive = true;
    s.gpioFailAt = s.gpioCalls + failed;
    RawPoint point{23, 45}; bool pressed = true;
    assert(!panel.read(point, pressed) && pressed && point.x == 23 && point.y == 45);
    restored();
  }
}

static void contactMismatchNoiseAndLiftAreNotTaps() {
  for (unsigned badContact = 1; badContact <= 2; ++badContact) {
    FakeResistive::reset(); TouchHardware panel; assert(panel.begin());
    auto& s = FakeResistive::state(); s.down = true; s.displayActive = true;
    s.digitalReadFailAt = s.contactReads + badContact;
    RawPoint point{33, 44}; bool pressed = true;
    assert(!panel.read(point, pressed) && pressed && point.x == 33 && point.y == 44); restored();
  }
  FakeResistive::reset();
  {
    TouchHardware panel; assert(panel.begin());
    auto& s = FakeResistive::state(); s.down = true; s.noiseAmplitude = 400; s.displayActive = true;
    RawPoint point{33, 44}; bool pressed = true;
    assert(!panel.read(point, pressed) && pressed && point.x == 33 && point.y == 44); restored();
  }
  const unsigned reads = successfulReadCount();
  for (unsigned lift = 1; lift <= reads; ++lift) {
    FakeResistive::reset(); TouchHardware panel; assert(panel.begin());
    auto& s = FakeResistive::state(); s.down = true; s.displayActive = true; s.liftAtAdcRead = s.readCalls + lift;
    RawPoint point{33, 44}; bool pressed = true;
    const bool valid = panel.read(point, pressed);
    assert(!valid || !pressed);
    assert(point.x == 33 && point.y == 44); restored();
  }
}

int main() {
  verifiedDefaultMappingAndCalibratableCoordinates();
  alternativeWriteSharedMappingUsesAdc1WhenAvailable();
  invalidConfigurationDoesNotTouchUnrelatedPins();
  adcTimeoutsNeverFabricateReleaseAndAlwaysRestorePads();
  failedMuxConfigurationAndInvalidConversionsAreCancelled();
  gpioFailuresRestoreSharedPadsBeforeDisplayCanWrite();
  contactMismatchNoiseAndLiftAreNotTaps();
  std::cout << "Resistive touch electrical/protocol tests passed.\n";
}
#else
int main() {
  FakeResistive::reset(); TouchHardware panel;
  assert(!panel.begin());
  RawPoint point{12, 34}; bool pressed = true;
  assert(!panel.read(point, pressed) && point.x == 12 && point.y == 34 && pressed);
  assert(FakeResistive::state().changedPins.empty() && FakeResistive::state().readCalls == 0);
  assert(FakeResistive::state().newCalls[0] == 0 && FakeResistive::state().newCalls[1] == 0);
  std::cout << "Disabled resistive touch performs no hardware I/O.\n";
}
#endif
