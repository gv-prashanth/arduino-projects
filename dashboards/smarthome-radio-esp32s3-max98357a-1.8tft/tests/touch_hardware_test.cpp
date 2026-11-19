#include <Arduino.h>
#include "../TouchHardware.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <type_traits>

using RadioTouch::TouchHardware;
using RadioTouch::RawPoint;

#if RADIO_TOUCH_ENABLED
static_assert(!std::is_copy_constructible<TouchHardware>::value, "SPI bus must have one owner");
static TouchHardware::Pins knownPins() { return TouchHardware::Pins(47, 41, 39, 40); }
static uint16_t wire(unsigned raw) { return uint16_t(raw << 3); }
static void finished() {
  const auto& s = FakeTouch::state();
  assert(!s.transaction && s.transactions == s.transactionEnds);
  assert(s.levels[16] == HIGH && s.levels[38] == HIGH && s.levels[47] == HIGH);
  assert(!s.commands.empty() && s.commands.back() == 0xD0);
  for (int pin : s.changedPins) assert(pin == 16 || pin == 38 || pin == 47);
}
static void pressureWords(std::deque<uint16_t>& words, unsigned z1, unsigned z2) {
  words.push_back(0); words.push_back(wire(z1));
  words.push_back(0); words.push_back(wire(z2));
}
static void sampleWords(unsigned z1After = 1000, unsigned z2After = 3000, bool unstable = false) {
  auto& words = FakeTouch::state().words;
  words.clear(); pressureWords(words, 1000, 3000);
  const unsigned xs[] = {1010, 1000, unstable ? 1400U : 1020U};
  const unsigned ys[] = {3010, 3000, 3020};
  for (unsigned i = 0; i < 3; ++i) {
    words.push_back(0); words.push_back(wire(xs[i]));
    words.push_back(wire(4095)); words.push_back(wire(ys[i]));
  }
  pressureWords(words, z1After, z2After); words.push_back(0); // Final power-down conversion.
}
static void defaultConfigurationAndInvalidPins() {
  FakeTouch::reset();
  {
    TouchHardware reader;
#if RADIO_TOUCH_XPT2046_CS < 0 || RADIO_TOUCH_SPI_SCLK < 0 || RADIO_TOUCH_SPI_MOSI < 0 || RADIO_TOUCH_SPI_MISO < 0
    assert(!reader.begin() && std::strstr(reader.error(), "Configure"));
    assert(FakeTouch::state().gpioCalls == 0 && FakeTouch::state().begins == 0);
#else
    assert(reader.begin()); finished();
#endif
  }
  for (const auto& pins : {
      TouchHardware::Pins(-1, 41, 39, 40), TouchHardware::Pins(47, -1, 39, 40),
      TouchHardware::Pins(47, 41, -1, 40), TouchHardware::Pins(47, 41, 39, -1),
      TouchHardware::Pins(16, 41, 39, 40), TouchHardware::Pins(38, 41, 39, 40),
      TouchHardware::Pins(47, 41, 39, 41), TouchHardware::Pins(47, 41, 39, 40, 47),
      TouchHardware::Pins(47, 3, 39, 40), TouchHardware::Pins(47, 41, 4, 40),
      TouchHardware::Pins(47, 41, 39, 19), TouchHardware::Pins(47, 41, 39, 26),
      TouchHardware::Pins(47, 41, 39, 22), TouchHardware::Pins(48, 41, 39, 40),
      TouchHardware::Pins(43, 41, 39, 40), TouchHardware::Pins(45, 41, 39, 40)}) {
    FakeTouch::reset();
    TouchHardware reader(pins);
    RawPoint point{1, 2}; bool pressed = true;
    assert(!reader.begin() && reader.error()[0] && !reader.read(point, pressed));
    assert(point.x == 1 && point.y == 2 && pressed);
    const auto& s = FakeTouch::state();
    assert(s.gpioCalls == 0 && s.begins == 0 && s.transactions == 0);
  }
}
static void successfulPollsUseOneBusAndLeaveItIdle() {
  FakeTouch::reset();
  {
    TouchHardware reader(knownPins());
    assert(reader.begin());
    const auto count = FakeTouch::state().transactions;
    assert(reader.begin() && FakeTouch::state().transactions == count);
    auto& s = FakeTouch::state();
    const std::vector<uint8_t> expected = {0xB1, 0xB1, 0xC1, 0xC1, 0xD0};
    assert(s.commands == expected); finished();
    RawPoint point{23, 24}; bool pressed = true;
    assert(reader.read(point, pressed) && !pressed && point.x == 23 && point.y == 24);
    finished();
    s.down = true;
    const unsigned before = s.reads;
    assert(reader.read(point, pressed) && pressed && point.x == 1200 && point.y == 2400);
    assert(s.reads - before == 21 && s.begins == 1); // Bounded conversions, no reinitialization.
    finished();
    sampleWords();
    assert(reader.read(point, pressed) && pressed && point.x == 1010 && point.y == 3010);
    assert(s.words.empty()); finished();
    sampleWords(0, 4095);
    assert(reader.read(point, pressed) && !pressed && point.x == 1010 && point.y == 3010);
    finished();
    s.down = false;
    assert(reader.read(point, pressed) && !pressed);
    assert(s.begins == 1 && s.delayTotal < 2000);
  }
  assert(FakeTouch::state().ends == 1);
}
static void failedInitializationCanRetryWithoutADeadCalibration() {
  FakeTouch::reset();
  {
    auto& s = FakeTouch::state(); s.failBegin = true;
    TouchHardware reader(knownPins());
    assert(!reader.begin() && std::strstr(reader.error(), "SPI bus"));
    assert(s.transactions == 0 && s.commands.empty() && s.levels[47] == HIGH);
    s.failBegin = false;
    assert(reader.begin() && s.begins == 2); finished();
  }
  for (const auto response : {FakeTouch::Response::FloatingHigh, FakeTouch::Response::ShortedLow,
                              FakeTouch::Response::AllMaximum}) {
    FakeTouch::reset();
    auto& s = FakeTouch::state(); s.response = response;
    TouchHardware reader(knownPins());
    assert(!reader.begin() && reader.error()[0]); finished();
    RawPoint point{2, 3}; bool pressed = true;
    assert(!reader.read(point, pressed) && pressed && point.x == 2);
    s.response = FakeTouch::Response::Healthy;
    assert(reader.begin() && s.begins == 1); finished();
  }
}
static void rejectedSamplesNeverFabricateARelease() {
  for (unsigned bad = 1; bad <= 20; ++bad) {
    FakeTouch::reset(); TouchHardware reader(knownPins()); assert(reader.begin());
    auto& s = FakeTouch::state(); s.down = true; s.failReadAt = s.reads + bad;
    RawPoint point{23, 24}; bool pressed = true;
    assert(!reader.read(point, pressed) && point.x == 23 && point.y == 24 && pressed);
    assert(reader.error()[0]); finished();
    s.failReadAt = 0;
    assert(reader.read(point, pressed) && pressed);
    assert(s.begins == 1); finished();
  }
  for (const auto response : {FakeTouch::Response::FloatingHigh, FakeTouch::Response::ShortedLow,
                              FakeTouch::Response::AllMaximum}) {
    FakeTouch::reset(); TouchHardware reader(knownPins()); assert(reader.begin());
    FakeTouch::state().response = response;
    RawPoint point{9, 8}; bool pressed = false;
    assert(!reader.read(point, pressed) && !pressed && point.x == 9 && point.y == 8);
    finished();
  }
  FakeTouch::reset(); TouchHardware reader(knownPins()); assert(reader.begin());
  sampleWords(1000, 3000, true);
  RawPoint point{3, 4}; bool pressed = true;
  assert(!reader.read(point, pressed) && point.x == 3 && point.y == 4 && pressed); finished();
  FakeTouch::state().words.clear();
  // Equal/near-equal high pressure values are legal, including small ADC noise.
  std::deque<uint16_t>& words = FakeTouch::state().words;
  pressureWords(words, 2005, 2000);
  for (unsigned i = 0; i < 3; ++i) {
    words.push_back(0); words.push_back(wire(1000));
    words.push_back(0); words.push_back(wire(2000));
  }
  pressureWords(words, 2001, 2000); words.push_back(0);
  assert(reader.read(point, pressed) && pressed && point.x == 1000 && point.y == 2000); finished();
}
static void controllerAndPinChangesInvalidateOldCalibration() {
  FakeTouch::reset();
  TouchHardware a(knownPins()), b(knownPins()), changed(TouchHardware::Pins(47, 39, 41, 40));
  assert(a.signature() && a.signature() == b.signature() && a.signature() != changed.signature());
  // Exact signature emitted by the previous GPIO10/15/7/9 raw ADC backend.
  assert(a.signature() != 0xC63E4A7FUL);
}
#endif

int main() {
#if RADIO_TOUCH_ENABLED
  defaultConfigurationAndInvalidPins();
  successfulPollsUseOneBusAndLeaveItIdle();
  failedInitializationCanRetryWithoutADeadCalibration();
  rejectedSamplesNeverFabricateARelease();
  controllerAndPinChangesInvalidateOldCalibration();
#else
  FakeTouch::reset();
  TouchHardware reader;
  RawPoint point{1, 2}; bool pressed = true;
  assert(!reader.begin() && !reader.read(point, pressed) && pressed && point.x == 1 && point.y == 2);
  assert(reader.signature() == 0);
  assert(FakeTouch::state().spiConstructions == 0 && FakeTouch::state().gpioCalls == 0);
#endif
  std::cout << "XPT2046 touch hardware tests passed\n";
}
