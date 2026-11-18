#pragma once
#include <cassert>
#include <cstdint>
#include <deque>
#include <vector>

namespace FakeTouch {
enum { Floating = 0, Input = 1, Output = 2, Analog = 3 };
struct Pin { int mode = Output, level = 0; bool pullup = false; };
struct State {
  Pin pins[49];
  uint64_t allowed = 0;
  int cs = 16, xp = 10, xm = 15, yp = 7, ym = 9;
  unsigned gpioCalls = 0, gpioFailAt = 0, newCalls = 0, delCalls = 0;
  unsigned configureCalls = 0, configureFailAt = 0, readCalls = 0, readFailAt = 0;
  unsigned contactCalls = 0, delayTotal = 0;
  bool denyUnit = false;
  std::deque<bool> contacts;
  std::deque<int> samples;
  std::vector<int> configuredPins, sampledPins, touchedPins;
};
inline State& state() { static State value; return value; }
inline void reset() { state() = State{}; }
inline void checkPin(int pin) {
  assert(pin >= 0 && pin < 49);
  assert(state().allowed & (uint64_t(1) << pin));
  state().touchedPins.push_back(pin);
}
}
inline void delayMicroseconds(unsigned delay) { FakeTouch::state().delayTotal += delay; }

// Arduino.h defines this macro; production helpers must not collide with it.
#ifndef bit
#define bit(b) (1UL << (b))
#endif
