#pragma once
#include <cassert>
#include <cstdint>
#include <deque>
#include <vector>
#define HIGH 1
#define LOW 0
#define OUTPUT 2
#define INPUT_PULLUP 3
#define bit(b) (1UL << (b))
namespace FakeTouch {
enum class Response { Healthy, FloatingHigh, ShortedLow, AllMaximum };
struct State {
  int levels[49] = {};
  int modes[49] = {};
  unsigned gpioCalls = 0, begins = 0, ends = 0, transactions = 0, transactionEnds = 0;
  unsigned reads = 0, failReadAt = 0, delayTotal = 0;
  unsigned spiConstructions = 0;
  bool down = false, transaction = false, failBegin = false;
  int cs = 47, clock = 41, mosi = 39, miso = 40;
  uint8_t command = 0;
  Response response = Response::Healthy;
  std::deque<uint16_t> words;
  std::vector<uint8_t> commands;
  std::vector<int> changedPins;
};
inline State& state() { static State value; return value; }
inline void reset() { state() = State{}; }
inline void checkPin(int pin) {
  auto& s = state(); assert(pin == 16 || pin == 38 || pin == s.cs);
  s.changedPins.push_back(pin); ++s.gpioCalls;
}
}
inline void pinMode(int pin, int mode) {
  FakeTouch::checkPin(pin); FakeTouch::state().modes[pin] = mode;
}
inline void digitalWrite(int pin, int level) {
  auto& s = FakeTouch::state(); FakeTouch::checkPin(pin);
  // Match Arduino-ESP32 3.x: a newly selected CS must become a GPIO first.
  if (s.modes[pin] != OUTPUT) return;
  if (pin == s.cs && level == LOW) assert(s.transaction && s.levels[16] == HIGH && s.levels[38] == HIGH);
  s.levels[pin] = level;
}
inline void delayMicroseconds(unsigned delay) { FakeTouch::state().delayTotal += delay; }
