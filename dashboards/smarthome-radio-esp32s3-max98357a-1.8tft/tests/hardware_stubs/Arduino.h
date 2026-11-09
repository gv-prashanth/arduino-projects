#pragma once
#include <cassert>
#include <cstdint>
#include <cstdio>

#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0

namespace FakeHardware {
struct State {
  int modes[49] = {}, levels[49] = {}, rotation = -1;
  int spiCalls = 0, initCalls = 0;
  int bulkFrames = 0, fills = 0, messages = 0;
  int frameWidth = 0, frameHeight = 0, readBits = 0;
  int width = 0, height = 0, setupCalls = 0;
  bool setupMismatch = false, pinMismatch = false, wrongController = false, wrongGeometry = false, byteSwap = false;
};
inline State& state() { static State value; return value; }
inline void reset() { state() = State{}; }
}
inline void pinMode(int pin, int mode) { FakeHardware::state().modes[pin] = mode; }
inline void digitalWrite(int pin, int level) { FakeHardware::state().levels[pin] = level; }
inline int digitalRead(int) {
  ++FakeHardware::state().readBits;
  return HIGH;
}
inline void delay(unsigned) {}
inline void delayMicroseconds(unsigned) {}
struct SerialStub { template<class... T> void printf(const char*, T...) {} };
static SerialStub Serial;
