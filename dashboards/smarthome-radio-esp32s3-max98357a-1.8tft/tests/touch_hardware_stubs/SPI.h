#pragma once
#if !RADIO_TOUCH_ENABLED
#error "Disabled/compact touch must not include SPI.h"
#endif
#include <Arduino.h>
constexpr uint8_t FSPI = 1, MSBFIRST = 1, SPI_MODE0 = 0;
class SPISettings {
 public:
  SPISettings(uint32_t clock, uint8_t order, uint8_t mode) : _clock(clock), _bitOrder(order), _dataMode(mode) {}
  uint32_t _clock; uint8_t _bitOrder, _dataMode;
};
class SPIClass {
 public:
  explicit SPIClass(uint8_t bus) { assert(bus == FSPI); ++FakeTouch::state().spiConstructions; }
  ~SPIClass() { if (started_) end(); }
  void begin(int clock, int miso, int mosi, int cs) {
    auto& s = FakeTouch::state();
    assert(clock == s.clock && miso == s.miso && mosi == s.mosi && cs == s.cs);
    assert(s.levels[cs] == HIGH && s.levels[16] == HIGH && s.levels[38] == HIGH);
    ++s.begins; started_ = !s.failBegin;
  }
  void* bus() { return started_ ? this : nullptr; }
  void end() {
    assert(!FakeTouch::state().transaction); started_ = false; ++FakeTouch::state().ends;
  }
  void beginTransaction(SPISettings config) {
    auto& s = FakeTouch::state();
    assert(started_ && !s.transaction && config._clock == RADIO_TOUCH_SPI_FREQUENCY);
    assert(config._bitOrder == MSBFIRST && config._dataMode == SPI_MODE0);
    s.transaction = true; ++s.transactions;
  }
  void endTransaction() {
    auto& s = FakeTouch::state();
    assert(s.transaction && s.levels[s.cs] == HIGH && s.command == 0xD0);
    s.transaction = false; ++s.transactionEnds;
  }
  uint8_t transfer(uint8_t command) {
    auto& s = FakeTouch::state();
    assert(s.transaction && s.levels[s.cs] == LOW && s.levels[16] == HIGH && s.levels[38] == HIGH);
    assert(command == 0xB1 || command == 0xC1 || command == 0xD1 || command == 0x91 || command == 0xD0);
    s.command = command; s.commands.push_back(command); return 0;
  }
  uint16_t transfer16(uint16_t outgoing) {
    auto& s = FakeTouch::state();
    assert(s.transaction && s.levels[s.cs] == LOW && outgoing == 0);
    if (++s.reads == s.failReadAt) return 0xFFFF;
    if (!s.words.empty()) { const uint16_t word = s.words.front(); s.words.pop_front(); return word; }
    if (s.response == FakeTouch::Response::FloatingHigh) return 0xFFFF;
    if (s.response == FakeTouch::Response::ShortedLow) return 0;
    if (s.response == FakeTouch::Response::AllMaximum) return 0x7FF8;
    const unsigned value = s.command == 0xB1 ? (s.down ? 1000U : 0U) :
        s.command == 0xC1 ? (s.down ? 3000U : 4095U) : s.command == 0xD1 ? 1200U :
        s.command == 0x91 ? 2400U : 0U;
    return uint16_t(value << 3);
  }
 private:
  bool started_ = false;
};
