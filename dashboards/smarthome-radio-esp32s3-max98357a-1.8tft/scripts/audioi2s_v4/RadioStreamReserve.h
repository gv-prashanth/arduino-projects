#pragma once

#include <stdint.h>

namespace radio_audio {

// Decoder-owned policy: defer compressed decoding while network intake continues.
// The caller drains queued PCM first and never resets the MP3 decoder on refill.
class RadioStreamReserve {
 public:
  static constexpr uint32_t MaxWaitMs = 2000;

  void reset() {
    initialized_ = false;
    waiting_ = false;
    clockArmed_ = false;
    waitStartedMs_ = 0;
    targetBytes_ = 0;
    refillCount_ = 0;
  }

  // Returns true when the decoder should wait. Terminal input bypasses the high
  // watermark so the library's existing end-of-input path can drain its tail.
  bool update(bool eligible, bool terminal, uint32_t nowMs,
              uint32_t filledBytes, uint32_t capacityBytes,
              uint32_t decodeFloorBytes, uint32_t bitrate) {
    if (!eligible || terminal || decodeFloorBytes == 0 ||
        capacityBytes <= decodeFloorBytes ||
        capacityBytes - decodeFloorBytes <= 1) {
      initialized_ = false;
      waiting_ = false;
      clockArmed_ = false;
      targetBytes_ = 0;
      return false;
    }

    targetBytes_ = target(capacityBytes, decodeFloorBytes, bitrate);
    if (!initialized_) {
      initialized_ = true;
      waiting_ = true;  // Prime before the first decoded frame.
      clockArmed_ = false;
    } else if (!waiting_ && filledBytes < decodeFloorBytes) {
      waiting_ = true;
      clockArmed_ = false;
      ++refillCount_;
    }

    if (!waiting_) return false;
    if (filledBytes >= targetBytes_) {
      waiting_ = false;
      clockArmed_ = false;
      return false;
    }
    if (filledBytes < decodeFloorBytes) {
      // These bytes could not have been decoded by the unpatched library either;
      // bound only the extra delay after its original decode floor is reached.
      clockArmed_ = false;
      return true;
    }
    if (!clockArmed_) {
      clockArmed_ = true;
      waitStartedMs_ = nowMs;
    }
    if (static_cast<uint32_t>(nowMs - waitStartedMs_) >= MaxWaitMs) {
      waiting_ = false;
      clockArmed_ = false;
      return false;
    }
    return true;
  }

  uint32_t targetBytes() const { return targetBytes_; }
  uint32_t refillCount() const { return refillCount_; }

  static uint32_t target(uint32_t capacityBytes, uint32_t decodeFloorBytes,
                         uint32_t bitrate) {
    if (capacityBytes <= decodeFloorBytes) return 0;
    uint64_t reserve = bitrate
        ? (static_cast<uint64_t>(bitrate) * MaxWaitMs + 7999) / 8000
        : 32U * 1024U;
    if (reserve < 8U * 1024U) reserve = 8U * 1024U;
    if (reserve > 128U * 1024U) reserve = 128U * 1024U;
    const uint32_t available = capacityBytes - decodeFloorBytes - 1;
    if (reserve > available) reserve = available;
    return decodeFloorBytes + static_cast<uint32_t>(reserve);
  }

 private:
  bool initialized_ = false;
  bool waiting_ = false;
  bool clockArmed_ = false;
  uint32_t waitStartedMs_ = 0;
  uint32_t targetBytes_ = 0;
  uint32_t refillCount_ = 0;
};

}  // namespace radio_audio
