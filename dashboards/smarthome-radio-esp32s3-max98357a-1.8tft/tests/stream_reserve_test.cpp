#include "../scripts/audioi2s_v4/RadioStreamReserve.h"

#include <assert.h>
#include <stdint.h>
#include <iostream>

using radio_audio::RadioStreamReserve;

namespace {
const uint32_t floorBytes = 18000;
const uint32_t capacity = 655350;

bool tick(RadioStreamReserve& gate, uint32_t now, uint32_t filled,
          uint32_t bitrate = 256000, bool terminal = false,
          bool eligible = true) {
  return gate.update(eligible, terminal, now, filled, capacity, floorBytes, bitrate);
}

void targets_follow_compressed_bitrate_and_buffer_limits() {
  assert(RadioStreamReserve::target(capacity, floorBytes, 32000) == 26192);
  assert(RadioStreamReserve::target(capacity, floorBytes, 128000) == 50000);
  assert(RadioStreamReserve::target(capacity, floorBytes, 192000) == 66000);
  assert(RadioStreamReserve::target(capacity, floorBytes, 256000) == 82000);
  assert(RadioStreamReserve::target(capacity, floorBytes, 320000) == 98000);
  assert(RadioStreamReserve::target(capacity, floorBytes, 0) == 50768);
  assert(RadioStreamReserve::target(capacity, floorBytes, 1) == 26192);
  assert(RadioStreamReserve::target(capacity, floorBytes, UINT32_MAX) == 149072);
  assert(RadioStreamReserve::target(20000, floorBytes, 256000) == 19999);
  assert(RadioStreamReserve::target(floorBytes + 1, floorBytes, 256000) == floorBytes);
  assert(RadioStreamReserve::target(floorBytes, floorBytes, 256000) == 0);
  assert(RadioStreamReserve::target(UINT32_MAX, UINT32_MAX - 4, UINT32_MAX) == UINT32_MAX - 1);
}

void startup_primes_above_decode_floor_without_counting_a_refill() {
  RadioStreamReserve gate;
  assert(tick(gate, 0, 100));
  assert(tick(gate, 10, floorBytes));
  assert(gate.targetBytes() == 82000); // Deliberately greater than readSpace's 65535 cap.
  assert(tick(gate, 20, 65535));
  assert(!tick(gate, 30, 82000));
  assert(gate.refillCount() == 0);
  assert(!tick(gate, 40, floorBytes)); // Do not hold every frame below the high watermark.
  assert(!tick(gate, 50, 19000));
}

void refill_has_hysteresis_and_counts_once_per_underrun() {
  RadioStreamReserve gate;
  assert(!tick(gate, 0, 82000));
  assert(tick(gate, 100, floorBytes - 1));
  assert(gate.refillCount() == 1);
  assert(tick(gate, 200, floorBytes - 2));
  assert(tick(gate, 300, 40000));
  assert(gate.refillCount() == 1);
  assert(!tick(gate, 400, 82000));
  assert(!tick(gate, 500, 50000));
  assert(tick(gate, 600, 0));
  assert(gate.refillCount() == 2);
}

void added_wait_is_bounded_only_after_original_decode_floor_is_available() {
  RadioStreamReserve gate;
  assert(tick(gate, 0, 1, 0));
  assert(tick(gate, 10000, floorBytes - 1, 0));
  assert(tick(gate, 11000, floorBytes, 0));
  assert(tick(gate, 12999, 20000, 0));
  assert(!tick(gate, 13000, 21000, 0));
  assert(gate.refillCount() == 0);

  assert(tick(gate, 14000, floorBytes - 1, 0));
  assert(tick(gate, 15000, floorBytes, 0));
  assert(tick(gate, 16000, floorBytes - 1, 0));
  assert(tick(gate, 17000, floorBytes, 0));
  assert(tick(gate, 18999, floorBytes, 0));
  assert(!tick(gate, 19000, floorBytes, 0));
}

void millis_rollover_and_zero_are_valid_clock_values() {
  RadioStreamReserve atZero;
  assert(tick(atZero, 0, floorBytes));
  assert(tick(atZero, 1999, floorBytes));
  assert(!tick(atZero, 2000, floorBytes));

  RadioStreamReserve wrapped;
  assert(tick(wrapped, UINT32_MAX - 1000, floorBytes));
  assert(tick(wrapped, 998, floorBytes));
  assert(!tick(wrapped, 999, floorBytes));
}

void terminal_and_excluded_formats_bypass_the_reserve_and_reset_reentry() {
  RadioStreamReserve gate;
  assert(tick(gate, 0, floorBytes));
  assert(!tick(gate, 1, floorBytes, 256000, true));
  assert(gate.targetBytes() == 0);
  assert(!tick(gate, 2, 1, 256000, true)); // Terminal tails are never held by this policy.
  assert(!tick(gate, 3, 1, 256000, false, false)); // HLS, files, TTS, other codecs.
  assert(tick(gate, 4, floorBytes)); // New eligibility starts priming again.
  assert(!tick(gate, 5, 82000));
  assert(tick(gate, 6, floorBytes - 1));
  assert(gate.refillCount() == 1);
  gate.reset();
  assert(gate.refillCount() == 0);
  assert(gate.targetBytes() == 0);
  assert(tick(gate, 7, floorBytes));
  assert(gate.refillCount() == 0);
}

void impossible_capacity_does_not_introduce_an_unreachable_wait() {
  RadioStreamReserve gate;
  assert(!gate.update(true, false, 0, floorBytes, floorBytes, floorBytes, 256000));
  assert(!gate.update(true, false, 1, floorBytes, floorBytes + 1, floorBytes, 256000));
  assert(!gate.update(true, false, 2, 0, capacity, 0, 256000));
  assert(gate.targetBytes() == 0);
}

struct StreamSimulation {
  uint32_t firstFrameMs;
  uint32_t missingFrames;
  uint32_t decodedFrames;
  uint32_t receivedBytes;
  uint32_t remainingBytes;
  uint32_t refillCount;
};

StreamSimulation simulate_bursty_192kbps_mp3(bool reserveEnabled) {
  // A 48 kHz MPEG1 Layer3 frame spans 24 ms and contains 576 bytes at 192 kbps.
  // The source delivers its full 24 kB/s, in 6 kB bursts every 250 ms. One burst
  // arrives 250 ms late and is compensated together with the following burst.
  const uint32_t frameBytes = 576;
  const uint32_t frameMs = 24;
  uint32_t filled = floorBytes + frameBytes;
  StreamSimulation result = {UINT32_MAX, 0, 0, filled, 0, 0};
  uint32_t nextFrameMs = 0;
  RadioStreamReserve gate;
  for (uint32_t now = 0; now <= 10000; ++now) {
    if (now && now % 250 == 0 && now != 5000) {
      const uint32_t incoming = now == 5250 ? 12000 : 6000;
      filled += incoming;
      result.receivedBytes += incoming;
      assert(filled < capacity);
    }
    const bool waiting = reserveEnabled &&
        gate.update(true, false, now, filled, capacity, floorBytes, 192000);
    if (result.firstFrameMs == UINT32_MAX) {
      if (waiting || filled < floorBytes) continue;
      result.firstFrameMs = now;
      nextFrameMs = now;
    }
    if (now < nextFrameMs) continue;
    nextFrameMs += frameMs;
    if (waiting || filled < floorBytes) {
      ++result.missingFrames;
      continue;
    }
    filled -= frameBytes;
    ++result.decodedFrames;
  }
  result.remainingBytes = filled;
  result.refillCount = gate.refillCount();
  // Every received compressed byte remains queued or is consumed exactly once.
  assert(result.receivedBytes == result.remainingBytes + result.decodedFrames * frameBytes);
  return result;
}

void startup_reserve_absorbs_network_jitter_without_dropping_compressed_bytes() {
  const StreamSimulation original = simulate_bursty_192kbps_mp3(false);
  const StreamSimulation buffered = simulate_bursty_192kbps_mp3(true);
  assert(original.firstFrameMs == 0);
  assert(original.missingFrames > 10);
  assert(buffered.firstFrameMs == RadioStreamReserve::MaxWaitMs);
  assert(buffered.missingFrames == 0);
  assert(buffered.refillCount == 0);
  assert(original.receivedBytes == buffered.receivedBytes);
}
}  // namespace

int main() {
  targets_follow_compressed_bitrate_and_buffer_limits();
  startup_primes_above_decode_floor_without_counting_a_refill();
  refill_has_hysteresis_and_counts_once_per_underrun();
  added_wait_is_bounded_only_after_original_decode_floor_is_available();
  millis_rollover_and_zero_are_valid_clock_values();
  terminal_and_excluded_formats_bypass_the_reserve_and_reset_reentry();
  impossible_capacity_does_not_introduce_an_unreachable_wait();
  startup_reserve_absorbs_network_jitter_without_dropping_compressed_bytes();
  std::cout << "stream_reserve_test: passed\n";
}
