#pragma once

#include <cstdint>
#include <cstring>
#include "RadioModel.h"

namespace RadioAudio {

enum class State : std::uint8_t { Idle, Connecting, Playing, Buffering, Retrying, Failed, Offline };

// Owner-task telemetry. All fields are values so snapshots remain safe for
// FreeRTOS byte-copy queues; no getter or logging runs in the PCM callback.
struct Diagnostics {
  std::uint32_t inputFilledBytes = 0, inputCapacityBytes = 0, minInputFilledBytes = 0;
  std::uint32_t sampleRateHz = 0, maxPcmGapMs = 0, maxPumpGapMs = 0;
  std::uint32_t slowStreamEvents = 0, streamLostEvents = 0;
  std::uint32_t reserveTargetBytes = 0, reserveRefills = 0;
  std::uint8_t channels = 0;
  bool hasBufferSample = false; // A post-PCM input low-water mark has been observed.
  bool reserveSupported = false, reserveBuffering = false;
};

struct Snapshot {
  State state = State::Idle;
  bool fatal = false;
  RadioModel::Station requested, confirmed;
  std::uint32_t generation = 0;
  std::uint16_t bitrateKbps = 0;
  char codec[12] = {}, title[192] = {}, detail[80] = {};
  Diagnostics diagnostics;
};

namespace presentation_detail {
inline bool sameStation(const RadioModel::Station& a, const RadioModel::Station& b) {
  return a.source == b.source && a.bitrateKbps == b.bitrateKbps &&
      !std::strncmp(a.id, b.id, sizeof(a.id)) && !std::strncmp(a.name, b.name, sizeof(a.name)) &&
      !std::strncmp(a.url, b.url, sizeof(a.url)) && !std::strncmp(a.country, b.country, sizeof(a.country)) &&
      !std::strncmp(a.location, b.location, sizeof(a.location));
}
} // namespace presentation_detail

// Buffer telemetry changes continuously. It must not turn a static player
// screen into a stream of full-frame redraws; callers still copy fresh data.
inline bool samePresentation(const Snapshot& a, const Snapshot& b) {
  return a.state == b.state && a.fatal == b.fatal && a.generation == b.generation &&
      a.bitrateKbps == b.bitrateKbps && presentation_detail::sameStation(a.requested, b.requested) &&
      presentation_detail::sameStation(a.confirmed, b.confirmed) &&
      !std::strncmp(a.codec, b.codec, sizeof(a.codec)) &&
      !std::strncmp(a.title, b.title, sizeof(a.title)) &&
      !std::strncmp(a.detail, b.detail, sizeof(a.detail));
}

}  // namespace RadioAudio
