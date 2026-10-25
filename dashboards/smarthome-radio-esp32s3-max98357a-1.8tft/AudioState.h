#pragma once

#include <cstdint>
#include "RadioModel.h"

namespace RadioAudio {

enum class State : std::uint8_t { Idle, Connecting, Playing, Buffering, Retrying, Failed, Offline };

struct Snapshot {
  State state = State::Idle;
  bool fatal = false;
  RadioModel::Station requested, confirmed;
  std::uint32_t generation = 0;
  std::uint16_t bitrateKbps = 0;
  char codec[12] = {}, title[192] = {}, detail[80] = {};
};

}  // namespace RadioAudio
