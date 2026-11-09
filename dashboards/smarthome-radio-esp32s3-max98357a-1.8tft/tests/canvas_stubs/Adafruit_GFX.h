#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace CanvasTestMemory {
struct State {
  unsigned baseRequests = 0, baseFrees = 0, baseDestructors = 0, borrowedDestructors = 0;
  unsigned capsRequests = 0, capsFrees = 0, unreleasedBorrowed = 0;
  size_t baseBytes = 0, capsCount = 0, capsSize = 0;
  uint32_t caps = 0;
  void* baseLive = nullptr;
  void* capsLive = nullptr;
  bool failBase = false, failCaps = false;
};
inline State& state() { static State value; return value; }
inline void reset() {
  assert(!state().baseLive && !state().capsLive);
  state() = State{};
}
} // namespace CanvasTestMemory

// Match Adafruit GFX 1.12.6's allocation and ownership semantics: allocate=true
// uses malloc + memset; allocate=false leaves a borrowed null buffer. Its base
// destructor frees only a non-null owned buffer. Instrumentation records which
// allocator/destructor actually owns each allocation.
class GFXcanvas16 {
 public:
  GFXcanvas16(uint16_t width, uint16_t height, bool allocate = true)
      : buffer(nullptr), buffer_owned(allocate), width_(width), height_(height) {
    if (allocate) {
      auto& memory = CanvasTestMemory::state();
      ++memory.baseRequests;
      memory.baseBytes = size_t(width) * height * 2;
      if (!memory.failBase) {
        buffer = static_cast<uint16_t*>(std::malloc(memory.baseBytes));
        assert(buffer && !memory.baseLive);
        memory.baseLive = buffer;
        std::memset(buffer, 0, memory.baseBytes);
      }
    }
  }
  ~GFXcanvas16() {
    auto& memory = CanvasTestMemory::state();
    ++memory.baseDestructors;
    if (!buffer_owned) {
      ++memory.borrowedDestructors;
      if (buffer) ++memory.unreleasedBorrowed;
    }
    if (buffer && buffer_owned) {
      assert(memory.baseLive == buffer);
      ++memory.baseFrees;
      memory.baseLive = nullptr;
      std::free(buffer);
    }
  }
  uint16_t* getBuffer() const { return buffer; }
  int width() const { return width_; }
  int height() const { return height_; }

 protected:
  uint16_t* buffer;
  bool buffer_owned;

 private:
  const uint16_t width_, height_;
};
