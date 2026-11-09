#pragma once

#include "Adafruit_GFX.h"

constexpr uint32_t MALLOC_CAP_8BIT = 1U << 2;
constexpr uint32_t MALLOC_CAP_SPIRAM = 1U << 10;

inline void* heap_caps_calloc(size_t count, size_t size, uint32_t capabilities) {
  auto& memory = CanvasTestMemory::state();
  ++memory.capsRequests;
  memory.capsCount = count; memory.capsSize = size; memory.caps = capabilities;
  if (memory.failCaps) return nullptr;
  assert(!memory.capsLive);
  memory.capsLive = std::calloc(count, size);
  assert(memory.capsLive);
  return memory.capsLive;
}
inline void heap_caps_free(void* buffer) {
  auto& memory = CanvasTestMemory::state();
  assert(buffer && buffer == memory.capsLive);
  ++memory.capsFrees;
  memory.capsLive = nullptr;
  std::free(buffer);
}
