#pragma once

#include <cassert>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

using std::int16_t;
using std::int32_t;
using std::uint8_t;
using std::uint16_t;
using std::uint32_t;

namespace RadioAudioHost {
struct StopSimulation {};
extern uint32_t nowMs, stopAt;
extern bool continuousFrames, pinoutReady;
extern bool framesEnabled, silentPcm, emitStartupMetadata;
extern int32_t frameSamples;
extern unsigned weakDefaultCalls;

inline std::size_t copyText(char* destination, const char* source, std::size_t capacity) {
  const std::size_t length = std::strlen(source);
  if (capacity) {
    const std::size_t copied = length < capacity ? length : capacity - 1;
    std::memcpy(destination, source, copied);
    destination[copied] = '\0';
  }
  return length;
}
}  // namespace RadioAudioHost

// Supply Arduino's strlcpy on hosts whose C library does not provide it.
#define strlcpy RadioAudioHost::copyText

inline uint32_t millis() { return RadioAudioHost::nowMs; }

// Queue fakes preserve the production byte-copy and one-item overwrite semantics.
// Tasks run deterministically in the test thread; these critical sections do not
// simulate concurrent execution or verify the actual library's internal mutex.
struct Queue {
  std::size_t length, itemSize;
  std::deque<std::vector<uint8_t>> items;
};
using QueueHandle_t = Queue*;
using TaskHandle_t = void*;
using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(mux) ((void)(mux))
#define portEXIT_CRITICAL(mux) ((void)(mux))
constexpr int pdTRUE = 1, pdPASS = 1;
constexpr uint32_t portMAX_DELAY = 0xffffffffu;
#define pdMS_TO_TICKS(ms) (ms)

inline QueueHandle_t xQueueCreate(std::size_t length, std::size_t itemSize) {
  return new Queue{length, itemSize, {}};
}
inline int xQueueSend(Queue* queue, const void* value, uint32_t) {
  if (queue->items.size() >= queue->length) return 0;
  const auto* bytes = static_cast<const uint8_t*>(value);
  queue->items.emplace_back(bytes, bytes + queue->itemSize);
  return pdTRUE;
}
inline int xQueueOverwrite(Queue* queue, const void* value) {
  assert(queue->length == 1);
  queue->items.clear();
  return xQueueSend(queue, value, 0);
}
inline int xQueueReceive(Queue* queue, void* value, uint32_t) {
  if (queue->items.empty()) return 0;
  std::memcpy(value, queue->items.front().data(), queue->itemSize);
  queue->items.pop_front();
  return pdTRUE;
}
inline int xTaskCreatePinnedToCore(void (*)(void*), const char*, uint32_t, void*, int,
                                 TaskHandle_t*, int) { return pdPASS; }
inline void vTaskDelete(void*) {}
inline void vTaskDelay(uint32_t milliseconds) {
  RadioAudioHost::nowMs += milliseconds;
  if (RadioAudioHost::nowMs >= RadioAudioHost::stopAt) throw RadioAudioHost::StopSimulation{};
}

struct SerialStub {
  template <class... Args> void printf(const char*, Args...) {}
};
inline SerialStub Serial;

class String {
 public:
  String(const char* text) : value_(text) {}
  void toLowerCase() {
    for (char& c : value_) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  int indexOf(const char* text) {
    const auto position = value_.find(text);
    return position == std::string::npos ? -1 : static_cast<int>(position);
  }

 private:
  std::string value_;
};
