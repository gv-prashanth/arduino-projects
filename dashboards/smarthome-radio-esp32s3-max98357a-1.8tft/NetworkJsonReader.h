#pragma once

#include <cstddef>
#include <cstdint>

namespace RadioNetwork {

enum class ReadStatus : uint8_t { Reading, Complete, Truncated, IdleTimeout, Deadline, TooLarge, InvalidTail, Cancelled };
struct ReadCancellation {
  bool (*test)(void*) = nullptr;
  void* context = nullptr;
  ReadCancellation(bool (*check)(void*) = nullptr, void* state = nullptr) : test(check), context(state) {}
  bool requested() const { return test && test(context); }
};
struct ReadLimits {
  uint32_t idleTimeoutMs = 5000;
  uint32_t totalTimeoutMs = 15000;
  size_t maxBytes = 128 * 1024;
  ReadCancellation cancellation;
};

// ArduinoJson's custom-reader interface deliberately bypasses NetworkClient's
// readBytes(). In ESP32 core 3.0.7 that method treats Secure::read() == -1 as
// EOF, even when it merely means that the next TLS record is not available yet.
// Client provides available(), read(uint8_t*, size_t), and connected(). Clock
// provides now() and pause(), keeping the actual transport policy host-testable.
template<class Client, class Clock, size_t BufferSize = 1024>
class JsonReader {
 public:
  JsonReader(Client& client, Clock& clock, int contentLength = -1, ReadLimits limits = {})
      : client_(client), clock_(clock), expected_(contentLength), limits_(limits), startedAt_(clock.now()) {
    static_assert(BufferSize > 0, "JSON reader buffer must not be empty");
    if (expected_ >= 0 && static_cast<size_t>(expected_) > limits_.maxBytes) status_ = ReadStatus::TooLarge;
    else if (expected_ == 0) status_ = ReadStatus::Complete;
  }

  int read() {
    if (position_ == filled_ && !refill()) return -1;
    ++consumed_;
    return buffer_[position_++];
  }
  size_t readBytes(char* destination, size_t length) {
    size_t count = 0;
    while (count < length) {
      const int value = read();
      if (value < 0) break;
      destination[count++] = static_cast<char>(value);
    }
    return count;
  }
  // deserializeJson stops at the closing delimiter. Verify the remaining HTTP
  // body before committing a catalog, including a promised but missing tail.
  bool finish() {
    for (int byte = read(); byte >= 0; byte = read()) {
      if (byte != ' ' && byte != '\t' && byte != '\r' && byte != '\n') {
        status_ = ReadStatus::InvalidTail;
        position_ = filled_;
        return false;
      }
    }
    return status_ == ReadStatus::Complete;
  }
  ReadStatus status() const { return status_; }
  size_t received() const { return received_; }
  size_t consumed() const { return consumed_; }

 private:
  Client& client_;
  Clock& clock_;
  const int expected_;
  const ReadLimits limits_;
  const uint32_t startedAt_;
  ReadStatus status_ = ReadStatus::Reading;
  uint8_t buffer_[BufferSize] = {};
  size_t position_ = 0, filled_ = 0, received_ = 0, consumed_ = 0;

  bool refill() {
    if (status_ != ReadStatus::Reading) return false;
    if (expected_ >= 0 && received_ == static_cast<size_t>(expected_)) {
      status_ = ReadStatus::Complete;
      return false;
    }
    const uint32_t waitingAt = clock_.now();
    for (;;) {
      if (limits_.cancellation.requested()) { status_ = ReadStatus::Cancelled; return false; }
      const uint32_t now = clock_.now();
      if (now - startedAt_ >= limits_.totalTimeoutMs) { status_ = ReadStatus::Deadline; return false; }
      const int available = client_.available();
      if (available > 0) {
        if (received_ >= limits_.maxBytes) { status_ = ReadStatus::TooLarge; return false; }
        size_t wanted = static_cast<size_t>(available);
        if (wanted > BufferSize) wanted = BufferSize;
        if (wanted > limits_.maxBytes - received_) wanted = limits_.maxBytes - received_;
        if (expected_ >= 0 && wanted > static_cast<size_t>(expected_) - received_)
          wanted = static_cast<size_t>(expected_) - received_;
        const int count = client_.read(buffer_, wanted);
        if (count > 0) {
          position_ = 0;
          filled_ = static_cast<size_t>(count);
          received_ += filled_;
          return true;
        }
        // Secure sockets may temporarily return -1 or 0 after available()>0.
        // A short/negative read alone is not proof that the response ended.
      }
      if (!client_.connected() && client_.available() <= 0) {
        status_ = expected_ >= 0 && received_ < static_cast<size_t>(expected_)
                    ? ReadStatus::Truncated : ReadStatus::Complete;
        return false;
      }
      if (now - waitingAt >= limits_.idleTimeoutMs) { status_ = ReadStatus::IdleTimeout; return false; }
      clock_.pause();
    }
  }
};

inline const char* statusName(ReadStatus status) {
  switch (status) {
    case ReadStatus::Reading: return "reading";
    case ReadStatus::Complete: return "complete";
    case ReadStatus::Truncated: return "truncated";
    case ReadStatus::IdleTimeout: return "read timeout";
    case ReadStatus::Deadline: return "body deadline";
    case ReadStatus::TooLarge: return "response too large";
    case ReadStatus::InvalidTail: return "invalid JSON tail";
    case ReadStatus::Cancelled: return "superseded request";
  }
  return "unknown";
}

// Read a complete non-JSON body using the same TLS-gap and framing checks.
// destination has capacity + 1 bytes, including its terminator. Check EOF even
// at exact capacity to distinguish a valid full buffer from overflow.
template<class Reader>
inline bool readCompleteBody(Reader& reader, char* destination, size_t capacity, size_t& length) {
  length = 0;
  if (!destination) return false;
  while (length < capacity) {
    const size_t count = reader.readBytes(destination + length,
                                         capacity - length < 1024 ? capacity - length : 1024);
    length += count;
    if (!count) break;
  }
  destination[length] = '\0';
  if (length == capacity && reader.read() >= 0) return false;
  return reader.status() == ReadStatus::Complete;
}

} // namespace RadioNetwork
