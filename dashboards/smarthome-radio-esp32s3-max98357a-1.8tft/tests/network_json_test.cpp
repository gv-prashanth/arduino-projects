#include "../NetworkJsonReader.h"
#include <ArduinoJson.h>

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

using RadioNetwork::JsonReader;
using RadioNetwork::ReadLimits;
using RadioNetwork::ReadStatus;

struct FakeClock {
  uint32_t origin = 0;
  uint64_t elapsed = 0;
  uint32_t now() const { return origin + static_cast<uint32_t>(elapsed); }
  void pause() { ++elapsed; }
};

// Arrival times belong to bytes already sent by the peer. connected() may be
// false while available() still exposes its buffered tail, as on a real socket.
struct FakeClient {
  explicit FakeClient(FakeClock& clock) : clock(clock) {}
  FakeClock& clock;
  std::string body;
  std::vector<uint64_t> arrivals;
  size_t position = 0, maximumRead = std::numeric_limits<size_t>::max();
  uint64_t closeAt = std::numeric_limits<uint64_t>::max();
  unsigned negativeReads = 0, zeroReads = 0, readCalls = 0;

  void packet(const std::string& text, uint64_t at) {
    assert(arrivals.empty() || at >= arrivals.back());
    body += text;
    arrivals.insert(arrivals.end(), text.size(), at);
  }
  int available() const {
    size_t end = position;
    while (end < arrivals.size() && arrivals[end] <= clock.elapsed) ++end;
    return static_cast<int>(end - position);
  }
  int read(uint8_t* destination, size_t length) {
    ++readCalls;
    const int ready = available();
    if (!ready) return -1;  // Pinned Secure::read() during a TLS packet gap.
    if (negativeReads) { --negativeReads; return -1; }
    if (zeroReads) { --zeroReads; return 0; }
    const size_t count = std::min(length, std::min(static_cast<size_t>(ready), maximumRead));
    std::memcpy(destination, body.data() + position, count);
    position += count;
    return static_cast<int>(count);
  }
  bool connected() const { return clock.elapsed < closeAt; }
};

// Reproduce the old Stream adapter's relevant boundary: ArduinoJson asks
// readBytes(..., 1), and NetworkClient::readBytes stops on Secure::read() == -1.
// This wrapper intentionally does not wait across that transient negative read.
class LegacyStreamReader {
 public:
  explicit LegacyStreamReader(FakeClient& client) : client_(client) {}
  int read() {
    char byte;
    return readBytes(&byte, 1) ? static_cast<unsigned char>(byte) : -1;
  }
  size_t readBytes(char* destination, size_t length) {
    size_t count = 0;
    while (count < length) {
      const int received = client_.read(reinterpret_cast<uint8_t*>(destination + count), length - count);
      if (received <= 0) break;
      count += static_cast<size_t>(received);
    }
    return count;
  }

 private:
  FakeClient& client_;
};

static ReadLimits limits(uint32_t idle, uint32_t total, size_t maximum = 128 * 1024) {
  ReadLimits result;
  result.idleTimeoutMs = idle;
  result.totalTimeoutMs = total;
  result.maxBytes = maximum;
  return result;
}

template<class Reader>
static std::string drain(Reader& reader) {
  std::string result;
  for (int byte = reader.read(); byte >= 0; byte = reader.read()) {
    assert(byte <= 255);
    result += static_cast<char>(byte);
  }
  return result;
}

template<class Reader>
static void assertTerminal(Reader& reader, FakeClient& client, FakeClock& clock, ReadStatus expected) {
  assert(reader.status() == expected);
  const uint64_t elapsed = clock.elapsed;
  const unsigned calls = client.readCalls;
  char byte = '?';
  assert(reader.read() == -1);
  assert(reader.readBytes(&byte, 1) == 0 && byte == '?');
  assert(reader.status() == expected);
  assert(clock.elapsed == elapsed && client.readCalls == calls);
}

static void testPacketGapRegressionWithArduinoJson() {
  const std::string body =
      "[{\"stationuuid\":\"test-id\",\"name\":\"Delayed radio\","
      "\"url_resolved\":\"https://example.org/live\",\"codec\":\"MP3\",\"unused\":\"ignored\"}]";
  const size_t split = body.find("Delayed") + 3;
  FakeClock oldClock;
  FakeClient oldClient(oldClock);
  oldClient.packet(body.substr(0, split), 0);
  oldClient.packet(body.substr(split), 25);
  LegacyStreamReader oldReader(oldClient);
  StaticJsonDocument<1024> oldDocument;
  const DeserializationError oldError = deserializeJson(oldDocument, oldReader);
  assert(oldError == DeserializationError::IncompleteInput);
  assert(oldClock.elapsed == 0 && oldClient.position == split);

  FakeClock clock;
  FakeClient client(clock);
  client.packet(body.substr(0, split), 0);
  client.packet(body.substr(split), 25);
  JsonReader<FakeClient, FakeClock, 16> reader(client, clock, static_cast<int>(body.size()));
  StaticJsonDocument<128> filter;
  filter[0]["name"] = true;
  filter[0]["url_resolved"] = true;
  StaticJsonDocument<1024> document;
  const DeserializationError error = deserializeJson(document, reader, DeserializationOption::Filter(filter));
  assert(!error);
  assert(std::strcmp(document[0]["name"], "Delayed radio") == 0);
  assert(std::strcmp(document[0]["url_resolved"], "https://example.org/live") == 0);
  assert(!document[0].containsKey("unused"));
  assert(drain(reader).empty());
  assert(reader.received() == body.size() && reader.consumed() == body.size());
  assert(clock.elapsed == 25);
  assertTerminal(reader, client, clock, ReadStatus::Complete);
}

static void testEscapeAndUnicodeGaps() {
  const std::string raw = "\xe0\xa4\xb0\xe0\xa4\xbe\xe0\xa4\x97";
  const std::string body = "{\"name\":\"A\\\"B\\n\\u20ac \\uD83D\\uDE03\",\"raw\":\"" + raw + "\"}";
  FakeClock clock;
  FakeClient client(clock);
  // One byte per packet splits escapes, surrogate pairs, and UTF-8 codepoints.
  for (size_t i = 0; i < body.size(); ++i) client.packet(body.substr(i, 1), i);
  JsonReader<FakeClient, FakeClock, 4> reader(client, clock, static_cast<int>(body.size()), limits(10, 2000));
  StaticJsonDocument<512> document;
  assert(!deserializeJson(document, reader));
  assert(std::strcmp(document["name"], "A\"B\n\xe2\x82\xac \xf0\x9f\x98\x83") == 0);
  assert(std::strcmp(document["raw"], raw.c_str()) == 0);
  assert(drain(reader).empty());
  assertTerminal(reader, client, clock, ReadStatus::Complete);
}

static void testPartialAndTransientReads() {
  FakeClock clock;
  FakeClient client(clock);
  client.packet("abcdef", 0);
  client.maximumRead = 2;
  client.negativeReads = client.zeroReads = 1;
  JsonReader<FakeClient, FakeClock, 4> reader(client, clock, 6, limits(10, 100));
  char output[8];
  std::memset(output, '?', sizeof(output));
  assert(reader.readBytes(output, 4) == 4);
  assert(std::memcmp(output, "abcd", 4) == 0 && output[4] == '?');
  assert(reader.consumed() == 4 && reader.received() >= reader.consumed());
  std::memset(output, '?', sizeof(output));
  assert(reader.readBytes(output, sizeof(output)) == 2);
  assert(std::memcmp(output, "ef", 2) == 0 && output[2] == '?');
  assert(reader.received() == 6 && reader.consumed() == 6 && client.position == 6);
  assert(client.negativeReads == 0 && client.zeroReads == 0 && clock.elapsed == 2);
  assertTerminal(reader, client, clock, ReadStatus::Complete);
}

static void testClosedBufferedTailAndLengths() {
  {
    FakeClock clock;
    FakeClient client(clock);
    client.packet("buffered tail", 0);
    client.closeAt = 0;
    JsonReader<FakeClient, FakeClock, 3> reader(client, clock);
    assert(drain(reader) == "buffered tail");
    assert(reader.received() == reader.consumed() && clock.elapsed == 0);
    assertTerminal(reader, client, clock, ReadStatus::Complete);
  }
  {
    FakeClock clock;
    FakeClient client(clock);
    client.packet("abc", 0);
    client.closeAt = 0;
    JsonReader<FakeClient, FakeClock, 2> reader(client, clock, 9);
    assert(drain(reader) == "abc");
    assert(reader.received() == 3 && reader.consumed() == 3);
    assertTerminal(reader, client, clock, ReadStatus::Truncated);
  }
  {
    FakeClock clock;
    FakeClient client(clock);
    client.packet("abcNEXT", 0);
    JsonReader<FakeClient, FakeClock, 16> reader(client, clock, 3);
    assert(drain(reader) == "abc");
    assert(reader.received() == 3 && client.position == 3 && clock.elapsed == 0);
    assertTerminal(reader, client, clock, ReadStatus::Complete);
  }
  {
    FakeClock clock;
    FakeClient client(clock);
    JsonReader<FakeClient, FakeClock> reader(client, clock, 0);
    assert(drain(reader).empty());
    assert(reader.received() == 0 && reader.consumed() == 0);
    assertTerminal(reader, client, clock, ReadStatus::Complete);
  }
}

static void testIdleAndTotalDeadlines() {
  {
    FakeClock clock;
    FakeClient client(clock);
    JsonReader<FakeClient, FakeClock> reader(client, clock, -1, limits(5, 50));
    assert(drain(reader).empty());
    assert(clock.elapsed >= 5 && clock.elapsed <= 6);
    assertTerminal(reader, client, clock, ReadStatus::IdleTimeout);
  }
  {
    FakeClock clock;
    FakeClient client(clock);
    client.packet("abc", 0);
    client.packet("def", 4);
    client.packet("ghi", 8);
    client.closeAt = 8;
    JsonReader<FakeClient, FakeClock, 2> reader(client, clock, -1, limits(5, 50));
    assert(drain(reader) == "abcdefghi");  // Progress resets the idle wait.
    assert(clock.elapsed == 8);
    assertTerminal(reader, client, clock, ReadStatus::Complete);
  }
  {
    FakeClock clock;
    FakeClient client(clock);
    for (int i = 0; i < 10; ++i) client.packet(std::string(1, static_cast<char>('a' + i)), i * 3);
    JsonReader<FakeClient, FakeClock, 2> reader(client, clock, -1, limits(5, 11));
    assert(drain(reader) == "abcd");  // Trickle traffic cannot defeat the total deadline.
    assert(clock.elapsed >= 11 && clock.elapsed <= 12);
    assertTerminal(reader, client, clock, ReadStatus::Deadline);
  }
}

static void testSizeLimits() {
  {
    FakeClock clock;
    FakeClient client(clock);
    client.packet("123456", 0);
    JsonReader<FakeClient, FakeClock> reader(client, clock, 6, limits(5, 50, 5));
    assert(drain(reader).size() <= 5 && reader.received() <= 5);
    assertTerminal(reader, client, clock, ReadStatus::TooLarge);
  }
  {
    FakeClock clock;
    FakeClient client(clock);
    client.packet("123456", 0);
    client.closeAt = 0;
    JsonReader<FakeClient, FakeClock, 2> reader(client, clock, -1, limits(5, 50, 5));
    assert(drain(reader).size() <= 5 && reader.received() <= 5);
    assertTerminal(reader, client, clock, ReadStatus::TooLarge);
  }
  for (const int contentLength : {-1, 5}) {
    FakeClock clock;
    FakeClient client(clock);
    client.packet("12345", 0);
    if (contentLength < 0) client.closeAt = 0;
    JsonReader<FakeClient, FakeClock, 2> reader(client, clock, contentLength, limits(5, 50, 5));
    assert(drain(reader) == "12345");  // Exactly maxBytes is still allowed.
    assert(reader.received() == 5 && reader.consumed() == 5);
    assertTerminal(reader, client, clock, ReadStatus::Complete);
  }
}

static void testClockRollover() {
  {
    FakeClock clock;
    clock.origin = std::numeric_limits<uint32_t>::max() - 3;
    FakeClient client(clock);
    const std::string body = "{\"value\":7}";
    client.packet(body, 8);
    JsonReader<FakeClient, FakeClock, 4> reader(client, clock, static_cast<int>(body.size()), limits(20, 50));
    StaticJsonDocument<128> document;
    assert(!deserializeJson(document, reader) && document["value"].as<int>() == 7);
    assert(drain(reader).empty());
    assert(clock.elapsed == 8 && clock.now() < clock.origin);
    assertTerminal(reader, client, clock, ReadStatus::Complete);
  }
  {
    FakeClock clock;
    clock.origin = std::numeric_limits<uint32_t>::max() - 2;
    FakeClient client(clock);
    JsonReader<FakeClient, FakeClock> reader(client, clock, -1, limits(5, 50));
    assert(drain(reader).empty());
    assert(clock.elapsed >= 5 && clock.elapsed <= 6 && clock.now() < clock.origin);
    assertTerminal(reader, client, clock, ReadStatus::IdleTimeout);
  }
}

static void testJsonBodyCompletion() {
  {
    FakeClock clock;
    FakeClient client(clock);
    client.packet("[]", 0);
    client.closeAt = 0;
    JsonReader<FakeClient, FakeClock> reader(client, clock, 100);
    StaticJsonDocument<128> document;
    assert(!deserializeJson(document, reader));
    assert(document.is<JsonArray>() && document.size() == 0);
    // Parsing an enclosed JSON value alone does not verify HTTP body framing.
    assert(!reader.finish());
    assertTerminal(reader, client, clock, ReadStatus::Truncated);
  }
  {
    FakeClock clock;
    FakeClient client(clock);
    const std::string whitespace = "\r\n \t\r\n";
    client.packet("[]", 0);
    client.packet(whitespace, 3);
    JsonReader<FakeClient, FakeClock, 4> reader(client, clock, static_cast<int>(2 + whitespace.size()));
    StaticJsonDocument<128> document;
    assert(!deserializeJson(document, reader));
    assert(reader.finish());
    assert(reader.received() == 2 + whitespace.size() && reader.consumed() == reader.received());
    assert(clock.elapsed == 3);
    assert(reader.finish());
    assertTerminal(reader, client, clock, ReadStatus::Complete);
  }
  for (const std::string& tail : {std::string("extra"), std::string("[]"), std::string("\v\f"), std::string(1, '\0')}) {
    FakeClock clock;
    FakeClient client(clock);
    client.packet("[]", 0);
    client.packet(tail, 3);
    JsonReader<FakeClient, FakeClock> reader(client, clock, static_cast<int>(2 + tail.size()));
    StaticJsonDocument<128> document;
    assert(!deserializeJson(document, reader));
    assert(!reader.finish());
    assert(reader.status() == ReadStatus::InvalidTail);
  }
  {
    FakeClock clock;
    FakeClient client(clock);
    client.packet("[]", 0);
    client.packet("\r\n", 3);
    client.closeAt = 3;
    JsonReader<FakeClient, FakeClock, 4> reader(client, clock);  // Connection-delimited body.
    StaticJsonDocument<128> document;
    assert(!deserializeJson(document, reader));
    assert(reader.finish());
    assert(reader.received() == 4 && reader.consumed() == 4 && clock.elapsed == 3);
    assertTerminal(reader, client, clock, ReadStatus::Complete);
  }
  for (const int contentLength : {-1, 100}) {
    FakeClock clock;
    FakeClient client(clock);
    client.packet("[]", 0);
    JsonReader<FakeClient, FakeClock> reader(client, clock, contentLength, limits(4, 20));
    StaticJsonDocument<128> document;
    assert(!deserializeJson(document, reader));
    assert(!reader.finish());  // JSON ended, but the declared/connection body did not.
    assert(clock.elapsed >= 4 && clock.elapsed <= 5);
    assertTerminal(reader, client, clock, ReadStatus::IdleTimeout);
  }
}

int main() {
  static_assert(ARDUINOJSON_VERSION_MAJOR == 6 && ARDUINOJSON_VERSION_MINOR == 21 &&
                ARDUINOJSON_VERSION_REVISION == 6, "Run with the pinned ArduinoJson 6.21.6 source");
  testPacketGapRegressionWithArduinoJson();
  testEscapeAndUnicodeGaps();
  testPartialAndTransientReads();
  testClosedBufferedTailAndLengths();
  testIdleAndTotalDeadlines();
  testSizeLimits();
  testClockRollover();
  testJsonBodyCompletion();
  std::puts("Network JSON reader tests passed (ArduinoJson 6.21.6)");
}
