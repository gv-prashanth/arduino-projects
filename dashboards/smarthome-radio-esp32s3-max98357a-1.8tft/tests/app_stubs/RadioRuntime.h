#pragma once

// Hardware/service boundary for the real RadioApp.h host integration tests.
// App navigation, transaction, models, text fitting and screen drawing stay real.
#include <algorithm>
#include <cassert>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <string>
#include <vector>
#include "../../RadioConfig.h"
#include "../../CatalogTypes.h"
#include "../../AudioState.h"
#include "../../SettingsRestore.h"
#include "../../RadioFonts.h"
#include "../../ProportionalText.h"
#define RADIO_INPUT_HOST_TEST
#include "../../RotaryInput.h"

using std::min;
using std::max;
template<class T> inline T constrain(T value, T low, T high) { return max(low, min(value, high)); }
inline std::size_t strlcpy(char* out, const char* in, std::size_t capacity) {
  const std::size_t length = std::strlen(in);
  if (capacity) { const std::size_t used = min(length, capacity - 1); std::memcpy(out, in, used); out[used] = 0; }
  return length;
}
class String {
 public:
  String() = default;
  String(const char* text) : text_(text ? text : "") {}
  String(const std::string& text) : text_(text) {}
  String(int value) : text_(std::to_string(value)) {}
  String(unsigned value) : text_(std::to_string(value)) {}
  const char* c_str() const { return text_.c_str(); }
  unsigned length() const { return static_cast<unsigned>(text_.size()); }
  String substring(unsigned begin, unsigned end) const {
    return begin >= length() ? String() : String(text_.substr(begin, end > begin ? end - begin : 0));
  }
  String& operator+=(char c) { text_ += c; return *this; }
  String& operator+=(const String& other) { text_ += other.text_; return *this; }
  friend String operator+(const String& a, const String& b) { return String(a.text_ + b.text_); }
 private:
  std::string text_;
};
namespace RadioHost { inline uint32_t now = 1000; }
inline uint32_t millis() { return RadioHost::now; }
inline void delay(uint32_t ms) { RadioHost::now += ms; }
struct SerialStub {
  void begin(unsigned) {}
  int available() const { return 0; }
  int read() const { return -1; }
  template<class... Args> void printf(const char*, Args...) {}
  void println(const char*) {}
};
inline SerialStub Serial;
struct SpiStub { void begin(int, int, int, int) {} };
inline SpiStub SPI;
struct EspStub { unsigned getFreeHeap() const { return 200000; } unsigned getPsramSize() const { return 8388608; } };
inline EspStub ESP;
constexpr uint16_t ST77XX_BLACK = 0x0000, ST77XX_WHITE = 0xffff, ST77XX_RED = 0xf800;
constexpr uint16_t ST77XX_YELLOW = 0xffe0, ST77XX_CYAN = 0x07ff, ST77XX_GREEN = 0x07e0;
constexpr int INITR_BLACKTAB = 0;

class GFXcanvas16 {
 public:
  GFXcanvas16(int width, int height) : width_(width), height_(height), pixels(width * height) {}
  uint16_t* getBuffer() { return pixels.data(); }
  const uint16_t* getBuffer() const { return pixels.data(); }
  void fillScreen(uint16_t color) { std::fill(pixels.begin(), pixels.end(), color); }
  void drawPixel(int x, int y, uint16_t color) {
    if (x < 0 || y < 0 || x >= width_ || y >= height_) { ++clipped; return; }
    pixels[static_cast<unsigned>(y * width_ + x)] = color;
  }
  void fillRect(int x, int y, int width, int height, uint16_t color) {
    for (int py = y; py < y + height; ++py) for (int px = x; px < x + width; ++px) drawPixel(px, py, color);
  }
  void drawFastHLine(int x, int y, int width, uint16_t color) { fillRect(x, y, width, 1, color); }
  void drawLine(int x0, int y0, int x1, int y1, uint16_t color) {
    const int dx = std::abs(x1 - x0), dy = -std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    for (;;) {
      drawPixel(x0, y0, color);
      if (x0 == x1 && y0 == y1) break;
      const int doubled = 2 * error;
      if (doubled >= dy) { error += dy; x0 += sx; }
      if (doubled <= dx) { error += dx; y0 += sy; }
    }
  }
  static int edge(int ax, int ay, int bx, int by, int x, int y) {
    return (x - ax) * (by - ay) - (y - ay) * (bx - ax);
  }
  void fillTriangle(int ax, int ay, int bx, int by, int cx, int cy, uint16_t color) {
    for (int y = min({ay, by, cy}); y <= max({ay, by, cy}); ++y)
      for (int x = min({ax, bx, cx}); x <= max({ax, bx, cx}); ++x) {
        const int a = edge(ax, ay, bx, by, x, y), b = edge(bx, by, cx, cy, x, y);
        const int c = edge(cx, cy, ax, ay, x, y);
        if ((a >= 0 && b >= 0 && c >= 0) || (a <= 0 && b <= 0 && c <= 0)) drawPixel(x, y, color);
      }
  }
  void setTextWrap(bool) {}
  void setTextSize(uint8_t size) { textSize_ = size; }
  void setTextColor(uint16_t color) { textColor_ = color; }
  void setCursor(int x, int y) { cursorX_ = x; cursorY_ = y; }
  // Only legacy startup/Wi-Fi text uses print; the three tested screens use the
  // real proportional bitmap renderer directly and do not pass through here.
  void print(const String& text) {
    RadioDisplay::drawText(*this, *RadioFonts::Chrome, cursorX_, cursorY_, width_ - cursorX_,
                          text.c_str(), textColor_, false, 0);
    cursorX_ += static_cast<int>(text.length()) * 6 * textSize_;
  }
  uint16_t at(int x, int y) const { return pixels[static_cast<unsigned>(y * width_ + x)]; }
  bool contains(int x, int y, int width, int height, uint16_t color) const {
    for (int py = y; py < y + height; ++py) for (int px = x; px < x + width; ++px)
      if (at(px, py) == color) return true;
    return false;
  }
  void writePpm(const std::string& path) const {
    std::ofstream out(path, std::ios::binary); assert(out.good());
    out << "P6\n" << width_ << ' ' << height_ << "\n255\n";
    for (uint16_t pixel : pixels) {
      const char rgb[] = {static_cast<char>(((pixel >> 11) & 31) * 255 / 31),
                          static_cast<char>(((pixel >> 5) & 63) * 255 / 63),
                          static_cast<char>((pixel & 31) * 255 / 31)};
      out.write(rgb, 3);
    }
  }
 private:
  int width_, height_, cursorX_ = 0, cursorY_ = 0;
  uint8_t textSize_ = 1;
  uint16_t textColor_ = ST77XX_WHITE;
 public:
  std::vector<uint16_t> pixels;
  unsigned clipped = 0;
};
class Adafruit_ST7735 : public GFXcanvas16 {
 public:
  Adafruit_ST7735(int, int, int) : GFXcanvas16(160, 128) {}
  void initR(int) {}
  void setRotation(int) {}
  void drawRGBBitmap(int, int, uint16_t* source, int width, int height) {
    pixels.assign(source, source + width * height);
  }
};
class Audio {};
namespace RadioAudio {
template<class T> struct MetadataCallbacks { static constexpr bool modern = false; };
inline const char* pcmApiName() { return "host PCM"; }
inline const char* stateName(State state) {
  static const char* names[] = {"idle", "connecting", "playing", "buffering", "retrying", "failed", "offline"};
  return names[static_cast<unsigned>(state)];
}
class AudioService {
 public:
  bool begin() { return true; }
  bool start(const RadioModel::Station& station, const RadioModel::Station&, uint32_t generation) {
    ++startCalls; lastStation = station; lastGeneration = generation; return accepting;
  }
  void volume(uint8_t value) { ++volumeCalls; lastVolume = value; }
  void network(bool value) { connected = value; }
  bool poll(Snapshot& value) { if (updates.empty()) return false; value = updates.front(); updates.pop_front(); return true; }
  bool accepting = true, connected = false;
  unsigned startCalls = 0, volumeCalls = 0;
  uint8_t lastVolume = 0;
  uint32_t lastGeneration = 0;
  RadioModel::Station lastStation;
  std::deque<Snapshot> updates;
};
}
namespace RadioCatalog {
class CatalogService {
 public:
  bool begin() { return true; }
  bool request(const Request& value) { requests.push_back(value); return accepting; }
  void cancel(uint32_t generation) { cancellations.push_back(generation); }
  bool poll(Result*& result) { if (results.empty()) return false; result = results.front(); results.pop_front(); return true; }
  bool pollStarted(uint32_t& generation) { if (started.empty()) return false; generation = started.front(); started.pop_front(); return true; }
  void clear() { for (Result* result : results) delete result; results.clear(); }
  bool accepting = true;
  std::vector<Request> requests;
  std::vector<uint32_t> cancellations;
  std::deque<Result*> results;
  std::deque<uint32_t> started;
};
}
namespace RadioStorage {
struct WifiCredentials { char ssid[33] = {}, password[65] = {}; };
enum class Kind : uint8_t { Settings, Wifi, Cache };
struct Status { Kind kind = Kind::Settings; char message[80] = {}; };
class SettingsService {
 public:
  bool begin() { return true; }
  bool load(RadioModel::Settings&, RadioModel::Station&, WifiCredentials&) { return false; }
  RadioCatalog::Result* loadCache(const RadioCatalog::Filter&) { return nullptr; }
  bool save(const RadioModel::Settings& value, const RadioModel::Station& station) {
    ++saveCalls; savedSettings = value; savedStation = station; return true;
  }
  bool saveWifi(const WifiCredentials&) { return true; }
  bool saveCache(const RadioCatalog::Result&, const RadioModel::Settings&) { ++cacheCalls; return true; }
  bool poll(Status&) { return false; }
  unsigned saveCalls = 0, cacheCalls = 0;
  RadioModel::Settings savedSettings;
  RadioModel::Station savedStation;
};
}
namespace RadioWifi {
class WifiManager {
 public:
  bool begin(const RadioStorage::WifiCredentials&, const char*, const char*) { return true; }
  void loop() {}
  bool online() const { return connected; }
  const char* status() const { return connected ? "Wi-Fi connected" : "Wi-Fi reconnecting"; }
  bool portalActive() const { return portal; }
  void startPortal() { portal = true; }
  const char* apName() const { return "GTS Radio Setup"; }
  const char* apPassword() const { return "test-password"; }
  String portalAddress() const { return String("http://192.168.4.1"); }
  bool pollCredentials(RadioStorage::WifiCredentials&) { return false; }
  bool connected = true, portal = false;
};
}
namespace RadioInput {
class RotaryInput {
 public:
  bool begin(int, int, int) { return true; }
  bool poll(Event& event) {
    if (events.empty()) return false;
    event = events.front(); events.pop_front();
    if (event.type == EventType::ButtonDown) { down = true; since = event.atMs; }
    if (event.type == EventType::ButtonUp) down = false;
    return true;
  }
  bool buttonDown() const { return down; }
  uint32_t pressedAt() const { return since; }
  uint32_t overflowCount() const { return 0; }
  std::deque<Event> events;
  bool down = false;
  uint32_t since = 0;
};
}
