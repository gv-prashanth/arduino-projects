#include "../ListeningScreen.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdlib>
#include <fstream>
#include <string>

namespace {
struct Canvas {
  static constexpr int Width = 160, Height = 128;
  std::array<uint16_t, Width * Height> pixels{};
  void drawPixel(int x, int y, uint16_t color) {
    assert(x >= 0 && x < Width && y >= 0 && y < Height);
    pixels[static_cast<unsigned>(y * Width + x)] = color;
  }
  void fillRect(int x, int y, int w, int h, uint16_t color) {
    for (int py = y; py < y + h; ++py)
      for (int px = x; px < x + w; ++px) drawPixel(px, py, color);
  }
  void drawFastHLine(int x, int y, int w, uint16_t color) { fillRect(x, y, w, 1, color); }
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
    for (int y = std::min({ay, by, cy}); y <= std::max({ay, by, cy}); ++y)
      for (int x = std::min({ax, bx, cx}); x <= std::max({ax, bx, cx}); ++x) {
        const int a = edge(ax, ay, bx, by, x, y), b = edge(bx, by, cx, cy, x, y);
        const int c = edge(cx, cy, ax, ay, x, y);
        if ((a >= 0 && b >= 0 && c >= 0) || (a <= 0 && b <= 0 && c <= 0)) drawPixel(x, y, color);
      }
  }
  uint16_t at(int x, int y) const { return pixels[static_cast<unsigned>(y * Width + x)]; }
  bool blank(int x, int y, int w, int h) const {
    for (int py = y; py < y + h; ++py) for (int px = x; px < x + w; ++px)
      if (at(px, py)) return false;
    return true;
  }
  bool contains(int x, int y, int w, int h, uint16_t color) const {
    for (int py = y; py < y + h; ++py) for (int px = x; px < x + w; ++px)
      if (at(px, py) == color) return true;
    return false;
  }
  void write(const std::string& path) const {
    std::ofstream out(path, std::ios::binary);
    assert(out.good());
    out << "P6\n160 128\n255\n";
    for (uint16_t pixel : pixels) {
      const char rgb[] = {static_cast<char>(((pixel >> 11) & 31) * 255 / 31),
                          static_cast<char>(((pixel >> 5) & 63) * 255 / 63),
                          static_cast<char>((pixel & 31) * 255 / 31)};
      out.write(rgb, 3);
    }
  }
};

void sameRegion(const Canvas& a, const Canvas& b, int x, int y, int w, int h) {
  for (int py = y; py < y + h; ++py) for (int px = x; px < x + w; ++px)
    assert(a.at(px, py) == b.at(px, py));
}

struct Radio {
  RadioModel::Settings settings;
  RadioModel::Station station;
  RadioAudio::Snapshot audio;
  RadioUi::NoticeCenter notices;
  RadioDisplay::ListeningMotion motion;
  bool online = true, buttonDown = false, fallback = false, animated = false;
  uint32_t heldMs = 0;
  Radio() {
    station.source = RadioModel::Source::Akashvani;
    RadioModel::copyText(station.id, "101");
    RadioModel::copyText(station.name, "Raagam");
    RadioModel::copyText(station.url, "https://example.com/live.m3u8");
    RadioModel::copyText(station.country, "IN");
    audio.state = RadioAudio::State::Playing;
    RadioModel::copyText(audio.codec, "AAC");
    RadioModel::copyText(audio.detail, "Decoded audio ready");
    audio.bitrateKbps = 128;
  }
  Canvas render(uint32_t now) {
    const RadioUi::Message message = RadioUi::messageFor(audio, online, "Wi-Fi reconnecting", settings.muted,
                                                        notices, fallback);
    RadioDisplay::ListeningFrame frame(settings, station, audio, station.name, message.text, message.tone, now);
    frame.online = online;
    frame.buttonDown = buttonDown;
    frame.heldMs = heldMs;
    Canvas canvas;
    animated = RadioDisplay::drawListeningScreen(canvas, frame, motion);
    return canvas;
  }
};

void testReadableAndStatic() {
  assert(RadioDisplay::fontHeight(*RadioFonts::Body) == 14);
  assert(RadioDisplay::fontHeight(*RadioFonts::Title) == 19);
  assert(RadioDisplay::textWidth(*RadioFonts::Body, "Click:mute Hold:menu") <= 152);
  Radio radio;
  const Canvas first = radio.render(100);
  assert(!radio.animated);
  assert(first.blank(0, 92, 160, 14)); // No redundant healthy playback text.
  for (uint32_t now : {4100u, 8100u, 60000u}) {
    const Canvas later = radio.render(now);
    assert(!radio.animated && first.pixels == later.pixels);
  }
  // Provider follows the actual station even when a different source is browsed.
  radio.settings.source = RadioModel::Source::RadioBrowser;
  const Canvas otherBrowser = radio.render(61000);
  assert(first.pixels == otherBrowser.pixels);
  Canvas expected;
  RadioDisplay::drawText(expected, *RadioFonts::Body, 4, 22, 152, "Akashvani / IN", RadioDisplay::Green, false, 0);
  sameRegion(first, expected, 0, 22, 160, 14);
  RadioDisplay::drawText(expected, *RadioFonts::Body, 4, 113, 152,
                         "Click:mute Hold:menu", RadioDisplay::Cyan, false, 0);
  sameRegion(first, expected, 0, 113, 160, 15);
  radio.station = RadioModel::Station{};
  radio.audio.state = RadioAudio::State::Idle;
  const Canvas noStation = radio.render(62000);
  expected = Canvas{};
  RadioDisplay::drawText(expected, *RadioFonts::Body, 4, 22, 152,
                         "No station selected", RadioDisplay::Green, false, 0);
  sameRegion(noStation, expected, 0, 22, 160, 14);
}

void testStatusAndMotion(const std::string& previewPrefix) {
  Radio radio;
  RadioModel::copyText(radio.audio.title, "Evening music");
  Canvas playing = radio.render(100);
  if (!previewPrefix.empty()) playing.write(previewPrefix + "playing.ppm");
  radio.settings.volume = 21;
  const Canvas maxVolume = radio.render(150);
  radio.settings.muted = true;
  const Canvas muted = radio.render(200);
  assert(!maxVolume.contains(100, 0, 16, 17, RadioDisplay::Yellow));
  assert(muted.contains(100, 0, 16, 17, RadioDisplay::Yellow));
  sameRegion(muted, maxVolume, 0, 0, 96, 18); // The mute symbol does not cover the logo.
  assert(!muted.blank(4, 92, 152, 14));
  if (!previewPrefix.empty()) muted.write(previewPrefix + "muted.ppm");
  radio.settings.muted = false;
  RadioModel::copyText(radio.station.name, "Visakhapatnam Akashvani Radio");
  RadioModel::copyText(radio.audio.title, "A long programme title from the station to exercise clipping and scrolling");
  const Canvas initial = radio.render(1000);
  assert(radio.animated);
  const Canvas dwell = radio.render(2400);
  assert(initial.pixels == dwell.pixels); // Newly changed rows start at the beginning.
  const Canvas scrolled = radio.render(4000);
  assert(initial.pixels != scrolled.pixels);
  sameRegion(initial, scrolled, 0, 110, 160, 18); // Body marquees never move the footer.
  if (!previewPrefix.empty()) initial.write(previewPrefix + "long-name.ppm");
  RadioModel::copyText(radio.station.name, "Raagam");
  radio.audio.title[0] = '\0';
  struct Example { RadioAudio::State state; const char* detail; const char* name; };
  const Example examples[] = {
    {RadioAudio::State::Connecting, "Connection attempt 1/3", "connecting"},
    {RadioAudio::State::Retrying, "Retrying 2/3: stream ended", "retrying"},
    {RadioAudio::State::Buffering, "Buffering; waiting for decoded audio", "buffering"},
    {RadioAudio::State::Failed, "Server unavailable", "failed"},
    {RadioAudio::State::Offline, "Wi-Fi offline; reconnecting", "offline"},
    {RadioAudio::State::Idle, "", "idle"},
  };
  uint32_t now = 10000;
  for (const Example& example : examples) {
    radio.audio.state = example.state;
    RadioModel::copyText(radio.audio.detail, example.detail);
    const Canvas frame = radio.render(now);
    assert(!frame.blank(4, 92, 152, 14)); // Status remains visible without metadata.
    if (example.state != RadioAudio::State::Buffering)
      assert(frame.blank(0, 70, 160, 14)); // Old codec never implies a new connection succeeded.
    for (uint32_t elapsed : {0u, 1499u, 1500u, 1540u, 4000u, 8000u, 60000u}) {
      const Canvas later = radio.render(now + elapsed);
      sameRegion(frame, later, 0, 110, 160, 18);
    }
    if (!previewPrefix.empty()) frame.write(previewPrefix + example.name + ".ppm");
    now += 100000;
  }
  radio.audio.state = RadioAudio::State::Playing;
  radio.audio.detail[0] = '\0';
  radio.notices.post("Station directory unavailable; previous list kept", 5000, now, RadioUi::NoticePriority::Warning);
  const Canvas warning = radio.render(now);
  if (!previewPrefix.empty()) warning.write(previewPrefix + "warning.ppm");
  radio.notices = RadioUi::NoticeCenter{};
  radio.fallback = true;
  const Canvas restored = radio.render(now + 1);
  if (!previewPrefix.empty()) restored.write(previewPrefix + "restored.ppm");
  radio.buttonDown = true;
  for (uint32_t held : {0u, 100u, 699u, 700u, 8000u}) {
    radio.heldMs = held;
    const Canvas heldFrame = radio.render(now + held);
    assert(radio.animated);
    sameRegion(restored, heldFrame, 0, 110, 160, 18);
  }
}
} // namespace

int main(int argc, char** argv) {
  testReadableAndStatic();
  testStatusAndMotion(argc > 1 ? argv[1] : "");
}
