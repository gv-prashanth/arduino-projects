#include "../ListeningScreen.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdlib>
#include <fstream>
#include <string>

namespace {
static_assert(RadioDisplay::LargeDisplay, "This suite exercises the 2.4-inch profile");
struct Canvas {
  static constexpr int Width = 320, Height = 240;
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
  bool contains(int x, int y, int w, int h, uint16_t color) const {
    for (int py = y; py < y + h; ++py) for (int px = x; px < x + w; ++px)
      if (at(px, py) == color) return true;
    return false;
  }
  bool blank(int x, int y, int w, int h) const {
    for (int py = y; py < y + h; ++py) for (int px = x; px < x + w; ++px)
      if (at(px, py)) return false;
    return true;
  }
  void write(const std::string& path) const {
    std::ofstream out(path, std::ios::binary); assert(out.good());
    out << "P6\n320 240\n255\n";
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
  bool online = true, buttonDown = false, animated = false;
  uint32_t heldMs = 0;
  Radio() {
    station.source = RadioModel::Source::Akashvani;
    RadioModel::copyText(station.id, "101");
    RadioModel::copyText(station.name, "Raagam");
    RadioModel::copyText(station.url, "https://example.com/live.m3u8");
    RadioModel::copyText(station.country, "IN");
    RadioModel::copyText(station.location, "Andhra Pradesh");
    audio.state = RadioAudio::State::Playing;
    RadioModel::copyText(audio.codec, "AAC"); audio.bitrateKbps = 128;
  }
  Canvas render(uint32_t now) {
    const RadioUi::Message message = RadioUi::messageFor(audio, online, "Wi-Fi reconnecting", settings.muted,
                                                        notices, false, settings.volume == 0);
    RadioDisplay::ListeningFrame frame(settings, station, audio, station.name, message.text, message.tone, now);
    frame.online = online; frame.buttonDown = buttonDown; frame.heldMs = heldMs;
    Canvas result;
    animated = RadioDisplay::drawListeningScreen(result, frame, motion);
    return result;
  }
};
void testLayoutAndChrome() {
  using namespace RadioDisplay;
  assert(ScreenWidth == Canvas::Width && ScreenHeight == Canvas::Height);
  assert(fontHeight(*ChromeFont) >= 15 && fontHeight(*ChromeFont) <= 18);
  assert(fontHeight(*RowFont) >= 20 && fontHeight(*RowFont) <= StationRowHeight);
  assert(PlayerTitleTop + fontHeight(*PlayerTitleFont) * 2 < PlayerStatusTop);
  assert(PlayerMessageTop + fontHeight(*DetailFont) < FooterTop);
  assert(textWidth(*ChromeFont, "GTS RADIO") <= 112);
  assert(textWidth(*ChromeFont, "Location") <= 92);
  assert(textWidth(*ChromeFont, "Click Stations") <= 126);
  assert(textWidth(*ChromeFont, "Hold Settings") <= 118);
  RadioModel::Settings settings;
  Canvas base;
  brandHeader(base, settings);
  for (const char* context : {"p1/3", "p14/14", "Settings", "Source", "Location", "Genre", "Codec", "Bitrate"}) {
    Canvas caption; brandHeader(caption, settings, context);
    sameRegion(base, caption, 0, 0, 132, HeaderHeight);
    sameRegion(base, caption, 224, 0, 96, HeaderHeight);
    assert(caption.contains(132, 8, 92, fontHeight(*ChromeFont), MutedInk));
  }
  for (int value = static_cast<int>(Icon::Play); value <= static_cast<int>(Icon::ChevronUpDown); ++value) {
    Canvas icon;
    drawIcon(icon, static_cast<Icon>(value), 296, 216, 24, Cyan, 300);
    assert(!icon.blank(296, 216, 24, 24));
  }
  Canvas nav;
  navButton(nav, false, Icon::Prev, "Prev", false, true);
  navButton(nav, true, Icon::Next, "Next", true, false);
  assert(nav.contains(8, FooterControlTop, 84, FooterControlHeight, DisabledInk));
  assert(nav.contains(228, FooterControlTop, 84, FooterControlHeight, FocusBackground));
  for (int row = 0; row < VisibleStationRows; ++row)
    focusRow(nav, FirstStationTop + row * StationRowHeight - 1);
}
void testPlayerContent(const std::string& prefix) {
  using namespace RadioDisplay;
  Radio radio;
  const Canvas first = radio.render(1000);
  assert(!radio.animated && first.blank(ContentLeft, PlayerMessageTop, ContentWidth, fontHeight(*DetailFont)));
  Canvas expected;
  drawText(expected, *ChromeFont, ContentLeft, FilterTop, ContentWidth,
           "Akashvani / Andhra Pradesh", Green, false, 0);
  sameRegion(first, expected, 0, FilterTop, ScreenWidth, fontHeight(*ChromeFont));
  radio.settings.source = RadioModel::Source::RadioBrowser;
  assert(first.pixels == radio.render(61000).pixels);
  if (!prefix.empty()) first.write(prefix + "player.ppm");
  for (const char* name : {"Thiruvananthapuram", "Visakhapatnam Akashvani Radio", "A Station With A Very Long Name Which Must Fit Two Static Lines", "LongUnbrokenStationNameWhichMustNotReachTheCodecRow"}) {
    RadioModel::copyText(radio.station.name, name);
    const Canvas title = radio.render(62000);
    assert(!radio.animated && title.pixels == radio.render(65000).pixels);
    char one[128] = {}, two[128] = {};
    const GFXfont& font = *listening_detail::titleFont(name);
    listening_detail::titleLines(name, one, two, font);
    assert(textWidth(font, one) <= ContentWidth);
    sameRegion(first, title, 0, PlayerStatusTop, ScreenWidth, ScreenHeight - PlayerStatusTop);
    if (!prefix.empty() && !std::strcmp(name, "Visakhapatnam Akashvani Radio")) title.write(prefix + "long-name.ppm");
  }
  RadioModel::copyText(radio.station.name, "Raagam");
  RadioModel::copyText(radio.audio.title, "An extended programme title which uses the larger message area and remains clear while scrolling");
  const Canvas initial = radio.render(70000);
  const Canvas later = radio.render(75000);
  assert(radio.animated && initial.pixels != later.pixels);
  sameRegion(initial, later, 0, FooterTop, ScreenWidth, ScreenHeight - FooterTop);
  radio.audio.title[0] = 0;
  RadioModel::copyText(radio.station.location, "ANDAMAN AND NICOBAR ISLANDS - LONG OFFICIAL LOCATION NAME");
  const Canvas contextInitial = radio.render(80000);
  const Canvas contextLater = radio.render(85000);
  assert(radio.animated && contextInitial.pixels != contextLater.pixels);
  sameRegion(contextInitial, contextLater, 0, PlayerTitleTop, ScreenWidth, ScreenHeight - PlayerTitleTop);
}
void testPlaybackStates(const std::string& prefix) {
  using namespace RadioDisplay;
  Radio radio;
  const Canvas base = radio.render(1000);
  const struct {RadioAudio::State state; const char* message; const char* preview;} cases[] = {
    {RadioAudio::State::Connecting, "Connection attempt 1/3", "connecting"},
    {RadioAudio::State::Retrying, "Retrying 2/3: stream ended", "retrying"},
    {RadioAudio::State::Buffering, "Buffering; waiting for decoded audio", "buffering"},
    {RadioAudio::State::Failed, "Server unavailable", "failed"},
    {RadioAudio::State::Offline, "Wi-Fi offline; reconnecting", "offline"},
    {RadioAudio::State::Idle, "", "idle"}
  };
  uint32_t time = 10000;
  for (const auto& item : cases) {
    radio.audio.state = item.state; RadioModel::copyText(radio.audio.detail, item.message);
    const Canvas frame = radio.render(time);
    assert(!frame.blank(44, PlayerStatusTop, 264, fontHeight(*RowFont)));
    sameRegion(base, frame, 0, FooterTop, ScreenWidth, ScreenHeight - FooterTop);
    if (!prefix.empty()) frame.write(prefix + item.preview + ".ppm");
    time += 100000;
  }
  radio.audio.state = RadioAudio::State::Playing; radio.audio.detail[0] = 0;
  radio.settings.muted = true;
  const Canvas muted = radio.render(time);
  assert(muted.contains(230, 8, 14, 14, Yellow));
  assert(!muted.blank(ContentLeft, PlayerMessageTop, ContentWidth, fontHeight(*DetailFont)));
  if (!prefix.empty()) muted.write(prefix + "muted.ppm");
  radio.settings.muted = false; radio.settings.volume = 0;
  const Canvas zero = radio.render(time + 1);
  assert(zero.contains(230, 8, 14, 14, Yellow));
  radio.buttonDown = true;
  for (uint32_t held : {0u, 100u, 699u, 700u, 8000u}) {
    radio.heldMs = held;
    const Canvas frame = radio.render(time + held);
    assert(radio.animated);
    sameRegion(zero, frame, 0, FooterTop, ScreenWidth, ScreenHeight - FooterTop - 2);
  }
}
} // namespace

int main(int argc, char** argv) {
  const std::string prefix = argc > 1 ? argv[1] : "";
  testLayoutAndChrome();
  testPlayerContent(prefix);
  testPlaybackStates(prefix);
}
