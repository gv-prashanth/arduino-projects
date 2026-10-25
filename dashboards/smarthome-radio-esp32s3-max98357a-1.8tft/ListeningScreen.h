#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include "RadioFonts.h"
#include "ProportionalText.h"
#include "ListeningUi.h"

namespace RadioDisplay {
constexpr uint16_t White = 0xffff, Green = 0x07e0, Yellow = 0xffe0;
constexpr uint16_t Red = 0xf800, Cyan = 0x07ff, HeaderBackground = 0x2808;

// Text has already passed through the UI's UTF-8-to-display conversion. The
// renderer is shared by the device and the host pixel-layout checks/previews.
struct ListeningFrame {
  const RadioModel::Settings& settings;
  const RadioModel::Station& station;
  const RadioAudio::Snapshot& playback;
  const char* stationName;
  const char* messageText;
  RadioUi::Tone messageTone;
  uint32_t now;
  bool online = true, buttonDown = false;
  uint32_t heldMs = 0;
  ListeningFrame(const RadioModel::Settings& volume, const RadioModel::Station& selected,
                 const RadioAudio::Snapshot& audio, const char* name, const char* message,
                 RadioUi::Tone tone, uint32_t time)
      : settings(volume), station(selected), playback(audio), stationName(name), messageText(message),
        messageTone(tone), now(time) {}
};

struct RowMotion {
  char previous[192] = {};
  uint32_t changedAt = 0;
  uint32_t elapsed(const char* text, uint32_t now) {
    if (!text) text = "";
    if (std::strcmp(previous, text)) {
      RadioModel::copyText(previous, text);
      changedAt = now;
    }
    const uint32_t age = now - changedAt;
    return age > 1500 ? age - 1500 : 0; // Read a new message's beginning before scrolling.
  }
};
struct ListeningMotion { RowMotion context, station, technical, message; };

template<class Canvas>
inline void drawMuteIcon(Canvas& canvas, int x, int y, uint16_t color) {
  canvas.fillRect(x, y + 3, 3, 5, color);
  canvas.fillTriangle(x + 2, y + 3, x + 6, y, x + 6, y + 10, color);
  canvas.drawLine(x + 8, y + 3, x + 11, y + 7, color);
  canvas.drawLine(x + 11, y + 3, x + 8, y + 7, color);
}

template<class Canvas>
inline bool drawListeningScreen(Canvas& canvas, const ListeningFrame& frame, ListeningMotion& motion) {
  bool animated = false;
  const GFXfont& body = *RadioFonts::Body;
  const GFXfont& title = *RadioFonts::Title;
  canvas.fillRect(0, 0, 160, 18, HeaderBackground);
  char volume[12];
  std::snprintf(volume, sizeof(volume), "Vol %u", frame.settings.volume);
  const int volumeX = 156 - textWidth(body, volume);
  drawText(canvas, body, 4, 2, volumeX - 20, "GTS RADIO", Green, false, frame.now);
  if (frame.settings.muted) drawMuteIcon(canvas, volumeX - 16, 3, Yellow);
  drawText(canvas, body, volumeX, 2, 156 - volumeX, volume,
           frame.settings.muted ? Yellow : White, false, frame.now);

  // Use the displayed station's provider, independently of browsing filters.
  const bool selected = RadioModel::validStation(frame.station);
  const char* provider = !selected ? "No station selected" :
      frame.station.source == RadioModel::Source::Akashvani ? "Akashvani" : "Radio Browser";
  char context[64];
  std::snprintf(context, sizeof(context), "%s%s%s", provider,
                selected && frame.station.country[0] ? " / " : "",
                selected ? frame.station.country : "");
  animated |= drawText(canvas, body, 4, 22, 152, context, Green, true, motion.context.elapsed(context, frame.now));
  animated |= drawText(canvas, title, 4, 43, 152, frame.stationName, White, true,
                       motion.station.elapsed(frame.stationName, frame.now));

  const bool playing = frame.online && frame.playback.state == RadioAudio::State::Playing;
  const bool buffering = frame.online && frame.playback.state == RadioAudio::State::Buffering;
  if (playing && !frame.settings.muted)
    canvas.fillTriangle(5, 71, 5, 82, 13, 76, Green);
  else if (playing && frame.settings.muted) {
    canvas.fillRect(5, 72, 3, 10, Yellow);
    canvas.fillRect(11, 72, 3, 10, Yellow);
  }
  // Decoder selection alone is not playback confirmation. Do not display stale
  // technical values from the previous connection during a retry or failure.
  if (playing || buffering) {
    char technical[48] = {};
    const char* codec = frame.playback.codec;
    const bool knownCodec = codec[0] && std::strcmp(codec, "unknown") != 0;
    if (frame.playback.bitrateKbps)
      std::snprintf(technical, sizeof(technical), "%s%s%u kbps", knownCodec ? codec : "",
                    knownCodec ? " " : "", frame.playback.bitrateKbps);
    else if (knownCodec) std::snprintf(technical, sizeof(technical), "%s", codec);
    animated |= drawText(canvas, body, 21, 70, 135, technical, Cyan, true,
                         motion.technical.elapsed(technical, frame.now));
  }
  const uint16_t messageColor = frame.messageTone == RadioUi::Tone::Error ? Red :
      frame.messageTone == RadioUi::Tone::Warning ? Yellow :
      frame.messageTone == RadioUi::Tone::Info ? Cyan : White;
  animated |= drawText(canvas, body, 4, 92, 152, frame.messageText, messageColor, true,
                       motion.message.elapsed(frame.messageText, frame.now));

  canvas.drawFastHLine(0, 110, 160, 0x7bef);
  // A fixed hint: neither time-based alternation nor footer marquee is used.
  drawText(canvas, body, 4, 113, 152, "Click:mute Hold:menu", Cyan, false, frame.now);
  if (frame.buttonDown) {
    const uint32_t progress = frame.heldMs >= 700 ? 160 : frame.heldMs * 160 / 700;
    canvas.fillRect(0, 108, progress, 2, Green);
    animated = true;
  }
  return animated;
}
} // namespace RadioDisplay
