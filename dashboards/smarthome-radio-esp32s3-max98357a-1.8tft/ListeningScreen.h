#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include "RadioChrome.h"
#include "ListeningUi.h"

namespace RadioDisplay {
// Names/messages have already passed through the app's ASCII conversion.
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
    return age > 1500 ? age - 1500 : 0;
  }
};
struct ListeningMotion { RowMotion context, station, technical, message; };

namespace listening_detail {
inline uint16_t toneColor(RadioUi::Tone tone) {
  return tone == RadioUi::Tone::Error ? Red : tone == RadioUi::Tone::Warning ? Yellow :
         tone == RadioUi::Tone::Info ? Cyan : White;
}

inline const char* detailAfterStatus(const char* message, const char* status) {
  const std::size_t length = std::strlen(status);
  if (std::strncmp(message, status, length)) return message;
  if (!message[length]) return "";
  if (message[length] != ';' && message[length] != ':') return message;
  message += length + 1;
  while (*message == ' ') ++message;
  return message;
}

// Use natural word breaks first, then split long unbroken names. The second
// line's static truncation marker makes names longer than both lines apparent.
inline const GFXfont* titleFont(const char* name) {
  if (!name) return RadioFonts::Title;
  while (*name == ' ') ++name;
  // Avoid an orphaned final character for one long station/city name when the
  // next readable size can show it whole. Multiword titles keep two big rows.
  if (!std::strchr(name, ' ') && textWidth(*RadioFonts::Title, name) > 152 &&
      textWidth(*RadioFonts::Body, name) <= 152) return RadioFonts::Body;
  return RadioFonts::Title;
}

inline void titleLines(const char* name, char (&first)[128], char (&second)[128],
                       const GFXfont& font = *RadioFonts::Title) {
  if (!name) name = "";
  while (*name == ' ') ++name;
  if (textWidth(font, name) <= 152) { RadioModel::copyText(first, name); second[0] = '\0'; return; }
  std::size_t used = 0, space = 0;
  while (name[used] && used + 1 < sizeof(first)) {
    first[used] = name[used]; first[used + 1] = '\0';
    if (textWidth(font, first) > 152) break;
    if (name[used] == ' ') space = used;
    ++used;
  }
  const std::size_t split = space ? space : used ? used : 1;
  std::memcpy(first, name, split); first[split] = '\0';
  name += split;
  while (*name == ' ') ++name;
  RadioModel::copyText(second, name);
}
} // namespace listening_detail

template<class Canvas>
inline bool drawListeningScreen(Canvas& canvas, const ListeningFrame& frame, ListeningMotion& motion) {
  using RadioAudio::State;
  bool animated = false;
  brandHeader(canvas, frame.settings);

  // Provider and location belong to the displayed station, independently of
  // source/location filters used to browse a different catalog.
  const bool selected = RadioModel::validStation(frame.station);
  const char* provider = !selected ? "No station selected" :
      frame.station.source == RadioModel::Source::Akashvani ? "Akashvani" : "RB";
  const char* rawLocation = frame.station.location[0] ? frame.station.location : frame.station.country;
  const char* location = compactLocationLabel(rawLocation);
  char context[96];
  std::snprintf(context, sizeof(context), "%s%s%s", provider,
                selected && rawLocation[0] ? " / " : "", selected && rawLocation[0] ? location : "");
  drawText(canvas, *ChromeFont, 4, FilterTop, 152, context, Green, false, 0);

  char first[128] = {}, second[128] = {};
  const GFXfont& title = *listening_detail::titleFont(frame.stationName);
  listening_detail::titleLines(frame.stationName, first, second, title);
  drawText(canvas, title, 4, 39, 152, first, White, false, 0);
  drawText(canvas, title, 4, 39 + fontHeight(title), 152, second, White, false, 0);

  const State state = frame.playback.state;
  const bool offline = !frame.online || state == State::Offline;
  const bool playing = !offline && state == State::Playing;
  const bool waiting = !offline && (state == State::Connecting || state == State::Retrying || state == State::Buffering);
  const bool silent = frame.settings.muted || frame.settings.volume == 0;
  const Icon stateIcon = offline ? Icon::Offline : state == State::Failed ? Icon::Warning :
                         waiting ? Icon::Spinner : playing ? silent ? Icon::Muted : Icon::Play : Icon::Stations;
  const uint16_t stateColor = offline ? Yellow : state == State::Failed ? Red : waiting || silent ? Yellow : Green;
  drawIcon(canvas, stateIcon, 5, 80, 13, stateColor, frame.now);
  animated |= waiting;

  char technical[48] = {};
  const char* status = nullptr;
  const char* message = frame.messageText ? frame.messageText : "";
  uint16_t messageColor = listening_detail::toneColor(frame.messageTone);
  if (playing) {
    const char* codec = frame.playback.codec;
    const bool knownCodec = codec[0] && std::strcmp(codec, "unknown");
    if (frame.playback.bitrateKbps)
      std::snprintf(technical, sizeof(technical), "%s%s%u kbps", knownCodec ? codec : "",
                    knownCodec ? " / " : "", frame.playback.bitrateKbps);
    else if (knownCodec) std::snprintf(technical, sizeof(technical), "%s", codec);
    status = technical;
    // Volume zero is also silent. Preserve a warning over this routine hint.
    if (!frame.settings.muted && frame.settings.volume == 0 &&
        (!message[0] || frame.messageTone == RadioUi::Tone::Normal)) {
      message = "Volume 0"; messageColor = Yellow;
    }
  } else {
    status = offline ? "Wi-Fi offline" : state == State::Failed ? "Playback failed" :
             state == State::Connecting ? "Connecting" : state == State::Retrying ? "Retrying audio" :
             state == State::Buffering ? "Buffering" : "Choose a station";
    // Short actionable progress belongs in the status row. Longer reasons
    // retain the status label and use the separate programme/message row.
    if (waiting && message[0] && textWidth(*RowFont, message) <= 133) status = message;
    message = listening_detail::detailAfterStatus(message, status);
  }
  drawText(canvas, *RowFont, 23, 80, 133, status, playing ? Cyan : stateColor, false, 0);
  animated |= drawText(canvas, *RowFont, 4, 97, 152, message, messageColor, true,
                       motion.message.elapsed(message, frame.now));

  footer(canvas, Icon::Stations, "Stations", Icon::Settings, "Settings");
  if (frame.buttonDown) {
    const uint32_t progress = frame.heldMs >= 700 ? 160 : frame.heldMs * 160 / 700;
    canvas.fillRect(0, 126, progress, 2, Green);
    animated = true;
  }
  return animated;
}
} // namespace RadioDisplay
