#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include "AudioState.h"

namespace RadioUi {

enum class NoticePriority : std::uint8_t { Info, Warning };
enum class Tone : std::uint8_t { Normal, Info, Warning, Error };

// Overflowing text needs an initial dwell, enough 25 px/s marquee travel to
// reveal its end, and a trailing dwell. Short rows retain their requested TTL.
inline std::uint32_t warningReadDuration(std::uint32_t requested, int pixelWidth, int rowWidth = 152) {
  if (!requested || pixelWidth <= rowWidth) return requested;
  const std::uint64_t overflow = static_cast<std::uint64_t>(static_cast<std::int64_t>(pixelWidth) - rowWidth);
  const std::uint64_t minimum = 1500u + overflow * 40u + 1500u;
  const std::uint64_t limit = std::numeric_limits<std::uint32_t>::max();
  const std::uint32_t bounded = static_cast<std::uint32_t>(minimum > limit ? limit : minimum);
  return requested > bounded ? requested : bounded;
}

namespace detail {
template<std::size_t Capacity>
inline void copyText(char (&out)[Capacity], const char* text) {
  std::size_t used = 0;
  if (text) while (used + 1 < Capacity && text[used]) { out[used] = text[used]; ++used; }
  out[used] = '\0';
}
inline bool usefulDetail(const char* text) {
  return text && text[0] && std::strcmp(text, "Decoded audio ready") && std::strcmp(text, "Playing");
}
}  // namespace detail

// Routine success messages have a wall-clock TTL. Warnings have a separate
// slot and consume their TTL only while the caller says the message is visible.
// Interrupting a visible warning restores its full reading budget, because the
// renderer restarts that message's marquee when it becomes visible again.
// update() charges the interval since its previous visibility sample first, so
// an already expired warning is never revived by a change of visibility.
class NoticeCenter {
 public:
  void post(const char* text, std::uint32_t durationMs, std::uint32_t now,
            NoticePriority priority = NoticePriority::Info) {
    if (!text || !text[0]) return;
    Slot& slot = priority == NoticePriority::Warning ? warning_ : info_;
    detail::copyText(slot.text, text);
    slot.active = durationMs != 0;
    slot.remaining = slot.duration = durationMs;
    slot.at = now;
    slot.visible = false;
  }

  const char* message() const { return warning_.active ? warning_.text : info_.active ? info_.text : ""; }
  bool warning() const { return warning_.active; }
  bool active() const { return warning_.active || info_.active; }

  bool update(std::uint32_t now, bool visible) {
    bool expired = false;
    if (info_.active && static_cast<std::uint32_t>(now - info_.at) >= info_.remaining) {
      info_.active = false;
      expired = true;
    }
    if (warning_.active) {
      const std::uint32_t elapsed = now - warning_.at;
      if (warning_.visible) {
        if (elapsed >= warning_.remaining) {
          warning_.active = false;
          expired = true;
        } else warning_.remaining -= elapsed;
      }
      if (warning_.active && warning_.visible && !visible) warning_.remaining = warning_.duration;
      warning_.at = now;
      warning_.visible = visible && warning_.active;
    }
    return expired;
  }

 private:
  struct Slot {
    char text[192] = {};
    std::uint32_t at = 0, remaining = 0, duration = 0;
    bool active = false, visible = false;
  };
  Slot warning_, info_;
};

struct Message {
  char text[192] = {};
  Tone tone = Tone::Normal;
  bool usesNotice = false;
  bool usesFallback = false;
};

inline Message messageFor(const RadioAudio::Snapshot& playback, bool online, const char* wifiStatus,
                          bool muted, const NoticeCenter& notices, bool fallbackVisible = false) {
  Message message;
  using RadioAudio::State;
  if (!online || playback.state == State::Offline) {
    const char* status = !online ? wifiStatus : detail::usefulDetail(playback.detail) ? playback.detail : nullptr;
    detail::copyText(message.text, status && status[0] ? status : "Wi-Fi offline; reconnecting");
    message.tone = Tone::Warning;
    return message;
  }
  if (playback.state == State::Failed) {
    if (detail::usefulDetail(playback.detail) && std::strcmp(playback.detail, "Playback failed")) {
      const bool hasPrefix = std::strncmp(playback.detail, "Playback failed", sizeof("Playback failed") - 1) == 0;
      const bool hasAction = std::strstr(playback.detail, "Retry audio") != nullptr;
      std::snprintf(message.text, sizeof(message.text), "%s%s%s", hasPrefix ? "" : "Playback failed; ",
                    playback.detail, hasAction ? "" : "; Menu > Retry audio");
    } else detail::copyText(message.text, "Playback failed; Menu > Retry audio");
    message.tone = Tone::Error;
    return message;
  }
  if (playback.state == State::Connecting || playback.state == State::Retrying || playback.state == State::Buffering) {
    const char* fallback = playback.state == State::Connecting ? "Connecting" :
                           playback.state == State::Retrying ? "Retrying audio" : "Buffering";
    detail::copyText(message.text, detail::usefulDetail(playback.detail) ? playback.detail : fallback);
    message.tone = playback.state == State::Connecting ? Tone::Info : Tone::Warning;
    return message;
  }
  if (notices.warning()) {
    detail::copyText(message.text, notices.message());
    message.tone = Tone::Warning;
    message.usesNotice = true;
    return message;
  }
  if (playback.state == State::Idle) {
    detail::copyText(message.text, "Choose a station");
    message.tone = Tone::Info;
    return message;
  }
  if (muted) {
    detail::copyText(message.text, "Muted");
    message.tone = Tone::Info;
    return message;
  }
  if (fallbackVisible) {
    detail::copyText(message.text, "Previous station restored");
    message.tone = Tone::Info;
    message.usesFallback = true;
    return message;
  }
  if (notices.active()) {
    detail::copyText(message.text, notices.message());
    message.tone = Tone::Info;
    message.usesNotice = true;
    return message;
  }
  detail::copyText(message.text, playback.title);
  return message;
}

}  // namespace RadioUi
