#include "../ListeningUi.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <type_traits>

using RadioAudio::Snapshot;
using RadioAudio::State;
using RadioUi::Message;
using RadioUi::NoticeCenter;
using RadioUi::NoticePriority;
using RadioUi::Tone;

static_assert(std::is_trivially_copyable<Snapshot>::value, "Snapshots must remain queue-safe values");

static Snapshot playback(State state, const char* detail = "", const char* title = "") {
  Snapshot snapshot;
  snapshot.state = state;
  assert(RadioModel::copyText(snapshot.detail, detail));
  assert(RadioModel::copyText(snapshot.title, title));
  return snapshot;
}

static void healthyPlaybackHasNoRedundantStatus() {
  NoticeCenter notices;
  Snapshot snapshot = playback(State::Playing, "Decoded audio ready");
  Message message = RadioUi::messageFor(snapshot, true, "Wi-Fi connected", false, notices);
  assert(!message.text[0] && !message.usesNotice && !message.usesFallback);
  assert(message.tone == Tone::Normal);
  assert(RadioModel::copyText(snapshot.title, "Artist - Track title"));
  message = RadioUi::messageFor(snapshot, true, "Wi-Fi connected", false, notices);
  assert(!std::strcmp(message.text, snapshot.title));
  assert(message.tone == Tone::Normal && !message.usesNotice && !message.usesFallback);
  // The worker's fallback detail remains in snapshots. The UI controls how
  // long the meaningful restoration notice should actually be visible.
  assert(RadioModel::copyText(snapshot.detail, "Previous station restored"));
  message = RadioUi::messageFor(snapshot, true, "", false, notices, true);
  assert(!std::strcmp(message.text, "Previous station restored"));
  assert(message.usesFallback && !message.usesNotice && message.tone == Tone::Info);
  message = RadioUi::messageFor(snapshot, true, "", false, notices, false);
  assert(!std::strcmp(message.text, snapshot.title) && !message.usesFallback);
  snapshot.title[0] = '\0';
  message = RadioUi::messageFor(snapshot, true, "", false, notices);
  assert(!message.text[0]);
}

static void staleMetadataCannotHidePlaybackState() {
  const struct { State state; const char* expected; Tone tone; } cases[] = {
      {State::Idle, "Choose a station", Tone::Info},
      {State::Connecting, "Connecting", Tone::Info},
      {State::Retrying, "Retrying audio", Tone::Warning},
      {State::Buffering, "Buffering", Tone::Warning},
      {State::Failed, "Playback failed; Menu > Retry audio", Tone::Error},
      {State::Offline, "Wi-Fi offline; reconnecting", Tone::Warning}};
  for (const auto& example : cases) {
    NoticeCenter notices;
    notices.post("Stations ready", 5000, 0);
    Snapshot snapshot = playback(example.state, "Decoded audio ready", "Old station's track");
    assert(RadioModel::copyText(snapshot.codec, "AAC"));
    snapshot.bitrateKbps = 128;
    const Message message = RadioUi::messageFor(snapshot, true, "Wi-Fi connected", false, notices, true);
    assert(!std::strcmp(message.text, example.expected));
    assert(message.tone == example.tone && !message.usesNotice && !message.usesFallback);
    assert(std::strstr(message.text, "Decoded audio ready") == nullptr);
    assert(std::strstr(message.text, "Old station") == nullptr);
  }
}

static void specificProgressAndFailureRemainActionable() {
  NoticeCenter notices;
  notices.post("Settings write failed", 10000, 0, NoticePriority::Warning);
  const struct { State state; const char* detail; const char* expected; } cases[] = {
      {State::Connecting, "Connection attempt 2/3", "Connection attempt 2/3"},
      {State::Connecting, "Connecting", "Connecting"},
      {State::Retrying, "Audio stalled; retry 2/3", "Audio stalled; retry 2/3"},
      {State::Retrying, "Starting selected station", "Starting selected station"},
      {State::Retrying, "Returning to previous station", "Returning to previous station"},
      {State::Buffering, "Waiting for audio data", "Waiting for audio data"},
      {State::Failed, "Open menu > Retry audio", "Playback failed; Open menu > Retry audio"},
      {State::Failed, "No decodable audio", "Playback failed; No decodable audio; Menu > Retry audio"},
      {State::Failed, "Playback failed", "Playback failed; Menu > Retry audio"},
      {State::Failed, "Playback failed; try again", "Playback failed; try again; Menu > Retry audio"}};
  for (const auto& example : cases) {
    const Snapshot snapshot = playback(example.state, example.detail, "Stale metadata");
    const Message message = RadioUi::messageFor(snapshot, true, "", true, notices, true);
    assert(!std::strcmp(message.text, example.expected));
    assert(!message.usesNotice && !message.usesFallback);
  }
}

static void priorityIsIndependentOfMuteAndMetadata() {
  NoticeCenter notices;
  notices.post("Stations ready", 5000, 0);
  notices.post("Settings write failed", 10000, 0, NoticePriority::Warning);
  Snapshot snapshot = playback(State::Failed, "Open menu > Retry audio", "Old title");
  Message message = RadioUi::messageFor(snapshot, false, "Wi-Fi lost; reconnecting", true, notices, true);
  assert(!std::strcmp(message.text, "Wi-Fi lost; reconnecting"));
  assert(!message.usesNotice && !message.usesFallback);
  message = RadioUi::messageFor(snapshot, false, nullptr, true, notices);
  assert(!std::strcmp(message.text, "Wi-Fi offline; reconnecting"));
  message = RadioUi::messageFor(snapshot, true, "", true, notices);
  assert(std::strstr(message.text, "Playback failed") == message.text);
  assert(message.tone == Tone::Error && !message.usesNotice);
  snapshot.state = State::Playing;
  message = RadioUi::messageFor(snapshot, true, "", true, notices, true);
  assert(!std::strcmp(message.text, "Settings write failed"));
  assert(message.usesNotice && !message.usesFallback && message.tone == Tone::Warning);
  assert(!notices.update(0, true));
  assert(notices.update(10000, true));
  message = RadioUi::messageFor(snapshot, true, "", true, notices, true);
  assert(!std::strcmp(message.text, "Muted"));
  assert(!message.usesNotice && !message.usesFallback);
  message = RadioUi::messageFor(snapshot, true, "", false, notices, true);
  assert(message.usesFallback && !message.usesNotice);
  notices.post("Wi-Fi connected", 5000, 10000);
  message = RadioUi::messageFor(snapshot, true, "", true, notices);
  assert(!std::strcmp(message.text, "Muted") && !message.usesNotice);
  message = RadioUi::messageFor(snapshot, true, "", false, notices, true);
  assert(message.usesFallback && !message.usesNotice);
  message = RadioUi::messageFor(snapshot, true, "", false, notices);
  assert(!std::strcmp(message.text, "Wi-Fi connected") && message.usesNotice && message.tone == Tone::Info);
}

static void idleKeepsItsPromptUnlessAWarningNeedsAttention() {
  const Snapshot snapshot = playback(State::Idle, "", "Stale track");
  NoticeCenter notices;
  notices.post("Stations ready", 5000, 0);
  Message message = RadioUi::messageFor(snapshot, true, "", true, notices, true);
  assert(!std::strcmp(message.text, "Choose a station") && !message.usesNotice);
  notices.post("Cannot queue station search", 5000, 0, NoticePriority::Warning);
  message = RadioUi::messageFor(snapshot, true, "", true, notices, true);
  assert(!std::strcmp(message.text, "Cannot queue station search") && message.usesNotice);
}

static void warningsHaveIndependentVisibleTime() {
  NoticeCenter notices;
  notices.post("Settings write failed", 5000, 100, NoticePriority::Warning);
  notices.post("Stations ready", 10000, 200);
  assert(notices.active() && notices.warning() && !std::strcmp(notices.message(), "Settings write failed"));
  assert(!notices.update(10000, false));
  assert(notices.warning()); // Hidden warning has consumed none of its five seconds.
  assert(notices.update(11000, true)); // The background info TTL elapsed instead.
  assert(notices.warning() && !std::strcmp(notices.message(), "Settings write failed"));
  assert(!notices.update(12000, true));
  notices.post("Wi-Fi connected", 10000, 12000);
  assert(!notices.update(15999, true));
  assert(notices.warning());
  assert(notices.update(16000, true));
  assert(!notices.warning() && notices.active());
  assert(!std::strcmp(notices.message(), "Wi-Fi connected"));
  assert(notices.update(22000, true));
  assert(!notices.active() && !notices.message()[0]);
  assert(!notices.update(23000, false));
}

static void criticalStatesAndOtherViewsPausePendingWarnings() {
  NoticeCenter notices;
  notices.post("Settings write failed", 4000, 100, NoticePriority::Warning);
  const Snapshot playing = playback(State::Playing);
  const Snapshot retrying = playback(State::Retrying, "Audio stalled; retry 2/3", "Stale title");
  Message message = RadioUi::messageFor(playing, true, "", false, notices);
  assert(message.usesNotice && !notices.update(100, message.usesNotice));
  message = RadioUi::messageFor(retrying, true, "", false, notices);
  assert(!message.usesNotice && !notices.update(1100, message.usesNotice));
  assert(!notices.update(100000, false)); // Includes time on another screen.
  message = RadioUi::messageFor(playing, true, "", true, notices);
  assert(message.usesNotice); // Warnings remain visible above a muted status.
  assert(!notices.update(100100, message.usesNotice));
  assert(!notices.update(104099, true));
  assert(notices.update(104100, true));
  assert(!notices.active());
}

static void restorationCanUseTheSameVisibleTimeMechanism() {
  NoticeCenter notices, restoration;
  const Snapshot snapshot = playback(State::Playing, "Previous station restored", "Current track");
  notices.post("Settings write failed", 1000, 0, NoticePriority::Warning);
  restoration.post("Previous station restored", 5000, 0, NoticePriority::Warning);
  Message message = RadioUi::messageFor(snapshot, true, "", false, notices, restoration.active());
  assert(message.usesNotice && !message.usesFallback);
  assert(!notices.update(0, true) && !restoration.update(0, message.usesFallback));
  assert(notices.update(1000, true));
  assert(!restoration.update(1000, false));
  message = RadioUi::messageFor(snapshot, true, "", true, notices, restoration.active());
  assert(!message.usesFallback && !restoration.update(100000, message.usesFallback));
  message = RadioUi::messageFor(snapshot, true, "", false, notices, restoration.active());
  assert(message.usesFallback && !restoration.update(100100, true));
  assert(!restoration.update(105099, true));
  assert(restoration.update(105100, true));
  message = RadioUi::messageFor(snapshot, true, "", false, notices, restoration.active());
  assert(!std::strcmp(message.text, "Current track") && !message.usesFallback);
}

static void noticeTimersSurviveMillisWraparound() {
  const std::uint32_t maximum = std::numeric_limits<std::uint32_t>::max();
  NoticeCenter info;
  info.post("Wi-Fi connected", 30, maximum - 10);
  assert(!info.update(5, false) && info.active());
  assert(info.update(19, false) && !info.active());
  NoticeCenter warnings;
  warnings.post("Settings write failed", 40, maximum - 20, NoticePriority::Warning);
  assert(!warnings.update(maximum - 20, true));
  assert(!warnings.update(maximum - 10, false)); // Interruption restores the full budget.
  assert(!warnings.update(100, false)); // Hidden across the clock rollover.
  assert(!warnings.update(200, true));
  assert(!warnings.update(239, true));
  assert(warnings.update(240, true) && !warnings.active());
}

static void interruptedWarningsGetACompleteReadingBudget() {
  NoticeCenter notices;
  notices.post("A long warning restarting its marquee", 1000, 100, NoticePriority::Warning);
  assert(!notices.update(100, true));
  assert(!notices.update(600, false)); // Half was read, then a menu hid it.
  assert(!notices.update(60000, true));
  assert(!notices.update(60999, true)); // The resumed display receives all 1000 ms.
  assert(notices.active());
  assert(notices.update(61000, false)); // Expiry wins over this interruption.
  assert(!notices.active());
  assert(!notices.update(100000, true)); // It must not be resurrected.
}

static void warningBudgetsCoverActualMarqueeTravel() {
  using RadioUi::warningReadDuration;
  assert(warningReadDuration(0, 1000) == 0);
  assert(warningReadDuration(5000, 0) == 5000);
  assert(warningReadDuration(1000, -10) == 1000);
  assert(warningReadDuration(1000, 152) == 1000);
  assert(warningReadDuration(1000, 153) == 3040);
  assert(warningReadDuration(5000, 200) == 5000);
  assert(warningReadDuration(5000, 252) == 7000);
  assert(warningReadDuration(5000, 300, 200) == 7000);
  const std::uint32_t maximum = std::numeric_limits<std::uint32_t>::max();
  assert(warningReadDuration(maximum, 153) == maximum);
  assert(warningReadDuration(1000, std::numeric_limits<int>::max()) == maximum);
  assert(warningReadDuration(1000, std::numeric_limits<int>::max(), std::numeric_limits<int>::min()) == maximum);
}

static void noticeTextIsOwnedAndBounded() {
  NoticeCenter notices;
  char temporary[] = "Original warning";
  notices.post(temporary, 100, 0, NoticePriority::Warning);
  std::memset(temporary, 'x', sizeof(temporary) - 1);
  assert(!std::strcmp(notices.message(), "Original warning"));
  notices.post("", 100, 0, NoticePriority::Warning);
  notices.post(nullptr, 100, 0, NoticePriority::Warning);
  assert(!std::strcmp(notices.message(), "Original warning"));
  const std::string longText(1000, 'x');
  notices.post(longText.c_str(), 100, 0, NoticePriority::Warning);
  const Message message = RadioUi::messageFor(playback(State::Playing), true, "", false, notices);
  assert(message.usesNotice && std::strlen(message.text) == sizeof(message.text) - 1);
  assert(message.text[sizeof(message.text) - 1] == '\0');
  notices.post("Expires immediately", 0, 0, NoticePriority::Warning);
  assert(!notices.active());
}

int main() {
  healthyPlaybackHasNoRedundantStatus();
  staleMetadataCannotHidePlaybackState();
  specificProgressAndFailureRemainActionable();
  priorityIsIndependentOfMuteAndMetadata();
  idleKeepsItsPromptUnlessAWarningNeedsAttention();
  warningsHaveIndependentVisibleTime();
  criticalStatesAndOtherViewsPausePendingWarnings();
  restorationCanUseTheSameVisibleTimeMechanism();
  noticeTimersSurviveMillisWraparound();
  interruptedWarningsGetACompleteReadingBudget();
  warningBudgetsCoverActualMarqueeTravel();
  noticeTextIsOwnedAndBounded();
  std::puts("Listening UI tests passed");
}
