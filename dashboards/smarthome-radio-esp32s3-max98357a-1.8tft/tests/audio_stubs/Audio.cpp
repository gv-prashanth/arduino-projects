#include "Audio.h"

#if RADIO_TEST_AUDIO_API == 3
// Audio 4.0.0 supplies a weak no-op body in its own TU. The application's
// separate strong definition must win even though its Audio.h declaration is weak.
void __attribute__((weak)) audio_process_i2s(int32_t*, int16_t, bool* continueI2S) {
  ++RadioAudioHost::weakDefaultCalls;
  *continueI2S = true;
}

void Audio::emitEvent(event_t event, const char* text) {
  if (!audio_info_callback) return;
  msg_t message;
  message.e = event;
  message.msg = text;
  audio_info_callback(message);
}
#endif

void Audio::emitTitle(const char* text) {
#if RADIO_TEST_AUDIO_API == 3
  emitEvent(evt_streamtitle, text);
#else
  if (audio_showstreamtitle) audio_showstreamtitle(text);
#endif
}
void Audio::emitBitrate(const char* text) {
#if RADIO_TEST_AUDIO_API == 3
  emitEvent(evt_bitrate, text);
#else
  if (audio_bitrate) audio_bitrate(text);
#endif
}
void Audio::emitInfo(const char* text) {
#if RADIO_TEST_AUDIO_API == 3
  emitEvent(evt_info, text);
#else
  if (audio_info) audio_info(text);
#endif
}
void Audio::emitLog(const char* text) {
#if RADIO_TEST_AUDIO_API == 3
  emitEvent(evt_log, text);
#else
  emitInfo(text);
#endif
}
void Audio::emitEof(const char* text) {
#if RADIO_TEST_AUDIO_API == 3
  emitEvent(evt_eof, text);
#else
  emitInfo(text);
#endif
}

void Audio::loop() {
#if RADIO_TEST_AUDIO_API == 3
  // v4 delivers its internally queued messages before checking m_f_running.
  // stopSong() deliberately leaves this queue intact, matching that library.
  while (!deferredTitles.empty()) {
    const std::string title = deferredTitles.front();
    deferredTitles.pop_front();
    ++deliveredDeferredTitles;
    emitTitle(title.c_str());
  }
#endif
  if (!running) return;
  if (startupMetadataPending && RadioAudioHost::emitStartupMetadata) {
    startupMetadataPending = false;
    emitTitle("Library title");
    emitBitrate("128000");
    emitInfo("Connection established");
  }
  if (!RadioAudioHost::framesEnabled ||
      (!RadioAudioHost::continuousFrames && frameBudget <= 0)) return;
  ++pcmCallbacks;
  if (RadioAudioHost::frameSamples > 0) ++positivePcmCallbacks;
  const bool silent = RadioAudioHost::silentPcm || volumeValue == 0;
  if (silent) ++silentPcmCallbacks;
  bool forward = false;
#if RADIO_TEST_AUDIO_API == 3
  int32_t pcm[2] = {silent ? 0 : 123456, silent ? 0 : -123456};
  if (audio_process_i2s)
    audio_process_i2s(pcm, static_cast<int16_t>(RadioAudioHost::frameSamples), &forward);
#else
  int16_t pcm[2] = {static_cast<int16_t>(silent ? 0 : 123), static_cast<int16_t>(silent ? 0 : -123)};
#if RADIO_TEST_AUDIO_API == 0
  if (audio_process_i2s)
    audio_process_i2s(pcm, static_cast<uint16_t>(RadioAudioHost::frameSamples), 16, 2, &forward);
#elif RADIO_TEST_AUDIO_API == 1
  if (audio_process_i2s)
    audio_process_i2s(pcm, static_cast<uint16_t>(RadioAudioHost::frameSamples), &forward);
#else
  if (audio_process_i2s) audio_process_i2s(pcm, RadioAudioHost::frameSamples, &forward);
#endif
#endif
  assert(forward);
  if (frameBudget) --frameBudget;
}
