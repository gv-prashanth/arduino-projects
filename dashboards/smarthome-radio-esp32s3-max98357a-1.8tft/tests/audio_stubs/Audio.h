#pragma once

#include "Arduino.h"
#include <functional>

#ifndef RADIO_TEST_AUDIO_API
#define RADIO_TEST_AUDIO_API 0
#endif
#if RADIO_TEST_AUDIO_API == 0
extern __attribute__((weak)) void audio_process_i2s(int16_t*, uint16_t, uint8_t, uint8_t, bool*);
#elif RADIO_TEST_AUDIO_API == 1
extern __attribute__((weak)) void audio_process_i2s(int16_t*, uint16_t, bool*);
#elif RADIO_TEST_AUDIO_API == 2
extern __attribute__((weak)) void audio_process_i2s(int16_t*, int32_t, bool*);
#elif RADIO_TEST_AUDIO_API == 3
extern __attribute__((weak)) void audio_process_i2s(int32_t*, int16_t, bool*);
#elif RADIO_TEST_AUDIO_API == 99
extern __attribute__((weak)) void audio_process_i2s(int32_t*, uint32_t, bool*);
#else
#error Unsupported RADIO_TEST_AUDIO_API test mode
#endif
extern __attribute__((weak)) void audio_showstreamtitle(const char*);
extern __attribute__((weak)) void audio_bitrate(const char*);
extern __attribute__((weak)) void audio_info(const char*);

class Audio {
 public:
#if RADIO_TEST_AUDIO_API == 3
  // Match the ESP32-audioI2S 4.0.0 dispatcher surface and event ordering.
  enum event_t {
    evt_info = 0, evt_id3data, evt_eof, evt_name, evt_icydescription,
    evt_streamtitle, evt_bitrate, evt_icyurl, evt_icylogo, evt_genre,
    evt_lasthost, evt_image, evt_lyrics, evt_log, evt_vu, evt_spectrum
  };
  struct msg_t {
    const char* msg = nullptr;
    const char* s = nullptr;
    event_t e = evt_info;
    int32_t i2s_num = 0, arg1 = 0, arg2 = 0;
    std::vector<uint32_t> vec1, vec2;
  };
  inline static std::function<void(msg_t)> audio_info_callback;
  static void emitEvent(event_t event, const char* text);
  std::deque<std::string> deferredTitles;
  unsigned deliveredDeferredTitles = 0;
  void deferTitle(const char* text) { deferredTitles.emplace_back(text); }
#endif
  int nullStops = 0, rawStops = 0, connections = 0, frameBudget = 0;
  unsigned pcmCallbacks = 0, positivePcmCallbacks = 0, silentPcmCallbacks = 0;
  bool running = false, startupMetadataPending = false;
  uint8_t volumeValue = 0;
  std::string lastUrl;

  bool setPinout(int, int, int) { return RadioAudioHost::pinoutReady; }
  void setConnectionTimeout(int, int) {}
  void setVolume(uint8_t volume) { volumeValue = volume; }
  bool connecttohost(const char* host) {
    if (!host) { ++nullStops; running = false; return false; }
    ++connections;
    lastUrl = host;
    frameBudget = 1;
    running = true;
    startupMetadataPending = true;
    return true;
  }
  void stopSong() { ++rawStops; running = false; }
  // Implemented in a separate library-like TU, independently of AudioService.
  void loop();
  static void emitTitle(const char* text);
  static void emitBitrate(const char* text);
  static void emitInfo(const char* text);
  static void emitLog(const char* text);
  static void emitEof(const char* text);
  bool isRunning() { return running; }
  const char* getCodecname() { return "MP3"; }
  uint32_t getBitRate() { return 128000; }
};
