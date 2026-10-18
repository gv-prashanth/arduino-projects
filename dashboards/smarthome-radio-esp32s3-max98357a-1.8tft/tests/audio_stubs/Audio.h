#pragma once

#include "Arduino.h"

extern void audio_process_i2s(int16_t*, uint16_t, uint8_t, uint8_t, bool*);

class Audio {
 public:
  int nullStops = 0, rawStops = 0, connections = 0, frameBudget = 0;
  bool running = false;
  std::string lastUrl;

  bool setPinout(int, int, int) { return RadioAudioHost::pinoutReady; }
  void setConnectionTimeout(int, int) {}
  void setVolume(uint8_t) {}
  bool connecttohost(const char* host) {
    if (!host) { ++nullStops; running = false; return false; }
    ++connections;
    lastUrl = host;
    frameBudget = 1;
    running = true;
    return true;
  }
  void stopSong() { ++rawStops; running = false; }
  void loop() {
    if (running && (RadioAudioHost::continuousFrames || frameBudget > 0)) {
      bool forward = false;
      audio_process_i2s(nullptr, 100, 16, 2, &forward);
      assert(forward);
      if (frameBudget) --frameBudget;
    }
  }
  bool isRunning() { return running; }
  const char* getCodecname() { return "MP3"; }
  uint32_t getBitRate() { return 128000; }
};
