#include "AudioCallbacks.h"

// Deliberately do not include Audio.h here: its weak declarations would make
// our definitions weak too. In 4.0.0 the library supplies a weak default body;
// these strong definitions must override it regardless of archive link order.
void audio_showstreamtitle(const char* info) { RadioAudio::metadata(RadioAudio::MetadataType::Title, info); }
void audio_bitrate(const char* info) { RadioAudio::metadata(RadioAudio::MetadataType::Bitrate, info); }
void audio_info(const char* info) { RadioAudio::metadata(RadioAudio::MetadataType::Info, info); }

// ESP32-audioI2S has used four PCM hook signatures. Only the signature declared
// by the installed Audio.h is called; every supported hook forwards I2S output.
void audio_process_i2s(int16_t*, uint16_t samples, uint8_t, uint8_t, bool* continueI2S) {
  *continueI2S = true;
  RadioAudio::decodedFrame(samples);
}
void audio_process_i2s(int16_t*, uint16_t samples, bool* continueI2S) {
  *continueI2S = true;
  RadioAudio::decodedFrame(samples);
}
void audio_process_i2s(int16_t*, int32_t samples, bool* continueI2S) {
  *continueI2S = true;
  RadioAudio::decodedFrame(samples);
}
void audio_process_i2s(int32_t*, int16_t samples, bool* continueI2S) {
  *continueI2S = true;
  RadioAudio::decodedFrame(samples);
}
