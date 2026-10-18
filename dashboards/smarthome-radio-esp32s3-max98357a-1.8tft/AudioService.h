#pragma once

#include <Arduino.h>
#include <Audio.h>
#include <type_traits>
#include "AudioCallbacks.h"
#include "RadioConfig.h"

namespace RadioAudio {
using RadioModel::Station;
// Capture the library's declaration before introducing any overloads. A future
// incompatible API must fail at compile time instead of silently losing PCM
// confirmation while otherwise playing audio normally.
using PcmCallback = decltype(&::audio_process_i2s);
using LegacyPcmCallback = void (*)(int16_t*, uint16_t, uint8_t, uint8_t, bool*);
using ThreeArgPcmCallback = void (*)(int16_t*, uint16_t, bool*);
using WideCountPcmCallback = void (*)(int16_t*, int32_t, bool*);
using Pcm32Callback = void (*)(int32_t*, int16_t, bool*);
static_assert(std::is_same<PcmCallback, LegacyPcmCallback>::value ||
              std::is_same<PcmCallback, ThreeArgPcmCallback>::value ||
              std::is_same<PcmCallback, WideCountPcmCallback>::value ||
              std::is_same<PcmCallback, Pcm32Callback>::value,
              "Unsupported ESP32-audioI2S PCM callback API; update AudioCallbacks.cpp");
inline const char* pcmApiName() {
  return std::is_same<PcmCallback, Pcm32Callback>::value ? "32-bit PCM (3-argument)" :
         std::is_same<PcmCallback, LegacyPcmCallback>::value ? "16-bit PCM (legacy 5-argument)" :
         "16-bit PCM (3-argument)";
}
enum class State : uint8_t { Idle, Connecting, Playing, Buffering, Retrying, Failed, Offline };
struct StartRequest { Station station, fallback; uint32_t generation = 0; };
struct Snapshot {
  State state = State::Idle;
  bool fatal = false;
  Station requested, confirmed;
  uint32_t generation = 0;
  uint16_t bitrateKbps = 0;
  char codec[12] = {}, title[192] = {}, detail[80] = {};
};
struct Metadata { MetadataType type; uint32_t generation, epoch; char text[192]; };
static QueueHandle_t metadataQueue = nullptr;
static portMUX_TYPE signalMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t activeGeneration = 0, decodedGeneration = 0, lastFrameMs = 0, activeMetadataEpoch = 0;

void metadata(MetadataType type, const char* text) {
  if (!metadataQueue || !text) return;
  Metadata event{};
  event.type = type;
  portENTER_CRITICAL(&signalMux);
  event.generation = activeGeneration;
  event.epoch = activeMetadataEpoch;
  portEXIT_CRITICAL(&signalMux);
  strlcpy(event.text, text, sizeof(event.text));
  xQueueSend(metadataQueue, &event, 0); // Telemetry may be dropped; playback confirmation uses its own signal.
}
void decodedFrame(int32_t samples) {
  if (samples <= 0) return;
  portENTER_CRITICAL(&signalMux);
  if (activeGeneration) { decodedGeneration = activeGeneration; lastFrameMs = millis(); }
  portEXIT_CRITICAL(&signalMux);
}

// Older versions call the global hooks in AudioCallbacks.cpp; 3.4.2 and later
// require explicit event callback registration. Detect that member rather than
// a version string (several releases report stale package versions).
template <typename AudioType, typename = void>
struct MetadataCallbacks {
  static constexpr bool modern = false;
  static void install() {}
};
template <typename AudioType>
struct MetadataCallbacks<AudioType, decltype((void)AudioType::audio_info_callback)> {
  static constexpr bool modern = true;
  static void dispatch(typename AudioType::msg_t event) {
    if (event.e == AudioType::evt_streamtitle) metadata(MetadataType::Title, event.msg);
    else if (event.e == AudioType::evt_bitrate) metadata(MetadataType::Bitrate, event.msg);
    else if (event.e == AudioType::evt_info || event.e == AudioType::evt_log || event.e == AudioType::evt_eof)
      metadata(MetadataType::Info, event.msg);
  }
  static void install() { AudioType::audio_info_callback = dispatch; }
};
inline const char* stateName(State state) {
  switch (state) {
    case State::Idle: return "Choose a station";
    case State::Connecting: return "Connecting";
    case State::Playing: return "Playing";
    case State::Buffering: return "Buffering";
    case State::Retrying: return "Retrying audio";
    case State::Failed: return "Playback failed";
    case State::Offline: return "Wi-Fi offline";
  }
  return "";
}

class AudioService {
 public:
  bool begin() {
    starts_ = xQueueCreate(1, sizeof(StartRequest));
    volumes_ = xQueueCreate(1, sizeof(uint8_t));
    networks_ = xQueueCreate(1, sizeof(bool));
    snapshots_ = xQueueCreate(1, sizeof(Snapshot));
    metadataQueue = xQueueCreate(16, sizeof(Metadata));
    if (!starts_ || !volumes_ || !networks_ || !snapshots_ || !metadataQueue) return false;
    return xTaskCreatePinnedToCore(taskEntry, "audio-owner", 8192, this, 2, &task_, 1) == pdPASS;
  }
  bool start(const Station& station, const Station& fallback, uint32_t generation) {
    if (!starts_ || !RadioModel::validStation(station)) return false;
    StartRequest request{station, fallback, generation};
    return xQueueOverwrite(starts_, &request) == pdTRUE;
  }
  void volume(uint8_t volume) { if (volumes_) xQueueOverwrite(volumes_, &volume); }
  void network(bool online) { if (networks_) xQueueOverwrite(networks_, &online); }
  bool poll(Snapshot& snapshot) { return snapshots_ && xQueueReceive(snapshots_, &snapshot, 0) == pdTRUE; }

 private:
  friend struct AudioServiceTestAccess;
  // Only the owner task calls this Audio instance's public API, including getters.
  Audio audio_;
  QueueHandle_t starts_ = nullptr, volumes_ = nullptr, networks_ = nullptr, snapshots_ = nullptr;
  TaskHandle_t task_ = nullptr;
  Snapshot snapshot_;
  Station desired_, fallback_;
  bool online_ = false, connecting_ = false, fallbackUsed_ = false, stablePlaying_ = false;
  uint8_t attempts_ = 0;
  uint32_t connectAt_ = 0, retryAt_ = 0, publishedAt_ = 0, metadataAt_ = 0;
  uint32_t playingSince_ = 0, metadataEpoch_ = 0;

  static void taskEntry(void* self) { static_cast<AudioService*>(self)->run(); }
  void publish() { xQueueOverwrite(snapshots_, &snapshot_); publishedAt_ = millis(); }
  void deactivate() {
    portENTER_CRITICAL(&signalMux);
    activeGeneration = decodedGeneration = lastFrameMs = activeMetadataEpoch = 0;
    portEXIT_CRITICAL(&signalMux);
  }
  void stopPlayback() {
    // In pinned audioI2S 928c420, raw stopSong() does not lock the decoder
    // mutex. The public null-host path acquires that mutex before stopSong(),
    // keeping buffer clearing serialized with the library's decoder task.
    // The 32-bit-PCM releases take their own decoder mutex in stopSong(). Avoid
    // their invalid-host error path; retain the workaround for legacy code.
    if (std::is_same<PcmCallback, Pcm32Callback>::value) audio_.stopSong();
    else (void)audio_.connecttohost(nullptr);
    if (MetadataCallbacks<Audio>::modern) {
      // 4.0.0 retains metadata in an internal queue across stopSong(). Its
      // loop() delivers those events even when stopped. Flush while our active
      // generation is zero, before a new connection can claim the old title.
      audio_.loop();
      Metadata discarded;
      while (xQueueReceive(metadataQueue, &discarded, 0) == pdTRUE) {}
    }
  }
  void reset(const StartRequest& request) {
    deactivate();
    stopPlayback();
    desired_ = request.station;
    fallback_ = request.fallback;
    snapshot_.requested = desired_;
    snapshot_.generation = request.generation;
    snapshot_.codec[0] = snapshot_.title[0] = 0;
    snapshot_.bitrateKbps = 0;
    attempts_ = 0;
    connecting_ = fallbackUsed_ = stablePlaying_ = false;
    retryAt_ = millis();
    snapshot_.state = online_ ? State::Retrying : State::Offline;
    strlcpy(snapshot_.detail, "Starting selected station", sizeof(snapshot_.detail));
    Metadata discarded;
    while (xQueueReceive(metadataQueue, &discarded, 0) == pdTRUE) {}
    publish();
  }
  void failed(const char* reason) {
    deactivate();
    stopPlayback();
    connecting_ = stablePlaying_ = false;
    Serial.printf("audio: %s\n", reason);
    if (attempts_ < 3) {
      snapshot_.state = State::Retrying;
      retryAt_ = millis() + (2000UL << (attempts_ ? attempts_ - 1 : 0));
      snprintf(snapshot_.detail, sizeof(snapshot_.detail), "%s; retry %u/3", reason, attempts_ + 1);
    } else if (!fallbackUsed_ && RadioModel::validStation(fallback_) &&
               strcmp(fallback_.url, desired_.url)) {
      desired_ = fallback_;
      snapshot_.requested = desired_;
      snapshot_.codec[0] = snapshot_.title[0] = 0;
      snapshot_.bitrateKbps = 0;
      fallbackUsed_ = true;
      attempts_ = 0;
      retryAt_ = millis() + 1000;
      snapshot_.state = State::Retrying;
      strlcpy(snapshot_.detail, "Returning to previous station", sizeof(snapshot_.detail));
    } else {
      snapshot_.state = State::Failed;
      strlcpy(snapshot_.detail, "Open menu > Retry audio", sizeof(snapshot_.detail));
    }
    publish();
  }
  void connect() {
    const size_t urlLength = strlen(desired_.url);
    // The pinned library rejects len >= 502 and parses schemes case-sensitively.
    // Keep the model's canonical URL intact for catalog matching/persistence.
    if (urlLength > 501) {
      attempts_ = 3;
      failed("Stream URL exceeds 501 bytes");
      return;
    }
    char streamUrl[sizeof(desired_.url)] = {};
    memcpy(streamUrl, desired_.url, urlLength + 1);
    for (size_t i = 0; i < 5 && streamUrl[i] != ':'; ++i)
      streamUrl[i] = tolower(static_cast<unsigned char>(streamUrl[i]));
    ++attempts_;
    Serial.printf("audio: connecting %u/3 to %s\n", attempts_, desired_.name);
    snapshot_.state = State::Connecting;
    snprintf(snapshot_.detail, sizeof(snapshot_.detail), "Connection attempt %u/3", attempts_);
    publish();
    deactivate();
    stablePlaying_ = false;
    // Socket setup can block, but never blocks the input/UI or directory tasks.
    // connecttohost() already stops/reset buffers while holding its decoder mutex.
    bool accepted = audio_.connecttohost(streamUrl);
    if (!accepted) { failed("Stream connection failed"); return; }
    if (++metadataEpoch_ == 0) ++metadataEpoch_;
    portENTER_CRITICAL(&signalMux);
    activeGeneration = snapshot_.generation;
    activeMetadataEpoch = metadataEpoch_;
    portEXIT_CRITICAL(&signalMux);
    connectAt_ = millis();
    connecting_ = true;
  }
  void drainMetadata() {
    uint32_t acceptedGeneration, acceptedEpoch;
    portENTER_CRITICAL(&signalMux);
    acceptedGeneration = activeGeneration;
    acceptedEpoch = activeMetadataEpoch;
    portEXIT_CRITICAL(&signalMux);
    Metadata event;
    for (int i = 0; i < 16 && xQueueReceive(metadataQueue, &event, 0) == pdTRUE; ++i) {
      if (event.type != MetadataType::Info &&
          (!event.generation || event.generation != snapshot_.generation ||
           event.generation != acceptedGeneration || !event.epoch ||
           event.epoch != acceptedEpoch)) continue;
      if (event.type == MetadataType::Title) strlcpy(snapshot_.title, event.text, sizeof(snapshot_.title));
      else if (event.type == MetadataType::Bitrate) snapshot_.bitrateKbps = strtoul(event.text, nullptr, 10) / 1000;
      else {
        String info(event.text);
        info.toLowerCase();
        if (info.indexOf("error") >= 0 || info.indexOf("fail") >= 0 || info.indexOf("timeout") >= 0)
          Serial.printf("audio: %s\n", event.text);
      }
    }
  }
  void run() {
    MetadataCallbacks<Audio>::install();
    Serial.printf("audio: %s; %s metadata callbacks\n", pcmApiName(),
                  MetadataCallbacks<Audio>::modern ? "event" : "legacy");
    if (!audio_.setPinout(RadioConfig::I2sBclk, RadioConfig::I2sLrck, RadioConfig::I2sData)) {
      snapshot_.state = State::Failed;
      snapshot_.fatal = true;
      strlcpy(snapshot_.detail, "I2S initialization failed", sizeof(snapshot_.detail));
      publish();
      vTaskDelete(nullptr);
      return;
    }
    audio_.setConnectionTimeout(2500, 4000);
    audio_.setVolume(0);
    publish();
    for (;;) {
      uint8_t volume;
      if (xQueueReceive(volumes_, &volume, 0) == pdTRUE) audio_.setVolume(volume);
      bool online;
      if (xQueueReceive(networks_, &online, 0) == pdTRUE && online_ != online) {
        online_ = online;
        if (!online_) {
          deactivate(); stopPlayback(); connecting_ = stablePlaying_ = false;
          snapshot_.state = State::Offline;
          strlcpy(snapshot_.detail, "Waiting for Wi-Fi", sizeof(snapshot_.detail));
        } else if (RadioModel::validStation(desired_)) {
          attempts_ = 0; retryAt_ = millis(); snapshot_.state = State::Retrying;
        } else snapshot_.state = State::Idle;
        publish();
      }
      StartRequest request;
      if (xQueueReceive(starts_, &request, 0) == pdTRUE) reset(request);
      uint32_t now = millis();
      if (online_ && RadioModel::validStation(desired_) && snapshot_.state == State::Retrying &&
          static_cast<int32_t>(now - retryAt_) >= 0) connect();
      audio_.loop();
      drainMetadata();
      uint32_t frameGeneration, frameAt;
      portENTER_CRITICAL(&signalMux);
      frameGeneration = decodedGeneration;
      frameAt = lastFrameMs;
      portEXIT_CRITICAL(&signalMux);
      // Sample time after the decoder timestamp: a new frame on the other core
      // must not appear to be in the future and underflow the unsigned age.
      now = millis();
      if (online_ && (connecting_ || snapshot_.state == State::Playing || snapshot_.state == State::Buffering)) {
        if (frameGeneration == snapshot_.generation && frameAt) {
          if (now - frameAt < 3000) {
            if (!stablePlaying_) { stablePlaying_ = true; playingSince_ = now; }
            // Restore the outage retry budget only after stable playback, so a
            // flapping stream cannot reset it by decoding a single frame.
            if (now - playingSince_ >= 20000) attempts_ = 0;
            if (snapshot_.state != State::Playing)
              Serial.printf("audio: Playing %s; decoded PCM confirmed\n", desired_.name);
            snapshot_.state = State::Playing;
            snapshot_.confirmed = desired_;
            strlcpy(snapshot_.detail, fallbackUsed_ ? "Previous station restored" : "Decoded audio ready", sizeof(snapshot_.detail));
            connecting_ = false;
          } else if (now - frameAt < 12000) {
            stablePlaying_ = false;
            snapshot_.state = State::Buffering;
            strlcpy(snapshot_.detail, "Waiting for audio data", sizeof(snapshot_.detail));
          } else failed("Audio stalled");
        } else if (now - connectAt_ > 15000) failed("No decodable audio");
        if ((connecting_ || snapshot_.state == State::Playing || snapshot_.state == State::Buffering) && !audio_.isRunning())
          failed("Stream stopped");
      }
      if (now - metadataAt_ >= 500) {
        metadataAt_ = now;
        strlcpy(snapshot_.codec, audio_.getCodecname(), sizeof(snapshot_.codec));
        snapshot_.bitrateKbps = audio_.getBitRate() / 1000;
      }
      if (now - publishedAt_ >= 200) publish();
      vTaskDelay(pdMS_TO_TICKS(2) ? pdMS_TO_TICKS(2) : 1);
    }
  }
};
}
