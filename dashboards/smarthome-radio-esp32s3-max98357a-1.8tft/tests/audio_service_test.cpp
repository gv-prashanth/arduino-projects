// Compile with -I tests/audio_stubs; this executes the production AudioService.
#include "../AudioService.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
#include <type_traits>

namespace RadioAudioHost {
uint32_t nowMs = 100, stopAt = 1000;
bool continuousFrames = false, pinoutReady = true;
bool framesEnabled = true, silentPcm = false, emitStartupMetadata = false;
int32_t frameSamples = 100;
unsigned weakDefaultCalls = 0;
struct PlaybackResolver {
  unsigned calls = 0;
  bool fail = false, unterminatedOutput = false;
  uint32_t delayMs = 0;
  std::string failedUrl, redirectedUrl;
  std::vector<std::string> retryUrls;
  std::vector<RadioModel::Station> requested;
};
PlaybackResolver playbackResolver;
}  // namespace RadioAudioHost

// Host builds link this deterministic resolver instead of the production HTTP
// implementation. All callback API variants exercise the same AudioService.
namespace RadioStreaming {
bool resolvePlaybackUrl(const RadioModel::Station& station, char (&out)[512], char (&error)[80]) {
  auto& resolver = RadioAudioHost::playbackResolver;
  ++resolver.calls;
  resolver.requested.push_back(station);
  RadioAudioHost::nowMs += resolver.delayMs;
  if (resolver.fail && (resolver.failedUrl.empty() || resolver.failedUrl == station.url)) {
    RadioModel::copyText(error, "HLS redirect resolution failed");
    return false;
  }
  if (resolver.unterminatedOutput) {
    std::memset(out, 'x', sizeof(out));
    return true;
  }
  std::string target = resolver.redirectedUrl.empty() ? station.url : resolver.redirectedUrl;
  if (!resolver.retryUrls.empty()) {
    const std::size_t index = resolver.calls - 1 < resolver.retryUrls.size() ?
        resolver.calls - 1 : resolver.retryUrls.size() - 1;
    target = resolver.retryUrls[index];
  }
  if (!RadioModel::copyText(out, target.c_str())) {
    RadioModel::copyText(error, "Resolved playback URL is too long");
    return false;
  }
  return true;
}
}  // namespace RadioStreaming

static_assert(std::is_trivially_copyable<RadioAudio::Snapshot>::value,
              "FreeRTOS queues require value-only snapshots");

namespace RadioAudio {

// All private test access stays within this explicitly friended type.
struct AudioServiceTestAccess {
  static RadioModel::Station station(const char* url, const char* id = "test-id") {
    RadioModel::Station result;
    assert(RadioModel::copyText(result.id, id));
    assert(RadioModel::copyText(result.name, "Test station"));
    assert(RadioModel::copyText(result.url, url));
    assert(RadioModel::validStation(result));
    return result;
  }

  static void cleanup(AudioService& service) {
    assert(RadioAudioHost::weakDefaultCalls == 0);
    service.deactivate();
    delete service.starts_;
    delete service.volumes_;
    delete service.networks_;
    delete service.snapshots_;
    delete metadataQueue;
    metadataQueue = nullptr;
#if RADIO_TEST_AUDIO_API == 3
    Audio::audio_info_callback = {};
#endif
    RadioAudioHost::framesEnabled = true;
    RadioAudioHost::silentPcm = false;
    RadioAudioHost::emitStartupMetadata = false;
    RadioAudioHost::frameSamples = 100;
    RadioAudioHost::playbackResolver = RadioAudioHost::PlaybackResolver{};
  }

  static void assertStopPolicy(const AudioService& service) {
#if RADIO_TEST_AUDIO_API == 3
    assert(service.audio_.rawStops >= 1 && service.audio_.nullStops == 0);
#else
    assert(service.audio_.rawStops == 0 && service.audio_.nullStops >= 1);
#endif
  }

  static void runUntil(AudioService& service, uint32_t durationMs, bool continuous) {
    RadioAudioHost::nowMs = 100;
    RadioAudioHost::stopAt = RadioAudioHost::nowMs + durationMs;
    RadioAudioHost::continuousFrames = continuous;
    try { service.run(); } catch (const RadioAudioHost::StopSimulation&) {}
  }

  static void testRetryBudget() {
    // An early healthy frame cannot reset the retry budget. Sustained playback
    // does reset it after 20 seconds, allowing recovery from a later outage.
    for (const uint32_t duration : {19900u, 20100u}) {
      AudioService service;
      assert(service.begin());
      service.network(true);
      assert(service.start(station("https://example.org/a"), RadioModel::Station{}, 1));
      runUntil(service, duration, true);
      assert(service.snapshot_.state == State::Playing);
      assert(service.attempts_ == (duration < 20000 ? 1 : 0));
      assert(service.audio_.connections == 1);
      assertStopPolicy(service);
      cleanup(service);
    }

    AudioService flapping;
    assert(flapping.begin());
    flapping.network(true);
    assert(flapping.start(station("https://example.org/flapping"), RadioModel::Station{}, 1));
    runUntil(flapping, 60000, false);  // One decoded frame per connection, then silence.
    assert(flapping.snapshot_.state == State::Failed);
    assert(flapping.attempts_ == 3 && flapping.audio_.connections == 3);
    assertStopPolicy(flapping);
    cleanup(flapping);
  }

  static void testMetadataOwnership() {
    AudioService service;
    assert(service.begin());
    MetadataCallbacks<Audio>::install();
    service.online_ = true;
    StartRequest request{station("https://example.org/a", "a"),
                         station("https://example.org/fallback", "fallback"), 1};
    service.reset(request);
    service.connect();
    Audio::emitTitle("Old title");
    Metadata old;
    assert(xQueueReceive(metadataQueue, &old, 0));
    request.generation = 2;
    service.reset(request);
    service.connect();
    assert(xQueueSend(metadataQueue, &old, 0));  // Late delivery from the old request.
    service.drainMetadata();
    assert(service.snapshot_.title[0] == '\0');
    Audio::emitTitle("New title");
    service.drainMetadata();
    assert(std::strcmp(service.snapshot_.title, "New title") == 0);

    Audio::emitTitle("Pre-fallback title");
    assert(xQueueReceive(metadataQueue, &old, 0));
    service.attempts_ = 3;
    service.failed("test");
    assert(xQueueSend(metadataQueue, &old, 0));
    service.drainMetadata();
    assert(service.snapshot_.title[0] == '\0');
    service.connect();  // Same user generation, different connection epoch.
    assert(xQueueSend(metadataQueue, &old, 0));
    service.drainMetadata();
    assert(service.snapshot_.title[0] == '\0');
    Audio::emitTitle("Fallback title");
    service.drainMetadata();
    assert(std::strcmp(service.snapshot_.title, "Fallback title") == 0);
    // Callback message storage is owned by the library and may be immediately reused.
    char title[] = "Temporary title", bitrate[] = "192000";
    Audio::emitTitle(title);
    Audio::emitBitrate(bitrate);
    std::memset(title, 'x', sizeof(title) - 1);
    std::memset(bitrate, '0', sizeof(bitrate) - 1);
    service.drainMetadata();
    assert(std::strcmp(service.snapshot_.title, "Temporary title") == 0);
    assert(service.snapshot_.bitrateKbps == 192);

    for (const auto emit : {&Audio::emitInfo, &Audio::emitLog, &Audio::emitEof}) {
      char information[] = "Original event text";
      emit(information);
      std::memset(information, 'x', sizeof(information) - 1);
      Metadata queued;
      assert(xQueueReceive(metadataQueue, &queued, 0));
      assert(queued.type == MetadataType::Info);
      assert(std::strcmp(queued.text, "Original event text") == 0);
      assert(queued.generation == service.snapshot_.generation && queued.epoch != 0);
    }
#if RADIO_TEST_AUDIO_API == 3
    Audio::emitEvent(Audio::evt_name, "Ignore unrelated events");
    Metadata ignored;
    assert(xQueueReceive(metadataQueue, &ignored, 0) == 0);
#endif
    assertStopPolicy(service);
    cleanup(service);
  }

  static void testUrlBoundaries() {
    AudioService service;
    assert(service.begin());
    service.online_ = true;
    StartRequest request{station("HTTPS://example.org/live"), RadioModel::Station{}, 3};
    service.reset(request);
    service.connect();
    assert(service.audio_.lastUrl == "https://example.org/live");
    assert(std::strcmp(service.desired_.url, "HTTPS://example.org/live") == 0);

    std::string longUrl = "http://example.org/";
    longUrl.resize(501, 'x');
    request.station = station(longUrl.c_str());
    service.reset(request);
    service.connect();
    assert(service.audio_.lastUrl.size() == 501);
    const int connections = service.audio_.connections;
    longUrl.push_back('x');
    request.station = station(longUrl.c_str());
    service.reset(request);
    service.connect();
    assert(service.audio_.connections == connections);
    assert(service.snapshot_.state == State::Failed);
    assertStopPolicy(service);
    cleanup(service);
  }

  static void testResolvedUrlPreservesStationIdentity() {
    AudioService service;
    assert(service.begin());
    const char* official = "https://radio.wavespb.com/live/1119aa26b4e9f653/1119aa26b4e9f653.m3u8";
    const char* finalUrl = "https://d1tmej9eu7kw5c.cloudfront.net/1119aa26b4e9f653/1119aa26b4e9f653.m3u8";
    RadioModel::Station selected = station(official, "48");
    selected.source = RadioModel::Source::Akashvani;
    RadioAudioHost::playbackResolver.redirectedUrl = finalUrl;
    service.network(true);
    assert(service.start(selected, RadioModel::Station{}, 21));
    runUntil(service, 1000, true);
    assert(RadioAudioHost::playbackResolver.calls == 1);
    assert(service.audio_.connections == 1 && service.audio_.lastUrl == finalUrl);
    assert(service.snapshot_.state == State::Playing);
    for (const RadioModel::Station* canonical : {&service.desired_, &service.snapshot_.requested,
                                                &service.snapshot_.confirmed}) {
      assert(canonical->source == RadioModel::Source::Akashvani);
      assert(std::strcmp(canonical->id, "48") == 0);
      assert(std::strcmp(canonical->url, official) == 0);
    }
    const auto& supplied = RadioAudioHost::playbackResolver.requested.front();
    assert(supplied.source == selected.source && std::strcmp(supplied.id, selected.id) == 0);
    assert(std::strcmp(supplied.url, official) == 0);
    assertStopPolicy(service);
    cleanup(service);
  }

  static void testResolutionFailuresRespectRetryAndFallback() {
    for (const bool useFallback : {false, true}) {
      AudioService service;
      assert(service.begin());
      RadioModel::Station selected = station("https://radio.wavespb.com/live/test/test.m3u8", "45");
      selected.source = RadioModel::Source::Akashvani;
      const RadioModel::Station fallback = useFallback ? station("https://example.org/previous", "previous") :
                                                        RadioModel::Station{};
      RadioAudioHost::playbackResolver.fail = true;
      RadioAudioHost::playbackResolver.failedUrl = selected.url;
      service.network(true);
      assert(service.start(selected, fallback, 22));
      runUntil(service, 10000, true);
      assert(RadioAudioHost::playbackResolver.calls == (useFallback ? 4u : 3u));
      assert(service.audio_.connections == (useFallback ? 1 : 0));
      if (useFallback) {
        assert(service.snapshot_.state == State::Playing && service.fallbackUsed_);
        assert(service.audio_.lastUrl == fallback.url);
        assert(service.snapshot_.confirmed.source == fallback.source);
        assert(std::strcmp(service.snapshot_.confirmed.id, fallback.id) == 0);
        assert(std::strcmp(service.snapshot_.confirmed.url, fallback.url) == 0);
      } else {
        assert(service.snapshot_.state == State::Failed && service.attempts_ == 3);
        assert(!RadioModel::validStation(service.snapshot_.confirmed));
        assert(std::strcmp(service.snapshot_.requested.url, selected.url) == 0);
      }
      assertStopPolicy(service);
      cleanup(service);
    }
  }

  static void testResolutionRepeatsForRetries() {
    AudioService service;
    assert(service.begin());
    RadioModel::Station selected = station("https://radio.wavespb.com/live/test/test.m3u8", "44");
    selected.source = RadioModel::Source::Akashvani;
    RadioAudioHost::playbackResolver.retryUrls = {
        "https://cdn-one.example/live.m3u8", "https://cdn-two.example/live.m3u8",
        "https://cdn-three.example/live.m3u8"};
    service.network(true);
    assert(service.start(selected, RadioModel::Station{}, 23));
    runUntil(service, 60000, false); // One PCM callback per attempt, then a real stall.
    assert(service.snapshot_.state == State::Failed && service.attempts_ == 3);
    assert(RadioAudioHost::playbackResolver.calls == 3 && service.audio_.connections == 3);
    assert(service.audio_.lastUrl == "https://cdn-three.example/live.m3u8");
    for (const auto& supplied : RadioAudioHost::playbackResolver.requested) {
      assert(supplied.source == selected.source && std::strcmp(supplied.id, selected.id) == 0);
      assert(std::strcmp(supplied.url, selected.url) == 0);
    }
    assert(std::strcmp(service.snapshot_.requested.url, selected.url) == 0);
    cleanup(service);
  }

  static void testResolutionDoesNotConsumeFirstPcmDeadline() {
    AudioService service;
    assert(service.begin());
    RadioAudioHost::playbackResolver.delayMs = 12000;
    RadioAudioHost::framesEnabled = false;
    service.network(true);
    assert(service.start(station("https://example.org/delayed.m3u8"), RadioModel::Station{}, 24));
    runUntil(service, 16000, true);
    // Sixteen seconds elapsed since Start, but the decoder has only had four
    // seconds since the redirect resolver and socket setup completed.
    assert(service.connectAt_ == 12100);
    assert(service.snapshot_.state == State::Connecting);
    assert(service.attempts_ == 1 && service.audio_.connections == 1);
    assert(RadioAudioHost::playbackResolver.calls == 1);
    assert(!RadioModel::validStation(service.snapshot_.confirmed));
    cleanup(service);
  }

  static void testResolvedUrlBoundaries() {
    AudioService service;
    assert(service.begin());
    service.online_ = true;
    StartRequest request{station("https://example.org/short.m3u8"), RadioModel::Station{}, 25};
    std::string target = "https://cdn.example/";
    target.resize(501, 'x');
    RadioAudioHost::playbackResolver.redirectedUrl = target;
    service.reset(request);
    service.connect();
    assert(service.audio_.lastUrl == target);
    const int connections = service.audio_.connections;
    target.push_back('x');
    RadioAudioHost::playbackResolver.redirectedUrl = target;
    service.reset(request);
    service.connect();
    assert(service.audio_.connections == connections);
    assert(service.snapshot_.state == State::Failed);
    RadioAudioHost::playbackResolver.unterminatedOutput = true;
    service.reset(request);
    service.connect();
    assert(service.audio_.connections == connections);
    assert(service.snapshot_.state != State::Playing);
    assert(!RadioModel::validStation(service.snapshot_.confirmed));
    cleanup(service);
  }

  static void testDeferredModernTitleAcrossReset() {
#if RADIO_TEST_AUDIO_API == 3
    AudioService service;
    assert(service.begin());
    MetadataCallbacks<Audio>::install();
    service.online_ = true;
    StartRequest oldRequest{station("https://example.org/old", "old"), RadioModel::Station{}, 11};
    service.reset(oldRequest);
    service.connect();
    service.audio_.deferTitle("Previous station's internally buffered title");
    assert(service.audio_.deferredTitles.size() == 1);

    StartRequest newRequest{station("https://example.org/new", "new"), RadioModel::Station{}, 12};
    service.reset(newRequest);
    // The stopped library must deliver its old queue while the app generation
    // is inactive, before connecting the next station. It is not enough to
    // clear only messages that have already reached the app metadata queue.
    assert(!service.audio_.running);
    assert(service.audio_.deliveredDeferredTitles == 1);
    assert(service.audio_.deferredTitles.empty());
    service.connect();
    service.audio_.loop();
    service.drainMetadata();
    assert(service.snapshot_.generation == 12 && service.snapshot_.title[0] == '\0');

    service.audio_.deferTitle("Current station's deferred title");
    service.audio_.loop();
    service.drainMetadata();
    assert(std::strcmp(service.snapshot_.title, "Current station's deferred title") == 0);
    cleanup(service);
#endif
  }

  static void testFatalI2sFailure() {
    AudioService service;
    assert(service.begin());
    RadioAudioHost::pinoutReady = false;
    service.run();
    assert(service.snapshot_.fatal && service.snapshot_.state == State::Failed);
    assert(service.snapshot_.generation == 0);
    Snapshot published;
    assert(service.poll(published));
    assert(published.fatal && published.state == State::Failed);
    cleanup(service);
    RadioAudioHost::pinoutReady = true;
  }

  static void testConnectionIsNotPcmConfirmation() {
    // The fake reports a running socket, codec, bitrate, and metadata while
    // emitting no PCM callbacks. None of those substitutes for decoded frames.
    for (const uint32_t duration : {1000u, 60000u}) {
      AudioService service;
      assert(service.begin());
      RadioAudioHost::framesEnabled = false;
      RadioAudioHost::emitStartupMetadata = true;
      service.network(true);
      assert(service.start(station("https://example.org/no-pcm"), RadioModel::Station{}, 1));
      runUntil(service, duration, true);
      assert(service.audio_.pcmCallbacks == 0);
      assert(!RadioModel::validStation(service.snapshot_.confirmed));
      assert(std::strcmp(service.snapshot_.title, "Library title") == 0);
      if (duration < 15000) {
        assert(service.snapshot_.state == State::Connecting);
        assert(service.audio_.isRunning());
        assert(service.snapshot_.bitrateKbps == 128);
      } else {
        assert(service.snapshot_.state == State::Failed);
        assert(service.audio_.connections == 3 && service.attempts_ == 3);
      }
      cleanup(service);
    }
  }

  static void testSilentAndMutedPcmCounts() {
    const struct { bool silent; uint8_t volume; } cases[] = {{false, 7}, {true, 7}, {false, 0}};
    for (const auto& example : cases) {
      AudioService service;
      assert(service.begin());
      RadioAudioHost::silentPcm = example.silent;
      service.volume(example.volume);
      service.network(true);
      assert(service.start(station("https://example.org/pcm"), RadioModel::Station{}, 1));
      runUntil(service, 1000, true);
      assert(service.snapshot_.state == State::Playing);
      assert(RadioModel::validStation(service.snapshot_.confirmed));
      assert(service.audio_.positivePcmCallbacks > 0);
      assert(service.audio_.volumeValue == example.volume);
      if (example.silent || example.volume == 0)
        assert(service.audio_.silentPcmCallbacks == service.audio_.pcmCallbacks);
      else assert(service.audio_.silentPcmCallbacks == 0);
      assertStopPolicy(service);
      cleanup(service);
    }
  }

  static void testNonpositivePcmCounts() {
    AudioService service;
    assert(service.begin());
    RadioAudioHost::frameSamples = 0;
    service.network(true);
    assert(service.start(station("https://example.org/zero-count"), RadioModel::Station{}, 1));
    runUntil(service, 16000, true);
    assert(service.audio_.pcmCallbacks > 0 && service.audio_.positivePcmCallbacks == 0);
    assert(!RadioModel::validStation(service.snapshot_.confirmed));
    assert(service.snapshot_.state == State::Retrying && service.attempts_ == 1);
    cleanup(service);
#if RADIO_TEST_AUDIO_API == 2 || RADIO_TEST_AUDIO_API == 3
    AudioService negative;
    assert(negative.begin());
    RadioAudioHost::frameSamples = -100;
    negative.network(true);
    assert(negative.start(station("https://example.org/negative-count"), RadioModel::Station{}, 1));
    runUntil(negative, 16000, true);
    assert(negative.audio_.pcmCallbacks > 0 && negative.audio_.positivePcmCallbacks == 0);
    assert(!RadioModel::validStation(negative.snapshot_.confirmed));
    assert(negative.snapshot_.state == State::Retrying && negative.attempts_ == 1);
    cleanup(negative);
#endif
  }
};

}  // namespace RadioAudio

int main() {
  RadioAudio::AudioServiceTestAccess::testRetryBudget();
  RadioAudio::AudioServiceTestAccess::testMetadataOwnership();
  RadioAudio::AudioServiceTestAccess::testDeferredModernTitleAcrossReset();
  RadioAudio::AudioServiceTestAccess::testUrlBoundaries();
  RadioAudio::AudioServiceTestAccess::testResolvedUrlPreservesStationIdentity();
  RadioAudio::AudioServiceTestAccess::testResolutionFailuresRespectRetryAndFallback();
  RadioAudio::AudioServiceTestAccess::testResolutionRepeatsForRetries();
  RadioAudio::AudioServiceTestAccess::testResolutionDoesNotConsumeFirstPcmDeadline();
  RadioAudio::AudioServiceTestAccess::testResolvedUrlBoundaries();
  RadioAudio::AudioServiceTestAccess::testFatalI2sFailure();
  RadioAudio::AudioServiceTestAccess::testConnectionIsNotPcmConfirmation();
  RadioAudio::AudioServiceTestAccess::testSilentAndMutedPcmCounts();
  RadioAudio::AudioServiceTestAccess::testNonpositivePcmCounts();
  std::printf("AudioService tests passed (callback API %d)\n", RADIO_TEST_AUDIO_API);
}
