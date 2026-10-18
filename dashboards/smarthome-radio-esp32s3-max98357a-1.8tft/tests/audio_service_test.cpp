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
}  // namespace RadioAudioHost

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
    service.deactivate();
    delete service.starts_;
    delete service.volumes_;
    delete service.networks_;
    delete service.snapshots_;
    delete metadataQueue;
    metadataQueue = nullptr;
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
      assert(service.audio_.rawStops == 0);
      assert(service.audio_.nullStops >= 1);
      cleanup(service);
    }

    AudioService flapping;
    assert(flapping.begin());
    flapping.network(true);
    assert(flapping.start(station("https://example.org/flapping"), RadioModel::Station{}, 1));
    runUntil(flapping, 60000, false);  // One decoded frame per connection, then silence.
    assert(flapping.snapshot_.state == State::Failed);
    assert(flapping.attempts_ == 3 && flapping.audio_.connections == 3);
    assert(flapping.audio_.rawStops == 0);
    cleanup(flapping);
  }

  static void testMetadataOwnership() {
    AudioService service;
    assert(service.begin());
    service.online_ = true;
    StartRequest request{station("https://example.org/a", "a"),
                         station("https://example.org/fallback", "fallback"), 1};
    service.reset(request);
    service.connect();
    metadata(MetadataType::Title, "Old title");
    Metadata old;
    assert(xQueueReceive(metadataQueue, &old, 0));
    request.generation = 2;
    service.reset(request);
    service.connect();
    assert(xQueueSend(metadataQueue, &old, 0));  // Late delivery from the old request.
    service.drainMetadata();
    assert(service.snapshot_.title[0] == '\0');
    metadata(MetadataType::Title, "New title");
    service.drainMetadata();
    assert(std::strcmp(service.snapshot_.title, "New title") == 0);

    metadata(MetadataType::Title, "Pre-fallback title");
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
    metadata(MetadataType::Title, "Fallback title");
    service.drainMetadata();
    assert(std::strcmp(service.snapshot_.title, "Fallback title") == 0);
    assert(service.audio_.rawStops == 0);
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
    assert(service.audio_.rawStops == 0);
    cleanup(service);
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
};

}  // namespace RadioAudio

int main() {
  RadioAudio::AudioServiceTestAccess::testRetryBudget();
  RadioAudio::AudioServiceTestAccess::testMetadataOwnership();
  RadioAudio::AudioServiceTestAccess::testUrlBoundaries();
  RadioAudio::AudioServiceTestAccess::testFatalI2sFailure();
  std::puts("AudioService tests passed");
}
