#pragma once

#include <esp_heap_caps.h>
#include <new>
#include "AkashvaniPage.h"
#include "CatalogTransport.h"

namespace RadioCatalog {
class AkashvaniProvider {
 public:
  ~AkashvaniProvider() { release(snapshot_); }
  bool canServe(const Request& request) const { return snapshot_ && !request.refresh; }

  bool fetch(Result& result, RadioNetwork::ReadCancellation cancellation = {}) {
    if (!canServe(result.request) && !refresh(result, cancellation)) return false;
    result.locationCount = static_cast<uint8_t>(snapshot_->locationCount);
    for (unsigned i = 0; i < result.locationCount; ++i)
      copyField(result.locations[i], sizeof(result.locations[i]), snapshot_->locations[i]);
    buildAkashvaniPage(snapshot_->entries, snapshot_->count, result);
    return true;
  }

 private:
  struct Snapshot {
    uint16_t count = 0, locationCount = 0;
    char locations[RadioAkashvani::MaxLocations][RadioAkashvani::LocationSize] = {};
    RadioAkashvani::Entry entries[RadioAkashvani::MaxStations];
  };
  Snapshot* snapshot_ = nullptr; // Only the catalog worker accesses the snapshot.

  static void release(Snapshot* snapshot) {
    if (snapshot) { snapshot->~Snapshot(); heap_caps_free(snapshot); }
  }
  bool refresh(Result& result, RadioNetwork::ReadCancellation cancellation) {
    WiFiClientSecure client;
    HTTPClient http;
    configureHttp(client, http);
    if (!http.begin(client, "https://akashvani.gov.in/radio/live.php")) {
      copyField(result.message, sizeof(result.message), "Cannot start Akashvani request");
      return false;
    }
    const char* headers[] = {"Transfer-Encoding"};
    http.collectHeaders(headers, 1);
    http.addHeader("Accept-Encoding", "identity");
    const int code = http.GET();
    if (cancellation.requested()) { http.end(); return false; }
    if (code != HTTP_CODE_OK) {
      snprintf(result.message, sizeof(result.message), "Akashvani HTTP/network error %d", code);
      http.end();
      return false;
    }
    String transfer = http.header("Transfer-Encoding");
    transfer.toLowerCase();
    if (transfer.length() && transfer != "identity") {
      copyField(result.message, sizeof(result.message), "Akashvani unsupported HTTP framing");
      http.end();
      return false;
    }
    const int expected = http.getSize();
    if (expected > static_cast<int>(RadioAkashvani::MaxHtmlBytes)) {
      copyField(result.message, sizeof(result.message), "Akashvani response too large");
      http.end();
      return false;
    }
    const size_t capacity = expected > 0 ? static_cast<size_t>(expected) : RadioAkashvani::MaxHtmlBytes;
    char* html = static_cast<char*>(heap_caps_malloc(capacity + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!html) {
      copyField(result.message, sizeof(result.message), "Akashvani needs free OPI PSRAM");
      http.end();
      return false;
    }
    RadioNetwork::ReadLimits limits;
    limits.maxBytes = RadioAkashvani::MaxHtmlBytes;
    limits.totalTimeoutMs = 45000;
    limits.cancellation = cancellation;
    NetworkClock clock;
    RadioNetwork::JsonReader<WiFiClientSecure, NetworkClock> reader(client, clock, expected, limits);
    size_t length = 0;
    const bool complete = RadioNetwork::readCompleteBody(reader, html, capacity, length);
    http.end();
    if (!complete) {
      snprintf(result.message, sizeof(result.message), "Akashvani read: %s (%u bytes)",
               RadioNetwork::statusName(reader.status()), static_cast<unsigned>(length));
      Serial.println(result.message);
      heap_caps_free(html);
      return false;
    }
    if (cancellation.requested()) { heap_caps_free(html); return false; }
    void* memory = heap_caps_malloc(sizeof(Snapshot), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    Snapshot* replacement = memory ? new (memory) Snapshot : nullptr;
    if (!replacement) {
      copyField(result.message, sizeof(result.message), "Akashvani catalog: no free PSRAM");
      heap_caps_free(html);
      return false;
    }
    const auto parsed = RadioAkashvani::parse(html, length, replacement->entries, replacement->locations);
    heap_caps_free(html);
    if (!parsed.ok()) {
      snprintf(result.message, sizeof(result.message), "Akashvani catalog: %s", RadioAkashvani::errorMessage(parsed.error));
      Serial.println(result.message);
      release(replacement);
      return false;
    }
    replacement->count = parsed.count;
    replacement->locationCount = parsed.locationCount;
    Snapshot* previous = snapshot_;
    snapshot_ = replacement;
    release(previous);
    Serial.printf("catalog: Akashvani %u stations, %u locations, %u HTML bytes; free heap=%u PSRAM=%u\n",
                  snapshot_->count, snapshot_->locationCount, static_cast<unsigned>(length),
                  ESP.getFreeHeap(), ESP.getFreePsram());
    return true;
  }
};
} // namespace RadioCatalog
