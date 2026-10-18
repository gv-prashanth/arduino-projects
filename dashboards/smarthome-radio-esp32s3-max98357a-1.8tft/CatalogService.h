#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <new>
#include <time.h>
#include "RadioConfig.h"
#include "CatalogState.h"
#include "NetworkJsonReader.h"

// The pinned ESP32 core ships the full Mozilla certificate bundle.
extern const uint8_t radioBundleStart[] asm("_binary_x509_crt_bundle_start");
extern const uint8_t radioBundleEnd[] asm("_binary_x509_crt_bundle_end");

namespace RadioCatalog {
using RadioModel::Station;
struct NetworkClock {
  uint32_t now() const { return millis(); }
  void pause() const { vTaskDelay(1); }
};
enum class ResultStatus : uint8_t { Success, Empty, Error };
struct Result {
  Request request;
  ResultStatus status = ResultStatus::Error;
  uint8_t count = 0;
  bool hasNext = false, cached = false;
  char message[80] = {};
  Station stations[RadioModel::kMaxStations];
};

inline bool copyField(char* dst, size_t capacity, const char* src) {
  if (!src || strlen(src) >= capacity) return false;
  memcpy(dst, src, strlen(src) + 1);
  return true;
}
inline String urlEncode(const char* value) {
  String encoded;
  const char* hex = "0123456789ABCDEF";
  for (const unsigned char* p = reinterpret_cast<const unsigned char*>(value); *p; ++p) {
    if (isalnum(*p) || *p == '-' || *p == '_' || *p == '.') encoded += char(*p);
    else { encoded += '%'; encoded += hex[*p >> 4]; encoded += hex[*p & 15]; }
  }
  return encoded;
}
inline bool supportedCodec(String codec) {
  codec.toUpperCase();
  return codec == "MP3" || codec == "AAC" || codec == "AAC+" || codec == "AACP" ||
         codec == "WAV" || codec == "FLAC" || codec == "OPUS" || codec == "OGG" ||
         codec == "VORBIS" || codec == "M4A";
}

class CatalogService {
 public:
  bool begin() {
    requests_ = xQueueCreate(1, sizeof(Request));
    results_ = xQueueCreate(1, sizeof(Result*));
    if (!requests_ || !results_) return false;
    return xTaskCreate(taskEntry, "catalog", 12288, this, 1, &task_) == pdPASS;
  }
  bool request(const Request& request) {
    return requests_ && xQueueOverwrite(requests_, &request) == pdTRUE;
  }
  // Results have exactly one owner: worker until received, then caller.
  bool poll(Result*& result) { return results_ && xQueueReceive(results_, &result, 0) == pdTRUE; }

 private:
  QueueHandle_t requests_ = nullptr, results_ = nullptr;
  TaskHandle_t task_ = nullptr;
  String servers_[8];
  uint8_t serverCount_ = 0;
  bool discovered_ = false;
  static void taskEntry(void* self) { static_cast<CatalogService*>(self)->run(); }
  void addServer(const String& name) {
    if (!name.endsWith(".api.radio-browser.info") || name.length() > 80) return;
    for (size_t i = 0; i < name.length(); ++i)
      if (!isalnum(static_cast<unsigned char>(name[i])) && name[i] != '.' && name[i] != '-') return;
    for (int i = 0; i < serverCount_; ++i) if (servers_[i] == name) return;
    if (serverCount_ < 8) servers_[serverCount_++] = name;
  }
  void configure(WiFiClientSecure& client, HTTPClient& http) {
    client.setCACertBundle(radioBundleStart, radioBundleEnd - radioBundleStart);
    client.setHandshakeTimeout(5);
    http.setConnectTimeout(3000);
    http.setTimeout(5000);
    http.setReuse(false);
    http.useHTTP10(true);  // getStream() must not contain chunked transfer framing.
    http.setUserAgent("GTS-Radio/2.0 (ESP32-S3)");
  }
  void discover() {
    if (discovered_) return;
    discovered_ = true;
    // all.api resolves across available mirrors. Seed names remain usable if discovery fails.
    WiFiClientSecure client;
    HTTPClient http;
    configure(client, http);
    if (!http.begin(client, "https://all.api.radio-browser.info/json/servers")) return;
    if (http.GET() == HTTP_CODE_OK) {
      DynamicJsonDocument doc(4096);
      StaticJsonDocument<96> filter;
      filter[0]["name"] = true;
      NetworkClock clock;
      RadioNetwork::JsonReader<WiFiClientSecure, NetworkClock> reader(client, clock, http.getSize());
      const DeserializationError error = deserializeJson(doc, reader, DeserializationOption::Filter(filter));
      if (!error && reader.finish())
        for (JsonObject server : doc.as<JsonArray>()) addServer(server["name"] | "");
      else Serial.printf("catalog discovery: JSON=%s transport=%s bytes=%u/%d\n", error.c_str(),
                         RadioNetwork::statusName(reader.status()), static_cast<unsigned>(reader.received()), http.getSize());
    }
    http.end();
  }
  bool fetch(const String& server, Result& result) {
    const Request& request = result.request;
    const Filter& f = request.filter;
    String url = "https://" + server + "/json/stations/search?hidebroken=true&order=clickcount&reverse=true&limit=" +
                 String(RadioModel::kPageSize) + "&offset=" + String(request.offset);
    if (f.country) url += "&countrycode=" + String(RadioConfig::Countries[f.country].code);
    if (f.genre) url += "&tag=" + urlEncode(RadioConfig::Genres[f.genre].tag);
    if (f.codec) url += "&codec=" + String(f.codec == 1 ? "AAC" : "MP3");
    if (f.bitrateCap) url += "&bitrateMax=" + String(f.bitrateCap);
    WiFiClientSecure client;
    HTTPClient http;
    configure(client, http);
    if (!http.begin(client, url)) { copyField(result.message, sizeof(result.message), "Cannot start directory request"); return false; }
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
      snprintf(result.message, sizeof(result.message), "Directory HTTP/network error %d", code);
      http.end();
      return false;
    }
    // Filter locally; the directory does not promise a server-side fields= projection.
    StaticJsonDocument<256> filter;
    const char* fields[] = {"stationuuid", "name", "url_resolved", "bitrate", "countrycode", "codec"};
    for (const char* field : fields) filter[0][field] = true;
    DynamicJsonDocument doc(32768);
    NetworkClock clock;
    const int contentLength = http.getSize();
    RadioNetwork::JsonReader<WiFiClientSecure, NetworkClock> reader(client, clock, contentLength);
    const DeserializationError error = deserializeJson(doc, reader, DeserializationOption::Filter(filter));
    const bool complete = !error && reader.finish();
    http.end();
    if (error || !complete || !doc.is<JsonArray>()) {
      const bool transportError = reader.status() != RadioNetwork::ReadStatus::Reading &&
                                  reader.status() != RadioNetwork::ReadStatus::Complete;
      const char* jsonReason = error ? error.c_str() : !doc.is<JsonArray>() ? "expected array" : "OK";
      snprintf(result.message, sizeof(result.message), "Directory %s: %s (%u bytes)", transportError ? "read" : "JSON",
               transportError ? RadioNetwork::statusName(reader.status()) : jsonReason,
               static_cast<unsigned>(reader.received()));
      Serial.printf("catalog: server=%s JSON=%s transport=%s bytes=%u/%d consumed=%u\n", server.c_str(),
                    jsonReason, RadioNetwork::statusName(reader.status()),
                    static_cast<unsigned>(reader.received()), contentLength, static_cast<unsigned>(reader.consumed()));
      return false;
    }
    JsonArray rows = doc.as<JsonArray>();
    result.hasNext = rows.size() >= RadioModel::kPageSize;
    for (JsonObject row : rows) {
      if (result.count >= RadioModel::kPageSize) break;
      if (!supportedCodec(row["codec"] | "")) continue;
      Station station;
      if (!copyField(station.id, sizeof(station.id), row["stationuuid"] | "") ||
          !copyField(station.name, sizeof(station.name), row["name"] | "") ||
          !copyField(station.url, sizeof(station.url), row["url_resolved"] | "") ||
          !copyField(station.country, sizeof(station.country), row["countrycode"] | "")) continue;
      station.bitrateKbps = row["bitrate"] | 0;
      if (RadioModel::validStation(station)) result.stations[result.count++] = station;
    }
    return true;
  }
  void deliver(Result* result) {
    Result* stale = nullptr;
    if (xQueueReceive(results_, &stale, 0) == pdTRUE) delete stale;
    if (xQueueSend(results_, &result, 0) != pdTRUE) delete result;
  }
  void run() {
    addServer("all.api.radio-browser.info");
    addServer("de1.api.radio-browser.info");
    addServer("de2.api.radio-browser.info");
    Request request;
    for (;;) {
      if (xQueueReceive(requests_, &request, portMAX_DELAY) != pdTRUE) continue;
      Result* result = new (std::nothrow) Result;
      if (!result) { Serial.println("catalog: no memory for result"); continue; }
      result->request = request;
      if (WiFi.status() != WL_CONNECTED) {
        copyField(result->message, sizeof(result->message), "Wi-Fi offline; previous results kept");
        deliver(result); continue;
      }
      const uint32_t start = millis();
      while (time(nullptr) < 1700000000 && millis() - start < 10000) vTaskDelay(pdMS_TO_TICKS(100));
      if (time(nullptr) < 1700000000) {
        copyField(result->message, sizeof(result->message), "Waiting for network clock (TLS)");
        deliver(result); continue;
      }
      discover();
      const int first = esp_random() % serverCount_;
      bool success = false;
      for (int i = 0; i < serverCount_ && i < 3; ++i) {
        if (fetch(servers_[(first + i) % serverCount_], *result)) { success = true; break; }
        Serial.println(result->message);
      }
      if (success) {
        result->status = result->count ? ResultStatus::Success : ResultStatus::Empty;
        copyField(result->message, sizeof(result->message), result->count ? "Stations ready" : "No matching supported stations");
      }
      deliver(result);
    }
  }
};
}
