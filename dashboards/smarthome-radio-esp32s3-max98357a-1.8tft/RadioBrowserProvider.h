#pragma once

#include <ArduinoJson.h>
#include "RadioConfig.h"
#include "CatalogTypes.h"
#include "CatalogTransport.h"

namespace RadioCatalog {
inline bool supportedCodec(String codec) {
  codec.toUpperCase();
  return codec == "MP3" || codec == "AAC" || codec == "AAC+" || codec == "AACP" ||
         codec == "WAV" || codec == "FLAC" || codec == "OPUS" || codec == "OGG" ||
         codec == "VORBIS" || codec == "M4A";
}

class RadioBrowserProvider {
 public:
  bool fetch(Result& result, RadioNetwork::ReadCancellation cancellation = {}) {
    addServer("all.api.radio-browser.info");
    addServer("de1.api.radio-browser.info");
    addServer("de2.api.radio-browser.info");
    discover(cancellation);
    const int first = esp_random() % serverCount_;
    for (int i = 0; i < serverCount_ && i < 3; ++i) {
      if (cancellation.requested()) return false;
      if (fetch(servers_[(first + i) % serverCount_], result, cancellation)) return true;
      Serial.println(result.message);
    }
    return false;
  }

 private:
  String servers_[8];
  uint8_t serverCount_ = 0;
  bool discovered_ = false;
  void addServer(const String& name) {
    if (!name.endsWith(".api.radio-browser.info") || name.length() > 80) return;
    for (size_t i = 0; i < name.length(); ++i)
      if (!isalnum(static_cast<unsigned char>(name[i])) && name[i] != '.' && name[i] != '-') return;
    for (int i = 0; i < serverCount_; ++i) if (servers_[i] == name) return;
    if (serverCount_ < 8) servers_[serverCount_++] = name;
  }
  void discover(RadioNetwork::ReadCancellation cancellation) {
    if (discovered_) return;
    discovered_ = true;
    // all.api resolves across available mirrors. Seed names remain usable if discovery fails.
    WiFiClientSecure client;
    HTTPClient http;
    configureHttp(client, http);
    if (!http.begin(client, "https://all.api.radio-browser.info/json/servers")) return;
    if (http.GET() == HTTP_CODE_OK) {
      DynamicJsonDocument doc(4096);
      StaticJsonDocument<96> filter;
      filter[0]["name"] = true;
      NetworkClock clock;
      RadioNetwork::ReadLimits limits;
      limits.cancellation = cancellation;
      RadioNetwork::JsonReader<WiFiClientSecure, NetworkClock> reader(client, clock, http.getSize(), limits);
      const DeserializationError error = deserializeJson(doc, reader, DeserializationOption::Filter(filter));
      if (!error && reader.finish())
        for (JsonObject server : doc.as<JsonArray>()) addServer(server["name"] | "");
      else Serial.printf("catalog discovery: JSON=%s transport=%s bytes=%u/%d\n", error.c_str(),
                         RadioNetwork::statusName(reader.status()), static_cast<unsigned>(reader.received()), http.getSize());
    }
    http.end();
  }
  bool fetch(const String& server, Result& result, RadioNetwork::ReadCancellation cancellation) {
    result.count = 0;
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
    configureHttp(client, http);
    if (!http.begin(client, url)) { copyField(result.message, sizeof(result.message), "Cannot start directory request"); return false; }
    int code = http.GET();
    if (cancellation.requested()) { http.end(); return false; }
    if (code != HTTP_CODE_OK) {
      snprintf(result.message, sizeof(result.message), "Directory HTTP/network error %d", code);
      http.end();
      return false;
    }
    // Filter locally; the directory does not promise a server-side fields= projection.
    StaticJsonDocument<320> filter;
    const char* fields[] = {"stationuuid", "name", "url_resolved", "bitrate", "countrycode", "country", "codec"};
    for (const char* field : fields) filter[0][field] = true;
    DynamicJsonDocument doc(32768);
    NetworkClock clock;
    const int contentLength = http.getSize();
    RadioNetwork::ReadLimits limits;
    limits.cancellation = cancellation;
    RadioNetwork::JsonReader<WiFiClientSecure, NetworkClock> reader(client, clock, contentLength, limits);
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
      RadioModel::assignStationLocation(station, row["country"] | "");
      station.bitrateKbps = row["bitrate"] | 0;
      if (RadioModel::validStation(station)) result.stations[result.count++] = station;
    }
    return true;
  }
};
} // namespace RadioCatalog
