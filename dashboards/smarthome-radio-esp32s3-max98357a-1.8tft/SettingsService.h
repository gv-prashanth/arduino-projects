#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <new>
#include "CatalogService.h"
#include "SettingsRestore.h"

namespace RadioStorage {
struct WifiCredentials { char ssid[33] = {}, password[65] = {}; };
inline bool validCredentials(const WifiCredentials& credentials) {
  return memchr(credentials.ssid, 0, sizeof(credentials.ssid)) && credentials.ssid[0] &&
         memchr(credentials.password, 0, sizeof(credentials.password));
}
enum class Kind : uint8_t { Settings, Wifi, Cache };
struct Status { Kind kind; char message[80]; };
class SettingsService {
 public:
  bool begin() {
    settings_ = xQueueCreate(1, sizeof(RadioModel::SettingsRecord));
    wifi_ = xQueueCreate(1, sizeof(WifiCredentials));
    cache_ = xQueueCreate(1, sizeof(CacheRecord*));
    status_ = xQueueCreate(1, sizeof(Status));
    if (!settings_ || !wifi_ || !cache_ || !status_) return false;
    return xTaskCreate(taskEntry, "settings", 6144, this, 1, &task_) == pdPASS;
  }
  bool load(RadioModel::Settings& settings, RadioModel::Station& lastGood, WifiCredentials& wifi) {
    Preferences prefs;
    if (!prefs.begin("radio", true)) return false; // Missing namespace is normal on first boot.
    RadioModel::SettingsRecord record;
    bool restored = prefs.getBytesLength("state_v2") == sizeof(record) &&
                    prefs.getBytes("state_v2", &record, sizeof(record)) == sizeof(record) &&
                    restoreStateRecord(record, settings, lastGood);
    if (!restored) {
      // One-time migration from the original sketch; it never stored pre-mute loudness.
      settings.volume = min<uint8_t>(prefs.getUChar("volume", RadioModel::kInitialVolume), RadioModel::kMaxVolume);
      settings.muted = prefs.getBool("muted", false);
      if (settings.muted && !settings.volume) settings.volume = RadioModel::kInitialVolume;
      String country = prefs.getString("country", "all"), genre = prefs.getString("genre", "all");
      for (int i = 1; i < RadioModel::kCountryCount; ++i) if (country == RadioConfig::Countries[i].code) settings.country = i;
      for (int i = 1; i < RadioModel::kGenreCount; ++i) if (genre == RadioConfig::Genres[i].tag) settings.genre = i;
      RadioModel::Station candidate;
      String url = prefs.getString("stUrl", ""), name = prefs.getString("stName", "Saved station");
      if (RadioCatalog::copyField(candidate.url, sizeof(candidate.url), url.c_str()) &&
          RadioCatalog::copyField(candidate.name, sizeof(candidate.name), name.c_str()) && RadioModel::validStation(candidate))
        lastGood = candidate;
    }
    if (prefs.getBytesLength("wifi_v1") == sizeof(wifi)) {
      WifiCredentials candidate;
      if (prefs.getBytes("wifi_v1", &candidate, sizeof(candidate)) == sizeof(candidate) && validCredentials(candidate)) wifi = candidate;
    }
    prefs.end();
    return restored;
  }
  RadioCatalog::Result* loadCache(const RadioCatalog::Filter& wanted) {
    Preferences prefs;
    if (!prefs.begin("radio", true)) return nullptr;
    CacheRecord* record = new (std::nothrow) CacheRecord;
    if (!record) { prefs.end(); return nullptr; }
    bool valid = prefs.getBytesLength("cache_v2") == sizeof(*record) &&
                 prefs.getBytes("cache_v2", record, sizeof(*record)) == sizeof(*record) &&
                 validCacheHeader(*record, wanted);
    prefs.end();
    RadioCatalog::Result* result = valid ? new (std::nothrow) RadioCatalog::Result : nullptr;
    if (result) {
      result->request.filter = wanted;
      result->cached = true;
      valid = restoreCacheRecord(*record, wanted, result->stations, result->count);
      result->status = result->count ? RadioCatalog::ResultStatus::Success : RadioCatalog::ResultStatus::Empty;
      strlcpy(result->message, "Cached stations; refresh when online", sizeof(result->message));
    }
    delete record;
    if (!valid || (result && !result->count)) { delete result; return nullptr; }
    return result;
  }
  bool save(const RadioModel::Settings& settings, const RadioModel::Station& lastGood) {
    RadioModel::SettingsRecord record = RadioModel::makeSettingsRecord(settings, lastGood);
    return settings_ && xQueueOverwrite(settings_, &record) == pdTRUE;
  }
  bool saveWifi(const WifiCredentials& wifi) {
    return wifi_ && validCredentials(wifi) && xQueueOverwrite(wifi_, &wifi) == pdTRUE;
  }
  bool saveCache(const RadioCatalog::Result& result, const RadioModel::Settings& settings) {
    if (!cache_ || result.request.offset || result.cached || result.status != RadioCatalog::ResultStatus::Success) return true;
    CacheRecord* record = new (std::nothrow) CacheRecord;
    if (!record) return false;
    record->filter = result.request.filter;
    record->count = min<int>(result.count, CachedStations);
    for (int i = 0; i < record->count; ++i) record->stations[i] = RadioModel::makeSettingsRecord(settings, result.stations[i]);
    CacheRecord* stale = nullptr;
    if (xQueueReceive(cache_, &stale, 0) == pdTRUE) delete stale;
    if (xQueueSend(cache_, &record, 0) != pdTRUE) { delete record; return false; }
    return true;
  }
  bool poll(Status& status) { return status_ && xQueueReceive(status_, &status, 0) == pdTRUE; }

 private:
  QueueHandle_t settings_ = nullptr, wifi_ = nullptr, cache_ = nullptr, status_ = nullptr;
  TaskHandle_t task_ = nullptr;
  static void taskEntry(void* self) { static_cast<SettingsService*>(self)->run(); }
  void error(Kind kind, const char* detail) {
    Status status{}; status.kind = kind;
    strlcpy(status.message, detail, sizeof(status.message));
    Serial.println(detail);
    xQueueOverwrite(status_, &status);
  }
  template<typename T> void write(const char* key, const T& record, Kind kind) {
    Preferences prefs;
    if (!prefs.begin("radio", false)) { error(kind, "Settings storage unavailable"); return; }
    const bool success = prefs.putBytes(key, &record, sizeof(record)) == sizeof(record);
    prefs.end();
    if (!success) error(kind, "Settings write failed; check NVS space");
  }
  void run() {
    for (;;) {
      RadioModel::SettingsRecord record;
      if (xQueueReceive(settings_, &record, 0) == pdTRUE) write("state_v2", record, Kind::Settings);
      WifiCredentials wifi;
      if (xQueueReceive(wifi_, &wifi, 0) == pdTRUE) write("wifi_v1", wifi, Kind::Wifi);
      CacheRecord* cache = nullptr;
      if (xQueueReceive(cache_, &cache, 0) == pdTRUE) { write("cache_v2", *cache, Kind::Cache); delete cache; }
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
};
}
