#include "../SettingsRestore.h"

#include <cassert>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <initializer_list>

using namespace RadioModel;
using namespace RadioStorage;

// Existing persisted structure, retained here to catch an accidental migration
// layout change when moving its definition into the pure restore helper.
struct PreviousCacheRecord {
  uint32_t magic = 0x47544332;
  RadioCatalog::Filter filter;
  uint8_t count = 0;
  SettingsRecord stations[8];
};
static_assert(sizeof(CacheRecord) == sizeof(PreviousCacheRecord), "Keep cache_v2 byte size");
static_assert(offsetof(CacheRecord, filter) == offsetof(PreviousCacheRecord, filter), "Keep cache filter offset");
static_assert(offsetof(CacheRecord, count) == offsetof(PreviousCacheRecord, count), "Keep cache count offset");
static_assert(offsetof(CacheRecord, stations) == offsetof(PreviousCacheRecord, stations), "Keep cached records offset");
static_assert(kSettingsRecordSize == 701 && CachedStations == 8, "Keep existing persisted record formats");

static Settings savedSettings() {
  Settings settings;
  settings.volume = 9;
  settings.muted = true;
  settings.country = 2;
  settings.genre = 5;
  settings.codec = 2;
  settings.bitrateCap = 192;
  return settings;
}

static void assertSettings(const Settings& actual, const Settings& expected) {
  assert(actual.volume == expected.volume && actual.muted == expected.muted);
  assert(actual.country == expected.country && actual.genre == expected.genre);
  assert(actual.codec == expected.codec && actual.bitrateCap == expected.bitrateCap);
}

static Station savedStation(const char* id, const char* name = "AIR Raagam",
                            const char* url = "https://example.org/shared-stream") {
  Station station;
  assert(copyText(station.id, id));
  assert(copyText(station.name, name));
  assert(copyText(station.url, url));
  assert(copyText(station.country, "IN"));
  station.bitrateKbps = 128;
  assert(validStation(station));
  return station;
}

static void assertStation(const Station& actual, const Station& expected) {
  assert(std::strcmp(actual.id, expected.id) == 0);
  assert(std::strcmp(actual.name, expected.name) == 0);
  assert(std::strcmp(actual.url, expected.url) == 0);
  assert(std::strcmp(actual.country, expected.country) == 0);
  assert(actual.bitrateKbps == expected.bitrateKbps);
}

static void testRetiredSavedStationPreservesSettingsAndRestoredStatus() {
  const Settings original = savedSettings();
  for (const char* id : {"curated-air-0", "curated-air-1"}) {
    const SettingsRecord previous = makeSettingsRecord(original, savedStation(id));
    Settings settings;
    Station lastGood = savedStation("previous-output");
    const bool restored = restoreStateRecord(previous, settings, lastGood);
    assert(restored);  // The NVS loader must not fall back to old stUrl/stName.
    assertSettings(settings, original);
    assert(!validStation(lastGood));
    assert(lastGood.id[0] == '\0' && lastGood.name[0] == '\0' && lastGood.url[0] == '\0');
    assert(lastGood.country[0] == '\0' && lastGood.bitrateKbps == 0);

    // The normal startup save writes a valid cleaned state, without changing
    // settings or requiring a schema bump. It stays clean on the next reboot.
    const SettingsRecord cleaned = makeSettingsRecord(settings, lastGood);
    Settings again;
    Station restoredAgain;
    assert(restoreStateRecord(cleaned, again, restoredAgain));
    assertSettings(again, original);
    assert(!validStation(restoredAgain) && restoredAgain.url[0] == '\0');
  }
}

static void testDirectoryAndIdlessStationsSurvive() {
  const Settings original = savedSettings();
  for (const char* id : {"65a03553-5d4f-11ef-a7c3-5254000caff0", "", "curated-air-00", "curated-air-2"}) {
    // Identical AIR name/URL is deliberately allowed; only exact retired IDs
    // identify the old manual injection. ID-less legacy selections are intact.
    const Station station = savedStation(id);
    Settings settings;
    Station lastGood;
    assert(restoreStateRecord(makeSettingsRecord(original, station), settings, lastGood));
    assertSettings(settings, original);
    assertStation(lastGood, station);
  }
  Settings settings = original;
  Station lastGood = savedStation("unchanged-output");
  const Station before = lastGood;
  SettingsRecord corrupt = makeSettingsRecord(original, savedStation("curated-air-0"));
  corrupt.bytes[30] ^= 1;
  assert(!restoreStateRecord(corrupt, settings, lastGood));
  assertSettings(settings, original);
  assertStation(lastGood, before);
}

static CacheRecord cacheFor(const Settings& settings) {
  CacheRecord record;
  record.filter = RadioCatalog::filterFor(settings);
  return record;
}

static void testMixedCacheCompactsWithoutReordering() {
  const Settings settings = savedSettings();
  CacheRecord record = cacheFor(settings);
  const Station rows[] = {
    savedStation("curated-air-0"), savedStation("directory-uuid-1"),
    savedStation("directory-uuid-2", "Other station", "https://example.org/other"),
    savedStation("curated-air-1"), savedStation(""), savedStation("curated-air-0-suffix")
  };
  record.count = sizeof(rows) / sizeof(rows[0]);
  for (int i = 0; i < record.count; ++i) record.stations[i] = makeSettingsRecord(settings, rows[i]);
  Station restored[CachedStations];
  uint8_t count = 99;
  assert(restoreCacheRecord(record, record.filter, restored, count));
  assert(count == 4);
  const int kept[] = {1, 2, 4, 5};
  for (int i = 0; i < count; ++i) assertStation(restored[i], rows[kept[i]]);
  assert(restored[count].id[0] == '\0');
}

static void testAllRetiredAndEmptyCachesAreValidEmptyResults() {
  const Settings settings = savedSettings();
  CacheRecord record = cacheFor(settings);
  record.count = 2;
  record.stations[0] = makeSettingsRecord(settings, savedStation("curated-air-0"));
  record.stations[1] = makeSettingsRecord(settings, savedStation("curated-air-1"));
  Station restored[CachedStations];
  uint8_t count = 99;
  assert(restoreCacheRecord(record, record.filter, restored, count) && count == 0);
  // SettingsService's existing empty-result branch returns nullptr to trigger
  // a fresh directory search, rather than treating saved settings as invalid.
  record.count = 0;
  count = 99;
  assert(restoreCacheRecord(record, record.filter, restored, count) && count == 0);
}

static void testInvalidCachesNeverCommitAPartialCount() {
  const Settings settings = savedSettings();
  CacheRecord record = cacheFor(settings);
  record.count = 2;
  record.stations[0] = makeSettingsRecord(settings, savedStation("directory-uuid-1"));
  record.stations[1] = makeSettingsRecord(settings, savedStation("curated-air-0"));
  record.stations[1].bytes[100] ^= 1;
  Station restored[CachedStations];
  uint8_t count = 99;
  assert(!restoreCacheRecord(record, record.filter, restored, count) && count == 0);
  record.count = CachedStations + 1;
  count = 99;
  assert(!restoreCacheRecord(record, record.filter, restored, count) && count == 0);
  record.count = 1;
  RadioCatalog::Filter different = record.filter;
  different.country = 1;
  assert(!restoreCacheRecord(record, different, restored, count) && count == 0);
  record.stations[0] = makeSettingsRecord(Settings{}, savedStation("directory-uuid-1"));
  assert(!restoreCacheRecord(record, record.filter, restored, count) && count == 0);
}

int main() {
  testRetiredSavedStationPreservesSettingsAndRestoredStatus();
  testDirectoryAndIdlessStationsSurvive();
  testMixedCacheCompactsWithoutReordering();
  testAllRetiredAndEmptyCachesAreValidEmptyResults();
  testInvalidCachesNeverCommitAPartialCount();
  std::puts("Saved settings/cache restore tests passed");
}
