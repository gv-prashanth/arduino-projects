#include "../SettingsRestore.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>

using namespace RadioModel;
using namespace RadioStorage;

static_assert(kLegacySettingsRecordSize == 701, "Keep the original migration format");
static_assert(kSettingsRecordSize == 831 && kPreviousSettingsRecordSize == 767 && kCacheRecordSize == 6080 && kPreviousCacheRecordSize == 5568 && CachedStations == 8,
              "Persistence formats must be explicit and bounded");

static Settings savedSettings() {
  Settings settings;
  settings.volume = 9; settings.muted = true;
  settings.country = 2; settings.genre = 5; settings.codec = 2; settings.bitrateCap = 192;
  return settings;
}
static Station savedStation(const char* id, Source source = Source::RadioBrowser) {
  Station station;
  station.source = source;
  assert(copyText(station.id, id));
  assert(copyText(station.name, "AIR Raagam"));
  assert(copyText(station.url, "https://example.org/shared-stream"));
  assert(copyText(station.country, "IN"));
  assert(copyText(station.location, source == Source::Akashvani ? "NATIONAL" : "India"));
  station.bitrateKbps = 128;
  assert(validStation(station));
  return station;
}
static void assertSettings(const Settings& a, const Settings& b) {
  assert(a.volume == b.volume && a.muted == b.muted && a.source == b.source);
  assert(a.country == b.country && a.genre == b.genre && a.codec == b.codec && a.bitrateCap == b.bitrateCap);
  assert(std::strcmp(a.airLocation, b.airLocation) == 0);
}
static void assertStation(const Station& a, const Station& b) {
  assert(a.source == b.source && a.bitrateKbps == b.bitrateKbps);
  assert(std::strcmp(a.id, b.id) == 0 && std::strcmp(a.name, b.name) == 0);
  assert(std::strcmp(a.url, b.url) == 0 && std::strcmp(a.country, b.country) == 0);
  assert(std::strcmp(a.location, b.location) == 0);
}
// Reproduce the old firmware's documented format independently of the new writer.
static LegacySettingsRecord oldRecord(const Settings& settings, const Station& station) {
  LegacySettingsRecord record;
  std::memcpy(record.bytes, "RADI", 4);
  record.bytes[4] = 1;
  record.bytes[5] = settings.volume;
  record.bytes[6] = settings.muted;
  record.bytes[7] = settings.country;
  record.bytes[8] = settings.genre;
  record.bytes[9] = settings.codec;
  detail::writeU16(record.bytes + 10, settings.bitrateCap);
  std::memcpy(record.bytes + 12, station.id, std::strlen(station.id));
  std::memcpy(record.bytes + 52, station.name, std::strlen(station.name));
  std::memcpy(record.bytes + 180, station.url, std::strlen(station.url));
  std::memcpy(record.bytes + 692, station.country, std::strlen(station.country));
  detail::writeU16(record.bytes + 695, station.bitrateKbps);
  detail::writeU32(record.bytes + 697, detail::crc32(record.bytes, 697));
  return record;
}
static PreviousSettingsRecord previousRecord(const Settings& settings, const Station& station) {
  const LegacySettingsRecord legacy = oldRecord(settings, station);
  PreviousSettingsRecord record;
  std::memcpy(record.bytes, legacy.bytes, 12);
  record.bytes[4] = 2;
  record.bytes[12] = static_cast<uint8_t>(settings.source);
  std::memcpy(record.bytes + 13, settings.airLocation, std::strlen(settings.airLocation));
  record.bytes[77] = static_cast<uint8_t>(station.source);
  std::memcpy(record.bytes + 78, legacy.bytes + 12, 685);
  detail::writeU32(record.bytes + 763, detail::crc32(record.bytes, 763));
  return record;
}
static PreviousCacheRecord previousCache(const RadioCatalog::Filter& filter, const Station* rows, uint8_t count) {
  const CacheRecord current = makeCacheRecord(filter, rows, count);
  PreviousCacheRecord record;
  std::memcpy(record.bytes, current.bytes, kCacheHeaderSize);
  record.bytes[3] = '3';
  for (uint8_t i = 0; i < count; ++i)
    std::memcpy(record.bytes + kCacheHeaderSize + i * 686,
                current.bytes + kCacheHeaderSize + i * kCacheStationSize, 686);
  detail::writeU32(record.bytes + kPreviousCacheRecordSize - 4,
                   detail::crc32(record.bytes, kPreviousCacheRecordSize - 4));
  return record;
}
static void refreshCacheChecksum(CacheRecord& record) {
  detail::writeU32(record.bytes + kCacheRecordSize - 4,
                   detail::crc32(record.bytes, kCacheRecordSize - 4));
}
static void testLegacyMigrationAndRetirement() {
  const Settings original = savedSettings();
  for (const char* id : {"directory-uuid", "", "curated-air-00", "curated-air-2"}) {
    Settings settings;
    settings.source = Source::Akashvani;
    assert(copyText(settings.airLocation, "Old value"));
    Station lastGood;
    Station station = savedStation(id);
    station.location[0] = 0; // No location existed in the original byte record.
    assert(restoreStateRecord(oldRecord(original, station), settings, lastGood));
    assertSettings(settings, original);
    assertStation(lastGood, station);
    // Migrated state remains intact when subsequently written as v3.
    assert(restoreStateRecord(makeSettingsRecord(settings, lastGood), settings, lastGood));
    assertSettings(settings, original);
    assertStation(lastGood, station);
  }
  for (const char* id : {"curated-air-0", "curated-air-1"}) {
    Settings settings;
    Station lastGood;
    assert(restoreStateRecord(oldRecord(original, savedStation(id)), settings, lastGood));
    assertSettings(settings, original);
    assert(detail::emptyStation(lastGood));
    assert(restoreStateRecord(makeSettingsRecord(original, savedStation(id)), settings, lastGood));
    assert(detail::emptyStation(lastGood));
  }
  auto corrupt = oldRecord(original, savedStation("directory-id"));
  corrupt.bytes[30] ^= 1;
  Settings settings = original;
  Station lastGood = savedStation("unchanged");
  const Station before = lastGood;
  assert(!restoreStateRecord(corrupt, settings, lastGood));
  assertSettings(settings, original); assertStation(lastGood, before);
}
static void testIndependentPlayingSource() {
  Settings settings = savedSettings();
  settings.source = Source::Akashvani;
  assert(copyText(settings.airLocation, "Karnataka"));
  const Station rb = savedStation("directory-id");
  Settings restored;
  Station lastGood;
  assert(restoreStateRecord(makeSettingsRecord(settings, rb), restored, lastGood));
  assertSettings(restored, settings); assertStation(lastGood, rb);
  const Station air = savedStation("air:4", Source::Akashvani);
  settings.source = Source::RadioBrowser;
  assert(restoreStateRecord(makeSettingsRecord(settings, air), restored, lastGood));
  assertSettings(restored, settings); assertStation(lastGood, air);
}
static void testCacheRoundTripAndSourceIsolation() {
  Settings settings = savedSettings();
  for (const Source source : {Source::RadioBrowser, Source::Akashvani}) {
    settings.source = source;
    assert(copyText(settings.airLocation, "Karnataka"));
    const auto filter = RadioCatalog::filterFor(settings);
    const Station rows[] = {savedStation("channel-one", source), savedStation("channel-two", source)};
    CacheRecord record = makeCacheRecord(filter, rows, 2);
    Station restored[CachedStations];
    uint8_t count = 99;
    assert(restoreCacheRecord(record, filter, restored, count) && count == 2);
    assertStation(restored[0], rows[0]); assertStation(restored[1], rows[1]);
    auto different = filter;
    different.source = source == Source::RadioBrowser ? Source::Akashvani : Source::RadioBrowser;
    assert(!restoreCacheRecord(record, different, restored, count) && count == 0);
    if (source == Source::Akashvani) {
      different = filter;
      assert(copyText(different.location, "Tamil Nadu"));
      assert(!restoreCacheRecord(record, different, restored, count) && count == 0);
    }
    record.bytes[kCacheHeaderSize + 1] ^= 1;
    assert(!restoreCacheRecord(record, filter, restored, count) && count == 0);
  }
}
static void testCacheRetirementAndValidation() {
  const auto filter = RadioCatalog::filterFor(savedSettings());
  const Station rows[] = {savedStation("curated-air-0"), savedStation("directory-id"),
                          savedStation("curated-air-1"), savedStation("curated-air-0-suffix")};
  CacheRecord record = makeCacheRecord(filter, rows, 4);
  Station restored[CachedStations];
  uint8_t count = 99;
  assert(restoreCacheRecord(record, filter, restored, count) && count == 2);
  assertStation(restored[0], rows[1]); assertStation(restored[1], rows[3]);
  record = makeCacheRecord(filter, rows, 1);
  assert(restoreCacheRecord(record, filter, restored, count) && count == 0);
  record = makeCacheRecord(filter, nullptr, 0);
  assert(restoreCacheRecord(record, filter, restored, count) && count == 0);
  record = makeCacheRecord(filter, rows, 4);
  record.bytes[75] = CachedStations + 1;
  refreshCacheChecksum(record);
  assert(!restoreCacheRecord(record, filter, restored, count) && count == 0);
  record = makeCacheRecord(filter, rows, 4);
  // A structurally valid CRC cannot permit an unterminated row string.
  std::memset(record.bytes + kCacheHeaderSize + kCacheStationSize + 1, 'x', 40);
  refreshCacheChecksum(record);
  assert(!restoreCacheRecord(record, filter, restored, count) && count == 0);
  record = makeCacheRecord(filter, rows, 4);
  record.bytes[kCacheHeaderSize + kCacheStationSize] = static_cast<uint8_t>(Source::Akashvani);
  refreshCacheChecksum(record);
  assert(!restoreCacheRecord(record, filter, restored, count) && count == 0);
  assert(!buildCacheRecord(record, filter, rows, CachedStations + 1));
  assert(!validCacheHeader(record, filter));
  const Station wrongSource = savedStation("air:4", Source::Akashvani);
  assert(!buildCacheRecord(record, filter, &wrongSource, 1));
  assert(!validCacheHeader(record, filter));
  Station tooSmall[1];
  record = makeCacheRecord(filter, rows, 4);
  assert(!restoreCacheRecord(record, filter, tooSmall, count) && count == 0);
}
static void testPreviousSourceAwareRecordsAndCache() {
  Settings original = savedSettings();
  original.source = Source::Akashvani;
  assert(copyText(original.airLocation, "ANDHRA PRADESH"));
  for (const Source playingSource : {Source::RadioBrowser, Source::Akashvani}) {
    Station expected = savedStation("previous-id", playingSource);
    const PreviousSettingsRecord old = previousRecord(original, expected);
    Settings settings;
    Station lastGood;
    assert(restoreStateRecord(old, settings, lastGood));
    assertSettings(settings, original);
    expected.location[0] = 0;
    assertStation(lastGood, expected);
    assert(restoreStateRecord(makeSettingsRecord(settings, lastGood), settings, lastGood));
    assertStation(lastGood, expected);
    auto corrupt = old;
    corrupt.bytes[30] ^= 1;
    assert(!restoreStateRecord(corrupt, settings, lastGood));
    assertStation(lastGood, expected);
  }
  const auto filter = RadioCatalog::filterFor(original);
  const Station rows[] = {savedStation("4", Source::Akashvani), savedStation("48", Source::Akashvani)};
  auto cache = previousCache(filter, rows, 2);
  Station restored[CachedStations]; uint8_t count = 99;
  assert(restoreCacheRecord(cache, filter, restored, count) && count == 2);
  for (unsigned i = 0; i < count; ++i) {
    Station expected = rows[i]; expected.location[0] = 0;
    assertStation(restored[i], expected);
  }
  CacheRecord migrated;
  assert(buildCacheRecord(migrated, filter, restored, count));
  assert(restoreCacheRecord(migrated, filter, restored, count) && count == 2);
  auto other = filter; copyText(other.location, "TELANGANA");
  assert(!restoreCacheRecord(cache, other, restored, count) && count == 0);
  cache.bytes[100] ^= 1;
  assert(!restoreCacheRecord(cache, filter, restored, count) && count == 0);
  // Even a correct CRC cannot validate an unterminated new location field.
  CacheRecord bad = makeCacheRecord(filter, rows, 2);
  std::memset(bad.bytes + kCacheHeaderSize + 686, 'x', 64);
  refreshCacheChecksum(bad);
  assert(!restoreCacheRecord(bad, filter, restored, count) && count == 0);
}
int main() {
  testLegacyMigrationAndRetirement();
  testPreviousSourceAwareRecordsAndCache();
  testIndependentPlayingSource();
  testCacheRoundTripAndSourceIsolation();
  testCacheRetirementAndValidation();
  std::puts("Saved settings/cache restore tests passed");
}
