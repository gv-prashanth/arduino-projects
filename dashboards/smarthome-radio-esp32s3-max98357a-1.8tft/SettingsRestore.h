#pragma once

#include "CatalogState.h"

namespace RadioStorage {

constexpr int CachedStations = 8;
// Explicit cache v4 bytes: GTC4/version, normalized filter, count, eight station
// payloads, CRC32. No settings repeated per row and no C++ padding persisted.
constexpr std::size_t kCacheHeaderSize = 76;
constexpr std::size_t kPreviousCacheRecordSize = 5568;
struct PreviousCacheRecord { uint8_t bytes[kPreviousCacheRecordSize] = {}; };
constexpr std::size_t kCacheStationSize = 750;
constexpr std::size_t kCacheRecordSize = kCacheHeaderSize + CachedStations * kCacheStationSize + 4;
struct CacheRecord { uint8_t bytes[kCacheRecordSize] = {}; };

inline bool retiredCuratedStation(const RadioModel::Station& station) {
  return station.source == RadioModel::Source::RadioBrowser &&
         (std::memcmp(station.id, "curated-air-0", sizeof("curated-air-0")) == 0 ||
          std::memcmp(station.id, "curated-air-1", sizeof("curated-air-1")) == 0);
}

template<class Record>
inline bool restoreStateRecord(const Record& record, RadioModel::Settings& settings,
                               RadioModel::Station& lastGood) {
  if (!RadioModel::decodeSettingsRecord(record, settings, lastGood)) return false;
  if (retiredCuratedStation(lastGood)) lastGood = RadioModel::Station{};
  return true;
}

inline bool validFilter(const RadioCatalog::Filter& filter) {
  if (!RadioModel::validSource(filter.source) ||
      !std::memchr(filter.location, 0, sizeof(filter.location))) return false;
  RadioModel::Settings settings;
  settings.source = filter.source;
  settings.country = filter.country;
  settings.genre = filter.genre;
  settings.codec = filter.codec;
  settings.bitrateCap = filter.bitrateCap;
  RadioModel::copyText(settings.airLocation, filter.location);
  return RadioModel::validSettings(settings) && RadioCatalog::filterFor(settings) == filter;
}

inline bool buildCacheRecord(CacheRecord& record, const RadioCatalog::Filter& filter,
                             const RadioModel::Station* stations, uint8_t count) {
  std::memset(record.bytes, 0, sizeof(record.bytes));
  if (!validFilter(filter) || count > CachedStations || (count && !stations)) return false;
  record.bytes[0] = 'G'; record.bytes[1] = 'T'; record.bytes[2] = 'C'; record.bytes[3] = '4';
  record.bytes[4] = 1;
  record.bytes[5] = static_cast<uint8_t>(filter.source);
  record.bytes[6] = filter.country;
  record.bytes[7] = filter.genre;
  record.bytes[8] = filter.codec;
  RadioModel::detail::writeU16(record.bytes + 9, filter.bitrateCap);
  uint8_t* bytes = record.bytes + 11;
  RadioModel::detail::writeText(bytes, filter.location);
  *bytes++ = count;
  for (uint8_t i = 0; i < count; ++i) {
    const RadioModel::Station& station = stations[i];
    if (!RadioModel::validStation(station) || station.source != filter.source) {
      std::memset(record.bytes, 0, sizeof(record.bytes));
      return false;
    }
    *bytes++ = static_cast<uint8_t>(station.source);
    RadioModel::detail::writeText(bytes, station.id);
    RadioModel::detail::writeText(bytes, station.name);
    RadioModel::detail::writeText(bytes, station.url);
    RadioModel::detail::writeText(bytes, station.country);
    RadioModel::detail::writeU16(bytes, station.bitrateKbps);
    bytes += 2;
    RadioModel::detail::writeText(bytes, station.location);
  }
  RadioModel::detail::writeU32(record.bytes + kCacheRecordSize - 4,
      RadioModel::detail::crc32(record.bytes, kCacheRecordSize - 4));
  return true;
}

inline CacheRecord makeCacheRecord(const RadioCatalog::Filter& filter,
                                  const RadioModel::Station* stations, uint8_t count) {
  CacheRecord record;
  buildCacheRecord(record, filter, stations, count);
  return record;
}

template<class Record>
inline bool validCacheHeader(const Record& record, const RadioCatalog::Filter& wanted) {
  static_assert(sizeof(record.bytes) == kCacheRecordSize || sizeof(record.bytes) == kPreviousCacheRecordSize,
                "Unsupported cache byte format");
  const bool current = sizeof(record.bytes) == kCacheRecordSize;
  const std::size_t payload = sizeof(record.bytes) - 4;
  if (record.bytes[0] != 'G' || record.bytes[1] != 'T' || record.bytes[2] != 'C' ||
      record.bytes[3] != (current ? '4' : '3') || record.bytes[4] != 1 || record.bytes[75] > CachedStations ||
      RadioModel::detail::readU32(record.bytes + payload) !=
          RadioModel::detail::crc32(record.bytes, payload)) return false;
  RadioCatalog::Filter filter;
  filter.source = static_cast<RadioModel::Source>(record.bytes[5]);
  filter.country = record.bytes[6];
  filter.genre = record.bytes[7];
  filter.codec = record.bytes[8];
  filter.bitrateCap = RadioModel::detail::readU16(record.bytes + 9);
  const uint8_t* bytes = record.bytes + 11;
  RadioModel::detail::readText(bytes, filter.location);
  return validFilter(filter) && filter == wanted;
}

// Count commits only after the whole record validates. The cache is a partial
// first-page preview; its count never implies a complete page or next offset.
template<class Record, std::size_t Capacity>
inline bool restoreCacheRecord(const Record& record, const RadioCatalog::Filter& wanted,
                               RadioModel::Station (&stations)[Capacity], uint8_t& count) {
  count = 0;
  if (!validCacheHeader(record, wanted)) return false;
  uint8_t accepted = 0;
  const uint8_t* bytes = record.bytes + kCacheHeaderSize;
  for (uint8_t i = 0; i < record.bytes[75]; ++i) {
    RadioModel::Station station;
    station.source = static_cast<RadioModel::Source>(*bytes++);
    RadioModel::detail::readText(bytes, station.id);
    RadioModel::detail::readText(bytes, station.name);
    RadioModel::detail::readText(bytes, station.url);
    RadioModel::detail::readText(bytes, station.country);
    station.bitrateKbps = RadioModel::detail::readU16(bytes);
    bytes += 2;
    if (sizeof(record.bytes) == kCacheRecordSize) RadioModel::detail::readText(bytes, station.location);
    if (!RadioModel::validStation(station) || station.source != wanted.source) return false;
    if (retiredCuratedStation(station)) continue;
    if (accepted >= Capacity) return false;
    stations[accepted++] = station;
  }
  count = accepted;
  return true;
}

}  // namespace RadioStorage
