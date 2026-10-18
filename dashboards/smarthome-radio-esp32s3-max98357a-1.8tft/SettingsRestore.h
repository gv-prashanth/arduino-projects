#pragma once

#include "CatalogState.h"

namespace RadioStorage {

// Preserve the cache_v2 layout and capacity used by existing firmware.
constexpr int CachedStations = 8;
struct CacheRecord {
  uint32_t magic = 0x47544332;
  RadioCatalog::Filter filter;
  uint8_t count = 0;
  RadioModel::SettingsRecord stations[CachedStations];
};

inline bool retiredCuratedStation(const RadioModel::Station& station) {
  // These synthetic IDs identify the removed injected entries. Names and URLs
  // also belong to legitimate directory results and must not be blacklisted.
  return std::memcmp(station.id, "curated-air-0", sizeof("curated-air-0")) == 0 ||
         std::memcmp(station.id, "curated-air-1", sizeof("curated-air-1")) == 0;
}

inline bool restoreStateRecord(const RadioModel::SettingsRecord& record,
                               RadioModel::Settings& settings, RadioModel::Station& lastGood) {
  if (!RadioModel::decodeSettingsRecord(record, settings, lastGood)) return false;
  if (retiredCuratedStation(lastGood)) lastGood = RadioModel::Station{};
  // A retired station does not invalidate its settings or permit legacy fallback.
  return true;
}

inline bool validCacheHeader(const CacheRecord& record, const RadioCatalog::Filter& wanted) {
  return record.magic == 0x47544332 && record.count <= CachedStations && record.filter == wanted;
}

// One station of temporary storage, as in the original NVS loader. On failure
// the caller must discard the destination; count stays zero until all rows are
// validated. A successful empty result means every old row was retired/absent.
template<std::size_t Capacity>
inline bool restoreCacheRecord(const CacheRecord& record, const RadioCatalog::Filter& wanted,
                               RadioModel::Station (&stations)[Capacity], uint8_t& count) {
  count = 0;
  if (!validCacheHeader(record, wanted)) return false;
  uint8_t accepted = 0;
  for (int i = 0; i < record.count; ++i) {
    RadioModel::Settings settings;
    RadioModel::Station station;
    if (!RadioModel::decodeSettingsRecord(record.stations[i], settings, station) ||
        !(RadioCatalog::filterFor(settings) == wanted) || !RadioModel::validStation(station)) return false;
    if (retiredCuratedStation(station)) continue;
    if (accepted >= Capacity) return false;
    stations[accepted++] = station;
  }
  count = accepted;
  return true;
}

}  // namespace RadioStorage
