#pragma once

#include "AkashvaniCatalog.h"
#include "CatalogTypes.h"

namespace RadioCatalog {
// Build a page from a complete immutable snapshot. Only this provider's
// location applies; remembered Radio Browser filters never exclude stations.
inline void buildAkashvaniPage(const RadioAkashvani::Entry* entries, std::size_t count,
                              Result& result) {
  result.count = 0;
  result.totalCount = 0;
  result.totalKnown = true;
  result.hasNext = false;
  const char* location = result.request.filter.location;
  for (std::size_t i = 0; i < count; ++i)
    if (!location[0] || std::strcmp(location, entries[i].location) == 0) ++result.totalCount;
  // A refresh can shrink the catalog. Keep the page within its new range.
  if (!result.totalCount) result.request.offset = 0;
  else if (result.request.offset >= result.totalCount)
    result.request.offset = ((result.totalCount - 1) / RadioModel::kPageSize) * RadioModel::kPageSize;
  unsigned matched = 0;
  for (std::size_t i = 0; i < count; ++i) {
    if (location[0] && std::strcmp(location, entries[i].location) != 0) continue;
    const unsigned index = matched++;
    if (index >= result.request.offset && result.count < RadioModel::kPageSize) {
      Station& station = result.stations[result.count++];
      station = entries[i].station;
      RadioModel::assignStationLocation(station, entries[i].location);
    }
  }
  result.hasNext = static_cast<unsigned>(result.request.offset) + result.count < result.totalCount;
}
} // namespace RadioCatalog
