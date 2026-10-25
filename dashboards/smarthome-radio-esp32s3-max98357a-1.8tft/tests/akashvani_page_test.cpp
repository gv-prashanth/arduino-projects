#include "../AkashvaniPage.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>

using namespace RadioCatalog;

int main() {
  // A full snapshot spans fourteen pages. Entries on different official IDs
  // remain distinct even when their stream URLs match.
  const unsigned size = 277;
  std::unique_ptr<RadioAkashvani::Entry[]> entries(new RadioAkashvani::Entry[size]);
  for (unsigned i = 0; i < size; ++i) {
    auto& row = entries[i];
    row.station.source = RadioModel::Source::Akashvani;
    std::snprintf(row.station.id, sizeof(row.station.id), "%u", i + 1);
    RadioModel::copyText(row.station.name, "Official station");
    RadioModel::copyText(row.station.url, "https://example.org/shared.m3u8");
    RadioModel::copyText(row.station.country, "IN");
    RadioModel::copyText(row.location, i < 3 ? "NATIONAL" : "KARNATAKA");
  }
  Result page;
  page.request.filter.source = RadioModel::Source::Akashvani;
  // Inapplicable preferences cannot accidentally filter out official rows.
  page.request.filter.country = 2;
  page.request.filter.genre = 5;
  page.request.filter.codec = 2;
  page.request.filter.bitrateCap = 64;
  for (unsigned offset = 0; offset < size; offset += RadioModel::kPageSize) {
    page.request.offset = offset;
    buildAkashvaniPage(entries.get(), size, page);
    const unsigned expected = size - offset < 20 ? size - offset : 20;
    assert(page.totalKnown && page.totalCount == size && page.count == expected);
    assert(page.hasNext == (offset + expected < size));
    assert(std::strtoul(page.stations[0].id, nullptr, 10) == offset + 1);
    assert(page.stations[0].source == RadioModel::Source::Akashvani);
    assert(std::strcmp(page.stations[0].location, offset < 3 ? "NATIONAL" : "KARNATAKA") == 0);
  }
  page.request.offset = 0;
  RadioModel::copyText(page.request.filter.location, "NATIONAL");
  buildAkashvaniPage(entries.get(), size, page);
  assert(page.totalKnown && page.totalCount == 3 && page.count == 3 && !page.hasNext);
  assert(std::strcmp(page.stations[0].id, "1") == 0 && std::strcmp(page.stations[2].id, "3") == 0);
  RadioModel::copyText(page.request.filter.location, "KARNATAKA");
  page.request.offset = 20;
  buildAkashvaniPage(entries.get(), size, page);
  assert(page.totalCount == 274 && page.count == 20 && page.hasNext);
  assert(std::strcmp(page.stations[0].id, "24") == 0);
  assert(std::strcmp(page.stations[0].location, "KARNATAKA") == 0);
  RadioModel::copyText(page.request.filter.location, "REMOVED LOCATION");
  buildAkashvaniPage(entries.get(), size, page);
  assert(page.totalKnown && !page.totalCount && !page.count && !page.hasNext);
  page.request.filter.location[0] = '\0';
  page.request.offset = 280;
  buildAkashvaniPage(entries.get(), size, page);
  assert(page.totalCount == 277 && page.count == 17 && !page.hasNext && page.request.offset == 260);
  // Refreshing an old last page after stations disappear cannot display14/13.
  page.request.offset = 260;
  buildAkashvaniPage(entries.get(), 249, page);
  assert(page.totalCount == 249 && page.count == 9 && !page.hasNext && page.request.offset == 240);
  std::puts("Akashvani paging tests passed");
}
