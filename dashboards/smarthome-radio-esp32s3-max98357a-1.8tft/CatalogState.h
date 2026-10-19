#pragma once

#include <cstdint>
#include "RadioModel.h"

namespace RadioCatalog {

struct Filter {
  RadioModel::Source source = RadioModel::Source::RadioBrowser;
  char location[64] = {};
  std::uint8_t country = 0, genre = 0, codec = 0;
  std::uint16_t bitrateCap = 0;
};

inline bool operator==(const Filter& a, const Filter& b) {
  return a.source == b.source && std::strncmp(a.location, b.location, sizeof(a.location)) == 0 &&
         a.country == b.country && a.genre == b.genre &&
         a.codec == b.codec && a.bitrateCap == b.bitrateCap;
}

inline Filter normalizeFilter(const Filter& candidate) {
  Filter filter;
  filter.source = candidate.source;
  if (candidate.source == RadioModel::Source::Akashvani) {
    RadioModel::copyText(filter.location, candidate.location);
  } else {
    filter.country = candidate.country;
    filter.genre = candidate.genre;
    filter.codec = candidate.codec;
    filter.bitrateCap = candidate.bitrateCap;
  }
  return filter;
}

inline Filter filterFor(const RadioModel::Settings& settings) {
  Filter filter;
  filter.source = settings.source;
  if (settings.source == RadioModel::Source::Akashvani) {
    RadioModel::copyText(filter.location, settings.airLocation);
    return filter;
  }
  filter.country = settings.country;
  filter.genre = settings.genre;
  filter.codec = settings.codec;
  filter.bitrateCap = settings.bitrateCap;
  return filter;
}

struct Request {
  std::uint32_t generation = 0;
  Filter filter;
  std::uint16_t offset = 0;
  bool refresh = false;
};

// State only: the caller queues requests and owns catalog/result storage.
class Transaction {
 public:
  Request request;
  std::uint32_t generation = 0;
  bool pending = false, failed = false;
  std::uint8_t failures = 0;
  std::uint32_t startedAt = 0, retryAt = 0;

  Request begin(Filter filter, std::uint16_t offset, std::uint32_t now, bool refresh = false) {
    if (++generation == 0) ++generation;
    request.generation = generation;
    request.filter = normalizeFilter(filter);
    request.refresh = refresh;
    request.offset = offset;
    pending = true;
    failed = false;
    startedAt = now;
    return request;
  }

  bool accepts(std::uint32_t resultGeneration) const {
    return pending && resultGeneration == request.generation;
  }

  bool started(std::uint32_t resultGeneration, std::uint32_t now) {
    if (!accepts(resultGeneration)) return false;
    startedAt = now; // Queuing behind a superseded download is not execution time.
    return true;
  }

  bool succeed(std::uint32_t resultGeneration) {
    if (!accepts(resultGeneration)) return false;
    pending = failed = false;
    failures = 0;
    return true;
  }

  bool fail(std::uint32_t resultGeneration, std::uint32_t now) {
    if (!accepts(resultGeneration)) return false;
    pending = false;
    failed = true;
    failures = failures < 4 ? failures + 1 : 4;
    retryAt = now + (5000UL << failures);
    return true;
  }

  bool expire(std::uint32_t now) {
    return pending && static_cast<std::uint32_t>(now - startedAt) > 75000U &&
           fail(request.generation, now);
  }

  bool retryDue(bool online, std::uint32_t now) const {
    return online && failed && !pending &&
           static_cast<std::int32_t>(now - retryAt) >= 0;
  }

  bool shouldApply(const Filter& committed, const Filter& candidate, bool hasCatalog) const {
    return !(committed == candidate) || pending || failed || !hasCatalog;
  }
};

}  // namespace RadioCatalog
