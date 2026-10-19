#pragma once

#include <cstdint>
#include <cstring>
#include "CatalogState.h"

namespace RadioCatalog {
using RadioModel::Station;

constexpr std::size_t MaxLocations = 64;
constexpr std::size_t LocationSize = 64;
enum class ResultStatus : std::uint8_t { Success, Empty, Error };
struct Result {
  Request request;
  ResultStatus status = ResultStatus::Error;
  std::uint8_t count = 0;
  bool hasNext = false, cached = false;
  bool totalKnown = false;
  std::uint16_t totalCount = 0;
  std::uint8_t locationCount = 0;
  char locations[MaxLocations][LocationSize] = {};
  char message[80] = {};
  Station stations[RadioModel::kMaxStations];
};

inline bool copyField(char* destination, std::size_t capacity, const char* source) {
  if (!source || std::strlen(source) >= capacity) return false;
  std::memcpy(destination, source, std::strlen(source) + 1);
  return true;
}
} // namespace RadioCatalog
