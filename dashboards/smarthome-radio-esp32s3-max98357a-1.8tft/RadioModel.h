#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace RadioModel {

constexpr int kMaxVolume = 21;
constexpr int kInitialVolume = 10;
constexpr int kCountryCount = 19;
constexpr int kGenreCount = 7;
constexpr int kCodecCount = 3;
constexpr int kPageSize = 20;
constexpr int kMaxStations = kPageSize;

enum class Source : std::uint8_t { RadioBrowser = 0, Akashvani = 1 };
inline bool validSource(Source source) {
  return source == Source::RadioBrowser || source == Source::Akashvani;
}

struct Station {
  Source source = Source::RadioBrowser;
  char id[40] = {};
  char name[128] = {};
  char url[512] = {};
  char country[3] = {};
  char location[64] = {};
  std::uint16_t bitrateKbps = 0;
};

struct Settings {
  Source source = Source::RadioBrowser;
  char airLocation[64] = {};
  std::uint8_t volume = kInitialVolume;
  bool muted = false;
  std::uint8_t country = 0;
  std::uint8_t genre = 0;
  std::uint8_t codec = 0;
  std::uint16_t bitrateCap = 0;
};

// The destination is always terminated and cleared beyond its text. False means
// the source was absent or truncated, so URL callers must reject that result.
template <std::size_t N>
inline bool copyText(char (&destination)[N], const char* source) {
  static_assert(N > 0, "A text buffer needs room for its terminator");
  std::memset(destination, 0, N);
  if (!source) return false;
  for (std::size_t i = 0; i < N; ++i) {
    if (source[i] == '\0') return true;
    if (i + 1 < N) destination[i] = source[i];
  }
  return false;
}

inline void adjustVolume(Settings& settings, int delta) {
  const std::uint8_t previous = settings.volume;
  const std::int64_t candidate = static_cast<std::int64_t>(settings.volume) + delta;
  settings.volume = static_cast<std::uint8_t>(
      candidate < 0 ? 0 : candidate > kMaxVolume ? kMaxVolume : candidate);
  if (settings.volume != previous) settings.muted = false;
}

inline void toggleMute(Settings& settings) { settings.muted = !settings.muted; }

inline std::uint8_t outputVolume(const Settings& settings) {
  if (settings.muted) return 0;
  return settings.volume > kMaxVolume ? kMaxVolume : settings.volume;
}

inline bool validSettings(const Settings& settings) {
  return validSource(settings.source) && std::memchr(settings.airLocation, 0, sizeof(settings.airLocation)) &&
         settings.volume <= kMaxVolume && settings.country < kCountryCount &&
         settings.genre < kGenreCount && settings.codec < kCodecCount;
}

namespace detail {

template <std::size_t N>
inline bool terminated(const char (&text)[N]) {
  return std::memchr(text, '\0', N) != nullptr;
}

template <std::size_t N>
inline bool allZero(const char (&text)[N]) {
  for (std::size_t i = 0; i < N; ++i) {
    if (text[i] != '\0') return false;
  }
  return true;
}

inline bool emptyStation(const Station& station) {
  return station.source == Source::RadioBrowser && allZero(station.id) && allZero(station.name) && allZero(station.url) &&
         allZero(station.country) && allZero(station.location) && station.bitrateKbps == 0;
}

inline char asciiLower(char c) {
  return c >= 'A' && c <= 'Z' ? static_cast<char>(c + ('a' - 'A')) : c;
}

inline bool startsWithIgnoreCase(const char* text, const char* prefix) {
  for (std::size_t i = 0; prefix[i]; ++i) {
    if (!text[i] || asciiLower(text[i]) != prefix[i]) return false;
  }
  return true;
}

}  // namespace detail

inline bool validStation(const Station& station) {
  if (!validSource(station.source) || !detail::terminated(station.id) || !detail::terminated(station.name) ||
      !detail::terminated(station.url) || !detail::terminated(station.country) ||
      !detail::terminated(station.location) ||
      station.name[0] == '\0') return false;

  const char* authority = nullptr;
  if (detail::startsWithIgnoreCase(station.url, "http://")) authority = station.url + 7;
  else if (detail::startsWithIgnoreCase(station.url, "https://")) authority = station.url + 8;
  if (!authority || !*authority || *authority == '/' || *authority == '?' ||
      *authority == '#') return false;
  for (const char* c = station.url; *c; ++c) {
    const unsigned char value = static_cast<unsigned char>(*c);
    if (value <= 0x20 || value == 0x7f) return false;
  }

  if (station.country[0] == '\0') return true;
  return station.country[0] >= 'A' && station.country[0] <= 'Z' &&
         station.country[1] >= 'A' && station.country[1] <= 'Z' &&
         station.country[2] == '\0';
}

// A provider's location is display metadata; missing/oversized names fall back
// to that station's own ISO country rather than any current search filter.
inline void assignStationLocation(Station& station, const char* location) {
  if (!location || !*location || !copyText(station.location, location))
    copyText(station.location, detail::terminated(station.country) ? station.country : "");
}

// IDs survive URL changes. Radio Browser also permits URL fallback for legacy
// selections; Akashvani IDs stay distinct even when several channels share a URL.
inline int findStation(const Station* list, int count, const Station& playing) {
  if (!list || count <= 0 || count > kMaxStations) return -1;
  if (detail::terminated(playing.id) && playing.id[0]) {
    for (int i = 0; i < count; ++i) {
      if (list[i].source == playing.source && detail::terminated(list[i].id) &&
          std::strcmp(list[i].id, playing.id) == 0)
        return i;
    }
  }
  if (playing.source == Source::RadioBrowser && detail::terminated(playing.url) && playing.url[0]) {
    for (int i = 0; i < count; ++i) {
      if (list[i].source == playing.source && detail::terminated(list[i].url) &&
          std::strcmp(list[i].url, playing.url) == 0)
        return i;
    }
  }
  return -1;
}

inline int previewIndex(int proposed, int count) {
  if (count <= 0) return -1;
  if (proposed < 0) return 0;
  return proposed >= count ? count - 1 : proposed;
}

// Explicit version 3 adds the playing station location to version 2.
// Earlier source-aware version 2 and original version 1 remain readable.
constexpr std::size_t kLegacySettingsRecordSize = 701;
struct LegacySettingsRecord { std::uint8_t bytes[kLegacySettingsRecordSize] = {}; };
constexpr std::size_t kPreviousSettingsRecordSize = 767;
struct PreviousSettingsRecord { std::uint8_t bytes[kPreviousSettingsRecordSize] = {}; };
constexpr std::size_t kSettingsRecordSize = 831;
struct SettingsRecord {
  std::uint8_t bytes[kSettingsRecordSize] = {};
};
static_assert(sizeof(SettingsRecord) == kSettingsRecordSize, "Record must be bytes only");

namespace detail {

constexpr std::size_t kRecordPayloadSize = kSettingsRecordSize - 4;

inline std::uint32_t crc32(const std::uint8_t* bytes, std::size_t size) {
  std::uint32_t crc = 0xffffffffu;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
    }
  }
  return ~crc;
}

inline void writeU16(std::uint8_t* bytes, std::uint16_t value) {
  bytes[0] = static_cast<std::uint8_t>(value);
  bytes[1] = static_cast<std::uint8_t>(value >> 8);
}

inline std::uint16_t readU16(const std::uint8_t* bytes) {
  return static_cast<std::uint16_t>(bytes[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[1]) << 8);
}

inline void writeU32(std::uint8_t* bytes, std::uint32_t value) {
  for (int i = 0; i < 4; ++i) bytes[i] = static_cast<std::uint8_t>(value >> (8 * i));
}

inline std::uint32_t readU32(const std::uint8_t* bytes) {
  std::uint32_t value = 0;
  for (int i = 0; i < 4; ++i) value |= static_cast<std::uint32_t>(bytes[i]) << (8 * i);
  return value;
}

template <std::size_t N>
inline void writeText(std::uint8_t*& bytes, const char (&text)[N]) {
  // The record starts cleared. Serialize only the initialized string content,
  // giving equal strings equal records regardless of unused buffer contents.
  std::memcpy(bytes, text, std::strlen(text));
  bytes += N;
}

template <std::size_t N>
inline void readText(const std::uint8_t*& bytes, char (&text)[N]) {
  std::memcpy(text, bytes, N);
  bytes += N;
}

}  // namespace detail

// An all-zero Station represents "no last good station". Invalid inputs produce
// an invalid, zeroed record; callers should validate settings/stations first.
inline SettingsRecord makeSettingsRecord(const Settings& settings, const Station& lastGood) {
  SettingsRecord record;
  if (!validSettings(settings) ||
      (!detail::emptyStation(lastGood) && !validStation(lastGood))) return record;
  record.bytes[0] = 'R';
  record.bytes[1] = 'A';
  record.bytes[2] = 'D';
  record.bytes[3] = 'I';
  record.bytes[4] = 3;
  record.bytes[5] = settings.volume;
  record.bytes[6] = settings.muted ? 1 : 0;
  record.bytes[7] = settings.country;
  record.bytes[8] = settings.genre;
  record.bytes[9] = settings.codec;
  detail::writeU16(record.bytes + 10, settings.bitrateCap);
  record.bytes[12] = static_cast<std::uint8_t>(settings.source);
  std::uint8_t* bytes = record.bytes + 13;
  detail::writeText(bytes, settings.airLocation);
  *bytes++ = static_cast<std::uint8_t>(lastGood.source);
  detail::writeText(bytes, lastGood.id);
  detail::writeText(bytes, lastGood.name);
  detail::writeText(bytes, lastGood.url);
  detail::writeText(bytes, lastGood.country);
  detail::writeU16(bytes, lastGood.bitrateKbps);
  bytes += 2;
  detail::writeText(bytes, lastGood.location);
  detail::writeU32(record.bytes + detail::kRecordPayloadSize,
                   detail::crc32(record.bytes, detail::kRecordPayloadSize));
  return record;
}

// Outputs remain untouched unless the complete versioned record is valid.
inline bool decodeSettingsRecord(const SettingsRecord& record, Settings& outSettings,
                                 Station& outStation) {
  if (record.bytes[0] != 'R' || record.bytes[1] != 'A' || record.bytes[2] != 'D' ||
      record.bytes[3] != 'I' || record.bytes[4] != 3 || record.bytes[6] > 1 ||
      detail::readU32(record.bytes + detail::kRecordPayloadSize) !=
          detail::crc32(record.bytes, detail::kRecordPayloadSize)) return false;

  Settings settings;
  settings.volume = record.bytes[5];
  settings.muted = record.bytes[6] == 1;
  settings.country = record.bytes[7];
  settings.genre = record.bytes[8];
  settings.codec = record.bytes[9];
  settings.bitrateCap = detail::readU16(record.bytes + 10);
  Station station;
  settings.source = static_cast<Source>(record.bytes[12]);
  const std::uint8_t* bytes = record.bytes + 13;
  detail::readText(bytes, settings.airLocation);
  station.source = static_cast<Source>(*bytes++);
  detail::readText(bytes, station.id);
  detail::readText(bytes, station.name);
  detail::readText(bytes, station.url);
  detail::readText(bytes, station.country);
  station.bitrateKbps = detail::readU16(bytes);
  bytes += 2;
  detail::readText(bytes, station.location);
  if (!validSettings(settings) ||
      (!detail::emptyStation(station) && !validStation(station))) return false;

  outSettings = settings;
  outStation = station;
  return true;
}

// Decode the previous source-aware 767-byte record; its station location was absent.
inline bool decodeSettingsRecord(const PreviousSettingsRecord& record, Settings& outSettings,
                                 Station& outStation) {
  if (record.bytes[0] != 'R' || record.bytes[1] != 'A' || record.bytes[2] != 'D' ||
      record.bytes[3] != 'I' || record.bytes[4] != 2 || record.bytes[6] > 1 ||
      detail::readU32(record.bytes + (kPreviousSettingsRecordSize - 4)) !=
          detail::crc32(record.bytes, (kPreviousSettingsRecordSize - 4))) return false;

  Settings settings;
  settings.volume = record.bytes[5];
  settings.muted = record.bytes[6] == 1;
  settings.country = record.bytes[7];
  settings.genre = record.bytes[8];
  settings.codec = record.bytes[9];
  settings.bitrateCap = detail::readU16(record.bytes + 10);
  Station station;
  settings.source = static_cast<Source>(record.bytes[12]);
  const std::uint8_t* bytes = record.bytes + 13;
  detail::readText(bytes, settings.airLocation);
  station.source = static_cast<Source>(*bytes++);
  detail::readText(bytes, station.id);
  detail::readText(bytes, station.name);
  detail::readText(bytes, station.url);
  detail::readText(bytes, station.country);
  station.bitrateKbps = detail::readU16(bytes);
  if (!validSettings(settings) ||
      (!detail::emptyStation(station) && !validStation(station))) return false;

  outSettings = settings;
  outStation = station;
  return true;
}

// Decode the original 701-byte state without depending on current struct layout.
inline bool decodeSettingsRecord(const LegacySettingsRecord& record, Settings& outSettings,
                                 Station& outStation) {
  constexpr std::size_t payload = kLegacySettingsRecordSize - 4;
  if (record.bytes[0] != 'R' || record.bytes[1] != 'A' || record.bytes[2] != 'D' ||
      record.bytes[3] != 'I' || record.bytes[4] != 1 || record.bytes[6] > 1 ||
      detail::readU32(record.bytes + payload) != detail::crc32(record.bytes, payload)) return false;
  Settings settings;
  settings.volume = record.bytes[5];
  settings.muted = record.bytes[6] == 1;
  settings.country = record.bytes[7];
  settings.genre = record.bytes[8];
  settings.codec = record.bytes[9];
  settings.bitrateCap = detail::readU16(record.bytes + 10);
  Station station;
  const std::uint8_t* bytes = record.bytes + 12;
  detail::readText(bytes, station.id);
  detail::readText(bytes, station.name);
  detail::readText(bytes, station.url);
  detail::readText(bytes, station.country);
  station.bitrateKbps = detail::readU16(bytes);
  if (!validSettings(settings) ||
      (!detail::emptyStation(station) && !validStation(station))) return false;
  outSettings = settings;
  outStation = station;
  return true;
}

}  // namespace RadioModel
