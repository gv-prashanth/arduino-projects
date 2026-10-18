#include "../RadioModel.h"

#include <cassert>
#include <climits>
#include <cstdio>
#include <cstring>

using namespace RadioModel;

static Station station(const char* id, const char* name, const char* url,
                       const char* country = "IN") {
  Station result;
  assert(copyText(result.id, id));
  assert(copyText(result.name, name));
  assert(copyText(result.url, url));
  assert(copyText(result.country, country));
  result.bitrateKbps = 128;
  assert(validStation(result));
  return result;
}

static void refreshChecksum(SettingsRecord& record) {
  detail::writeU32(record.bytes + detail::kRecordPayloadSize,
                   detail::crc32(record.bytes, detail::kRecordPayloadSize));
}

static void expectRejected(const SettingsRecord& record) {
  Settings settings;
  settings.volume = 11;
  settings.muted = true;
  const Station original = station("untouched", "Keep this", "https://example.org/keep");
  Station playing = original;
  assert(!decodeSettingsRecord(record, settings, playing));
  assert(settings.volume == 11 && settings.muted);
  assert(std::strcmp(playing.id, original.id) == 0);
  assert(std::strcmp(playing.url, original.url) == 0);
}

static void testVolumeAndRestart() {
  Settings settings;
  assert(settings.volume == 6 && !settings.muted && settings.bitrateCap == 0);
  settings.volume = 7;
  toggleMute(settings);
  assert(outputVolume(settings) == 0 && settings.volume == 7);
  const Station original = station("saved", "Saved radio", "https://example.org/live");
  const SettingsRecord record = makeSettingsRecord(settings, original);
  Settings restored;
  Station playing;
  assert(decodeSettingsRecord(record, restored, playing));
  assert(restored.volume == 7 && restored.muted && outputVolume(restored) == 0);
  assert(std::strcmp(playing.url, original.url) == 0);
  toggleMute(restored);
  assert(outputVolume(restored) == 7);

  toggleMute(restored);
  adjustVolume(restored, 1);
  assert(!restored.muted && restored.volume == 8 && outputVolume(restored) == 8);
  adjustVolume(restored, INT_MAX);
  assert(restored.volume == kMaxVolume);
  restored.muted = true;
  adjustVolume(restored, 1);
  assert(!restored.muted && restored.volume == kMaxVolume);
  adjustVolume(restored, INT_MIN);
  assert(restored.volume == 0 && outputVolume(restored) == 0);
  restored.muted = true;
  adjustVolume(restored, -1);
  assert(!restored.muted && restored.volume == 0);
}

static void testRecordsAndValidation() {
  const std::uint8_t crcExample[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  assert(detail::crc32(crcExample, sizeof(crcExample)) == 0xcbf43926u);

  Settings settings;
  settings.volume = 21;
  settings.country = 18;
  settings.genre = 6;
  settings.codec = 2;
  settings.bitrateCap = 65535;
  assert(validSettings(settings));
  const Station original = station("id-1", "Unicode radio: \xe0\xa4\xb0", "HTTP://example.org/live", "US");
  SettingsRecord good = makeSettingsRecord(settings, original);
  Settings restored;
  Station playing;
  assert(decodeSettingsRecord(good, restored, playing));
  assert(restored.volume == 21 && restored.country == 18 && restored.genre == 6 &&
         restored.codec == 2 && restored.bitrateCap == 65535);
  assert(playing.bitrateKbps == 128 && std::strcmp(playing.name, original.name) == 0);

  SettingsRecord bad = good;
  bad.bytes[200] ^= 1;
  expectRejected(bad);
  bad = good;
  bad.bytes[4] = 2;
  refreshChecksum(bad);
  expectRejected(bad);
  bad = good;
  bad.bytes[0] = 'X';
  refreshChecksum(bad);
  expectRejected(bad);

  const int fields[] = {5, 6, 7, 8, 9};
  const std::uint8_t invalidValues[] = {22, 2, 19, 7, 3};
  for (std::size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); ++i) {
    bad = good;
    bad.bytes[fields[i]] = invalidValues[i];
    refreshChecksum(bad);
    expectRejected(bad);
  }

  // All four station strings must be terminated even when the record CRC is good.
  const std::size_t starts[] = {12, 52, 180, 692};
  const std::size_t sizes[] = {40, 128, 512, 3};
  for (std::size_t i = 0; i < 4; ++i) {
    bad = good;
    std::memset(bad.bytes + starts[i], 'X', sizes[i]);
    refreshChecksum(bad);
    expectRejected(bad);
  }

  Station invalid = original;
  assert(copyText(invalid.url, "https://"));
  assert(!validStation(invalid));
  assert(copyText(invalid.url, "https://example.org/bad path"));
  assert(!validStation(invalid));
  assert(copyText(invalid.url, "file:///etc/passwd"));
  assert(!validStation(invalid));
  assert(copyText(invalid.url, original.url));
  assert(copyText(invalid.country, "I"));
  assert(!validStation(invalid));
  assert(copyText(invalid.country, "in"));
  assert(!validStation(invalid));
  assert(copyText(invalid.country, ""));
  assert(validStation(invalid));

  Station absent;
  assert(!validStation(absent));
  good = makeSettingsRecord(Settings{}, absent);
  assert(decodeSettingsRecord(good, restored, playing));
  assert(playing.id[0] == '\0' && playing.url[0] == '\0' && playing.bitrateKbps == 0);
  Settings invalidSettings;
  invalidSettings.volume = 22;
  expectRejected(makeSettingsRecord(invalidSettings, original));
  invalid = original;
  std::memset(invalid.url, 'x', sizeof(invalid.url));
  expectRejected(makeSettingsRecord(Settings{}, invalid));
}

static void testCatalogIdentityAndPreviews() {
  Station list[3];
  list[0] = station("different", "URL match", "https://example.org/old");
  list[1] = station("same-id", "ID match", "https://example.org/new");
  list[2] = station("third", "Third", "https://example.org/third");
  Station playing = station("same-id", "Saved", "https://example.org/old");
  assert(findStation(list, 3, playing) == 1);
  assert(copyText(playing.id, "not-in-list"));
  assert(findStation(list, 3, playing) == 0);
  assert(copyText(playing.url, "https://example.org/absent"));
  assert(findStation(list, 3, playing) == -1);
  assert(findStation(nullptr, 0, playing) == -1);
  assert(findStation(list, 0, playing) == -1);
  assert(findStation(list, kMaxStations + 1, playing) == -1);
  assert(findStation(list, 3, Station{}) == -1);
  std::memset(playing.id, 'x', sizeof(playing.id));
  std::memset(playing.url, 'x', sizeof(playing.url));
  assert(findStation(list, 3, playing) == -1);

  assert(previewIndex(15, 0) == -1);
  assert(previewIndex(15, -1) == -1);
  assert(previewIndex(15, 3) == 2);
  assert(previewIndex(-1, 3) == 0);
  assert(previewIndex(0, 1) == 0);
  assert(previewIndex(2, 3) == 2);
}

static void testCopiesAndCanonicalEncoding() {
  char text[4];
  assert(copyText(text, "abc"));
  assert(std::strcmp(text, "abc") == 0);
  assert(!copyText(text, "abcd"));
  assert(std::strcmp(text, "abc") == 0);
  assert(!copyText(text, nullptr));
  assert(text[0] == '\0' && text[3] == '\0');

  Station a = station("id", "Name", "https://example.org/live");
  Station b = a;
  b.name[100] = 'x';
  b.url[500] = 'y';
  SettingsRecord first = makeSettingsRecord(Settings{}, a);
  SettingsRecord second = makeSettingsRecord(Settings{}, b);
  assert(std::memcmp(first.bytes, second.bytes, sizeof(first.bytes)) == 0);
}

int main() {
  testVolumeAndRestart();
  testRecordsAndValidation();
  testCatalogIdentityAndPreviews();
  testCopiesAndCanonicalEncoding();
  std::puts("RadioModel tests passed");
}
