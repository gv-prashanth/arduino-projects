#pragma once

#include "RadioModel.h"

namespace RadioConfig {
constexpr int I2sLrck = 4, I2sBclk = 5, I2sData = 6;
constexpr int TftCs = 10, TftDc = 13, TftReset = 14, TftMosi = 11, TftClock = 12;
constexpr int EncoderClock = 1, EncoderData = 2, EncoderButton = 42;
constexpr uint32_t MenuTimeoutMs = 15000, EditTimeoutMs = 30000;
constexpr uint32_t SaveDelayMs = 1500;

struct Country { const char* code; const char* name; };
constexpr Country Countries[] = {
  {"", "All countries"}, {"US", "USA"}, {"IN", "India"}, {"GB", "UK"},
  {"DE", "Germany"}, {"FR", "France"}, {"JP", "Japan"}, {"CA", "Canada"},
  {"AU", "Australia"}, {"IT", "Italy"}, {"ES", "Spain"}, {"BR", "Brazil"},
  {"MX", "Mexico"}, {"NL", "Netherlands"}, {"SE", "Sweden"}, {"NO", "Norway"},
  {"ZA", "South Africa"}, {"SG", "Singapore"}, {"AE", "UAE"}
};
struct Genre { const char* tag; const char* name; };
constexpr Genre Genres[] = {
  {"", "All genres"}, {"music", "Music"}, {"news", "News"}, {"jazz", "Jazz"},
  {"classical", "Classical"}, {"indian classical", "Indian classical"}, {"rock", "Rock"}
};
constexpr const char* Codecs[] = {"Any supported", "AAC", "MP3"};
constexpr uint16_t BitrateCaps[] = {0, 64, 96, 128, 192, 256, 320};
constexpr int BitrateCapCount = sizeof(BitrateCaps) / sizeof(BitrateCaps[0]);
static_assert(sizeof(Countries) / sizeof(Countries[0]) == RadioModel::kCountryCount, "Country model mismatch");
static_assert(sizeof(Genres) / sizeof(Genres[0]) == RadioModel::kGenreCount, "Genre model mismatch");
}
