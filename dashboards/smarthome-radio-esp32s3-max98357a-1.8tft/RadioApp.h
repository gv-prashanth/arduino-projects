#pragma once

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "AudioService.h"
#include "SettingsService.h"
#include "WifiPortal.h"
#include "RotaryInput.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

namespace RadioApp {
using RadioModel::Station;
using RadioModel::Settings;
using RadioCatalog::Filter;
enum class View : uint8_t { Listening, Menu, Stations, Edit, Wifi };
enum MenuItem : uint8_t { Stations, Source, Location, Genre, Codec, Bitrate, Wifi, Retry, MenuCount };
constexpr const char* MenuLabels[] = {"Stations", "Source", "Location", "Genre", "Codec", "Max bitrate", "Wi-Fi setup", "Retry audio"};

Adafruit_ST7735 tft(RadioConfig::TftCs, RadioConfig::TftDc, RadioConfig::TftReset);
GFXcanvas16 canvas(160, 128);
RadioAudio::AudioService audio;
RadioCatalog::CatalogService directory;
RadioStorage::SettingsService storage;
RadioWifi::WifiManager wifi;
RadioInput::RotaryInput input;
Settings settings;
Settings editSettings;
Station lastGood, confirmationCandidate;
RadioAudio::Snapshot playback;
RadioCatalog::Result* catalog = nullptr;
RadioCatalog::Transaction search;
View view = View::Listening;
MenuItem editItem = Location;
int menuIndex = 0, draft = 0, stationCursor = 0;
// The official location menu is a snapshot, so turning the encoder never starts
// network requests or depends on a worker-owned catalog's lifetime.
uint8_t airLocationCount = 0;
char airLocations[64][64] = {};
bool airLocationsReady = false;
bool audioReady = false, directoryReady = false, storageReady = false, inputReady = false;
bool dirty = true, animate = false, settingsDirty = false;
bool online = false, autoStartFirst = true;
bool portalWasActive = false;
bool eventButtonDown = false;
View gestureView = View::Listening;
uint32_t audioGeneration = 0;
uint32_t dirtySince = 0, lastDrawAt = 0, viewEnteredAt = 0, lastActionAt = 0;
uint32_t candidateSince = 0, lastOverflow = 0;
char notice[96] = {}, initializationError[80] = {};
char lastWifiStatus[80] = {};
uint32_t noticeUntil = 0;

inline bool due(uint32_t now, uint32_t deadline) { return static_cast<int32_t>(now - deadline) >= 0; }
inline void notify(const char* message, uint32_t duration = 5000) {
  strlcpy(notice, message, sizeof(notice));
  noticeUntil = millis() + duration;
  Serial.printf("radio: %s\n", message);
  dirty = true;
}
inline void markSettingsDirty() { settingsDirty = true; dirtySince = millis(); }
inline void enter(View next) { view = next; viewEnteredAt = lastActionAt = millis(); dirty = true; }
inline const char* sourceLabel(RadioModel::Source source) {
  return source == RadioModel::Source::Akashvani ? "Akashvani" : "Radio Browser";
}
inline bool menuItemVisible(MenuItem item) {
  return settings.source != RadioModel::Source::Akashvani || (item != Genre && item != Codec && item != Bitrate);
}
inline int visibleMenuCount() {
  int count = 0;
  for (int item = 0; item < MenuCount; ++item) if (menuItemVisible(static_cast<MenuItem>(item))) ++count;
  return count;
}
inline MenuItem visibleMenuItem(int index) {
  for (int item = 0; item < MenuCount; ++item) {
    if (!menuItemVisible(static_cast<MenuItem>(item))) continue;
    if (index-- == 0) return static_cast<MenuItem>(item);
  }
  return Stations;
}
inline void rememberAirLocations(const RadioCatalog::Result& result) {
  if (result.request.filter.source != RadioModel::Source::Akashvani || result.cached) return;
  airLocationCount = min<unsigned>(result.locationCount, sizeof(airLocations) / sizeof(airLocations[0]));
  memset(airLocations, 0, sizeof(airLocations));
  for (int i = 0; i < airLocationCount; ++i)
    strlcpy(airLocations[i], result.locations[i], sizeof(airLocations[i]));
  airLocationsReady = true;
}

// Decode UTF-8 as codepoints before fitting text. The classic TFT font is ASCII;
// common Latin accents/punctuation are transliterated, other glyphs become one '?'.
inline String displayText(const char* raw) {
  String out;
  if (!raw) return out;
  const unsigned char* p = reinterpret_cast<const unsigned char*>(raw);
  while (*p) {
    uint32_t cp = *p++;
    if (cp >= 0x80) {
      int extra = cp >= 0xF0 ? 3 : cp >= 0xE0 ? 2 : cp >= 0xC0 ? 1 : 0;
      cp &= extra == 3 ? 7 : extra == 2 ? 15 : extra == 1 ? 31 : 127;
      bool valid = extra != 0;
      for (int i = 0; i < extra; ++i) {
        if (!*p || (*p & 0xC0) != 0x80) { valid = false; break; }
        cp = (cp << 6) | (*p++ & 63);
      }
      if (!valid) cp = '?';
    }
    if (cp >= 32 && cp <= 126) out += char(cp);
    else if (cp == 0x2013 || cp == 0x2014) out += '-';
    else if (cp == 0x2018 || cp == 0x2019) out += '\'';
    else if (cp == 0x201C || cp == 0x201D) out += '"';
    else if (cp == 0x00A0) out += ' ';
    else if ((cp >= 0xC0 && cp <= 0xC5) || (cp >= 0xE0 && cp <= 0xE5)) out += (cp < 0xE0 ? 'A' : 'a');
    else if ((cp >= 0xC8 && cp <= 0xCB) || (cp >= 0xE8 && cp <= 0xEB)) out += (cp < 0xE0 ? 'E' : 'e');
    else if ((cp >= 0xCC && cp <= 0xCF) || (cp >= 0xEC && cp <= 0xEF)) out += (cp < 0xE0 ? 'I' : 'i');
    else if ((cp >= 0xD2 && cp <= 0xD6) || (cp >= 0xF2 && cp <= 0xF6)) out += (cp < 0xE0 ? 'O' : 'o');
    else if ((cp >= 0xD9 && cp <= 0xDC) || (cp >= 0xF9 && cp <= 0xFC)) out += (cp < 0xE0 ? 'U' : 'u');
    else if (cp == 0xD1 || cp == 0xF1) out += (cp == 0xD1 ? 'N' : 'n');
    else if (cp == 0xC7 || cp == 0xE7) out += (cp == 0xC7 ? 'C' : 'c');
    else out += cp < 32 ? ' ' : '?';
  }
  return out;
}
inline void text(int x, int y, int width, const char* raw, uint16_t color = ST77XX_WHITE,
                 uint8_t size = 1, bool scroll = false) {
  String value = displayText(raw);
  const int cells = width / (6 * size);
  if (cells <= 0) return;
  if (value.length() > static_cast<unsigned>(cells)) {
    if (scroll) {
      animate = true;
      String ring = value + "    " + value;
      const unsigned offset = (millis() / 350) % (value.length() + 4);
      value = ring.substring(offset, offset + cells);
    } else value = value.substring(0, cells > 1 ? cells - 1 : cells) + (cells > 1 ? "~" : "");
  }
  canvas.setTextWrap(false);
  canvas.setTextSize(size);
  canvas.setTextColor(color);
  canvas.setCursor(x, y);
  canvas.print(value);
}
inline void header(const char* title) {
  canvas.fillRect(0, 0, 160, 16, 0x2808);
  text(4, 4, 100, title, ST77XX_GREEN);
  char right[12];
  snprintf(right, sizeof(right), "%s%02u %c", settings.muted ? "M" : "V", settings.volume, online ? '+' : '-');
  text(112, 4, 48, right, settings.muted ? ST77XX_YELLOW : ST77XX_WHITE);
}
inline void footer(const char* hint) {
  canvas.drawFastHLine(0, 110, 160, 0x7BEF);
  text(3, 115, 150, hint, ST77XX_CYAN);
  if (inputReady && input.buttonDown()) {
    const uint32_t pressedAt = input.pressedAt();
    const uint32_t held = millis() - pressedAt;
    canvas.fillRect(0, 108, min<uint32_t>(160, held * 160 / 700), 2, ST77XX_GREEN);
    animate = true;
  }
}
inline String filterLabel(const Filter& f) {
  if (f.source == RadioModel::Source::Akashvani)
    return String("India / ") + (f.location[0] ? f.location : "All locations");
  return String(f.country ? RadioConfig::Countries[f.country].code : "ALL") + " / " + RadioConfig::Genres[f.genre].name;
}
inline int stationOptions() {
  if (!catalog) return 1;
  return catalog->count + (catalog->request.offset ? 1 : 0) + (catalog->hasNext ? 1 : 0) + 1;
}
inline String stationOptionLabel(int index) {
  if (catalog && index < catalog->count) return catalog->stations[index].name;
  int action = index - (catalog ? catalog->count : 0);
  if (catalog && catalog->request.offset) { if (action == 0) return "< Previous page"; --action; }
  if (catalog && catalog->hasNext) { if (action == 0) return "More stations >"; --action; }
  if (search.pending) return "Loading stations...";
  if (search.failed) return String("Retry ") + sourceLabel(search.request.filter.source);
  return "Refresh stations";
}
inline void drawListening() {
  header("GTS RADIO");
  const char* status = !online ? wifi.status() : settings.muted && playback.state == RadioAudio::State::Playing
                       ? "Muted (stream playing)" : RadioAudio::stateName(playback.state);
  text(4, 21, 152, status, playback.state == RadioAudio::State::Failed ? ST77XX_RED : ST77XX_YELLOW);
  const Station& displayed = RadioModel::validStation(playback.requested) ? playback.requested : lastGood;
  text(4, 38, 152, displayed.name[0] ? displayed.name : "GTS Radio", ST77XX_WHITE, 2, true);
  char technical[64];
  if (playback.bitrateKbps) snprintf(technical, sizeof(technical), "%s %u kbps", playback.codec, playback.bitrateKbps);
  else snprintf(technical, sizeof(technical), "%s", playback.codec[0] && strcmp(playback.codec, "unknown") ? playback.codec : "Audio not ready");
  text(4, 61, 152, technical, ST77XX_CYAN);
  const bool showingNotice = notice[0] && !due(millis(), noticeUntil);
  text(4, 76, 152, showingNotice ? notice : playback.title[0] ? playback.title : playback.detail, ST77XX_WHITE, 1, true);
  // Browsing another source does not retune playback. Identify the displayed
  // station's provider rather than presenting the current catalog's filters.
  const bool hasStation = RadioModel::validStation(displayed);
  String provider = hasStation ? sourceLabel(displayed.source) : "No station selected";
  if (hasStation && displayed.country[0]) provider += String(" / ") + displayed.country;
  text(4, 94, 152, provider.c_str(), ST77XX_GREEN, 1, true);
  footer((millis() / 4000) % 2 ? "Hold: menu" : "Turn:vol  Click:mute");
  animate = true; // Alternate complete hints even when metadata is unchanged.
}
inline void drawMenu() {
  header("Menu");
  const int count = visibleMenuCount();
  menuIndex = constrain(menuIndex, 0, count - 1);
  const int first = constrain(menuIndex - 2, 0, max(0, count - 6));
  for (int row = 0; row < 6 && first + row < count; ++row) {
    const int index = first + row, y = 20 + row * 14;
    const MenuItem item = visibleMenuItem(index);
    const bool selected = menuIndex == index;
    if (selected) canvas.fillRect(2, y - 1, 156, 13, 0x0320);
    char label[64];
    if (item == Source) snprintf(label, sizeof(label), "%c Source: %s", selected ? '>' : ' ', sourceLabel(settings.source));
    else snprintf(label, sizeof(label), "%c %s", selected ? '>' : ' ', MenuLabels[item]);
    text(4, y, 152, label, ST77XX_WHITE, 1, selected);
  }
  footer("Click:select Hold:back");
}
inline void drawStations() {
  char title[32];
  const Filter filter = catalog ? catalog->request.filter : RadioCatalog::filterFor(settings);
  const unsigned page = catalog ? catalog->request.offset / RadioModel::kPageSize + 1 : 1;
  const bool totalKnown = catalog && catalog->totalKnown && !catalog->cached;
  if (catalog && catalog->cached)
    snprintf(title, sizeof(title), "%s cached", filter.source == RadioModel::Source::Akashvani ? "Akashvani" : "RB");
  else if (totalKnown && catalog->totalCount) {
    const unsigned pages = (catalog->totalCount + RadioModel::kPageSize - 1) / RadioModel::kPageSize;
    snprintf(title, sizeof(title), "%s p%u/%u", filter.source == RadioModel::Source::Akashvani ? "Akashvani" : "RB", page, pages);
  } else snprintf(title, sizeof(title), "%s p%u", filter.source == RadioModel::Source::Akashvani ? "Akashvani" : "RB", page);
  header(title);
  String filters = filterLabel(filter);
  text(4, 20, 152, filters.c_str(), ST77XX_GREEN, 1, true);
  const int options = stationOptions();
  stationCursor = constrain(stationCursor, 0, options - 1);
  char summary[80];
  if (search.pending) snprintf(summary, sizeof(summary), "Loading %s...", sourceLabel(search.request.filter.source));
  else if (search.failed) snprintf(summary, sizeof(summary), "%s unavailable", sourceLabel(search.request.filter.source));
  else if (!catalog || !catalog->count) snprintf(summary, sizeof(summary), "No stations; edit filters");
  else if (stationCursor < catalog->count) {
    if (totalKnown) snprintf(summary, sizeof(summary), "Station %u/%u; %u here",
                             catalog->request.offset + stationCursor + 1, catalog->totalCount, catalog->count);
    else snprintf(summary, sizeof(summary), "%u %s; station %d/%u", catalog->count,
                  catalog->cached ? "cached" : "here", stationCursor + 1, catalog->count);
  } else {
    int action = stationCursor - catalog->count;
    const char* label = "Refresh";
    if (catalog->request.offset && action-- == 0) label = "Prev page";
    else if (catalog->hasNext && action == 0) label = "Next page";
    snprintf(summary, sizeof(summary), "%u %s; %s", totalKnown ? catalog->totalCount : catalog->count,
             totalKnown ? "total" : catalog->cached ? "cached" : "here", label);
  }
  text(4, 33, 152, summary, ST77XX_YELLOW);
  int first = constrain(stationCursor - 1, 0, max(0, options - 4));
  for (int row = 0; row < 4 && first + row < options; ++row) {
    int index = first + row, y = 49 + row * 14;
    const bool selected = index == stationCursor;
    if (selected) canvas.fillRect(2, y - 2, 156, 12, 0x0320);
    const bool playing = catalog && index < catalog->count &&
                         RadioModel::findStation(&catalog->stations[index], 1, playback.confirmed) == 0;
    text(4, y, 8, playing ? "*" : selected ? ">" : " ", ST77XX_YELLOW);
    String label = stationOptionLabel(index);
    text(14, y, 142, label.c_str(), ST77XX_WHITE, 1, selected);
  }
  footer(catalog && stationCursor < catalog->count ? "Click:play Hold:back" : "Click:load Hold:back");
}
inline const char* draftLabel(String& scratch) {
  if (editItem == Source) return sourceLabel(static_cast<RadioModel::Source>(draft));
  if (editItem == Location) {
    if (editSettings.source == RadioModel::Source::Akashvani)
      return draft > 0 && draft <= airLocationCount ? airLocations[draft - 1] : "All locations";
    return draft ? RadioConfig::Countries[draft].name : "All locations";
  }
  if (editItem == Genre) return RadioConfig::Genres[draft].name;
  if (editItem == Codec) return RadioConfig::Codecs[draft];
  scratch = draft ? String(RadioConfig::BitrateCaps[draft]) + " kbps" : "No bitrate cap";
  return scratch.c_str();
}
inline void drawEdit() {
  header(MenuLabels[editItem]);
  const bool missingLocations = editItem == Location && editSettings.source == RadioModel::Source::Akashvani && !airLocationsReady;
  text(4, 26, 152, missingLocations ? "All until source refresh" : "Turn to choose", ST77XX_CYAN);
  String scratch;
  text(4, 49, 152, draftLabel(scratch), ST77XX_YELLOW, 1, true);
  text(4, 74, 152, "Playback continues");
  text(4, 90, 152, "Applied after search", ST77XX_GREEN);
  footer("Click:apply Hold:cancel");
}
inline void drawWifi() {
  header("Wi-Fi setup");
  text(4, 22, 152, wifi.status(), ST77XX_YELLOW, 1, true);
  if (!wifi.portalActive()) {
    text(4, 48, 152, "Setup AP is restarting");
    text(4, 68, 152, "Retries within 60 seconds", ST77XX_CYAN);
    footer("Hold:back");
    return;
  }
  text(4, 38, 152, "Connect phone to:", ST77XX_CYAN);
  text(4, 51, 152, wifi.apName());
  String password = "Key: " + String(wifi.apPassword());
  text(4, 65, 152, password.c_str());
  String address = "Open " + wifi.portalAddress();
  text(4, 81, 152, address.c_str(), ST77XX_GREEN);
  text(4, 96, 152, "Enter home Wi-Fi there");
  footer("Hold:back");
}
inline void draw() {
  if (!canvas.getBuffer()) return;
  animate = false;
  canvas.fillScreen(ST77XX_BLACK);
  if (initializationError[0]) {
    header("Startup error");
    text(4, 28, 152, initializationError, ST77XX_RED, 1, true);
    text(4, 60, 152, "See USB serial log");
    footer("Restart after correction");
  } else switch (view) {
    case View::Listening: drawListening(); break;
    case View::Menu: drawMenu(); break;
    case View::Stations: drawStations(); break;
    case View::Edit: drawEdit(); break;
    case View::Wifi: drawWifi(); break;
  }
  tft.drawRGBBitmap(0, 0, canvas.getBuffer(), 160, 128);
  lastDrawAt = millis();
  dirty = false;
}

inline void startStation(const Station& station) {
  if (!audioReady || !RadioModel::validStation(station)) { notify("Invalid station or audio unavailable"); return; }
  const uint32_t generation = ++audioGeneration;
  if (!audio.start(station, lastGood, generation)) { notify("Cannot queue station"); return; }
  playback.requested = station;
  playback.generation = generation;
  playback.state = online ? RadioAudio::State::Connecting : RadioAudio::State::Offline;
  playback.title[0] = playback.codec[0] = 0;
  playback.bitrateKbps = 0;
  strlcpy(playback.detail, online ? "Starting station" : "Will resume when Wi-Fi connects", sizeof(playback.detail));
  candidateSince = 0;
  autoStartFirst = false;
  dirty = true;
}
inline void requestDirectory(const Filter& filter, uint16_t offset, bool refresh = false) {
  if (!directoryReady) { notify("Station service unavailable"); return; }
  const RadioCatalog::Request request = search.begin(filter, offset, millis(), refresh);
  if (!directory.request(request)) {
    search.fail(request.generation, millis());
    notify("Cannot queue station search");
    return;
  }
  dirty = true;
}
inline void showStations() {
  int playing = catalog ? RadioModel::findStation(catalog->stations, catalog->count, playback.confirmed) : -1;
  stationCursor = playing >= 0 ? playing : 0;
  enter(View::Stations);
  if (!catalog && !search.pending && !search.failed) requestDirectory(RadioCatalog::filterFor(settings), 0);
}
inline void beginEdit(MenuItem item) {
  // A pending search can finish while this editor is open. Keep the draft's
  // provider and options stable until Apply or Cancel.
  editSettings = settings;
  editItem = item;
  if (item == Source) draft = static_cast<int>(editSettings.source);
  else if (item == Location) {
    if (editSettings.source == RadioModel::Source::Akashvani) {
      if (catalog) rememberAirLocations(*catalog);
      draft = 0;
      for (int i = 0; i < airLocationCount; ++i)
        if (!strcmp(editSettings.airLocation, airLocations[i])) draft = i + 1;
    } else draft = editSettings.country;
  }
  else if (item == Genre) draft = editSettings.genre;
  else if (item == Codec) draft = editSettings.codec;
  else {
    draft = 0;
    for (int i = 0; i < RadioConfig::BitrateCapCount; ++i) if (editSettings.bitrateCap == RadioConfig::BitrateCaps[i]) draft = i;
  }
  enter(View::Edit);
}
inline void applyDraft() {
  Filter candidate = RadioCatalog::filterFor(editSettings);
  if (editItem == Source) {
    Settings sourceSettings = editSettings;
    sourceSettings.source = static_cast<RadioModel::Source>(draft);
    // Before the official location catalog has been loaded, All is the only
    // selectable location. Existing Radio Browser filters remain remembered.
    if (sourceSettings.source == RadioModel::Source::Akashvani &&
        editSettings.source != RadioModel::Source::Akashvani && !airLocationsReady)
      sourceSettings.airLocation[0] = '\0';
    candidate = RadioCatalog::filterFor(sourceSettings);
  } else if (editItem == Location) {
    if (editSettings.source == RadioModel::Source::Akashvani)
      strlcpy(candidate.location, draft > 0 && draft <= airLocationCount ? airLocations[draft - 1] : "", sizeof(candidate.location));
    else candidate.country = draft;
  }
  else if (editItem == Genre) candidate.genre = draft;
  else if (editItem == Codec) candidate.codec = draft;
  else candidate.bitrateCap = RadioConfig::BitrateCaps[draft];
  // Keep the committed settings, catalog, and currently playing station intact until a valid response.
  // A re-applied committed filter must supersede an older pending/retrying
  // request too; otherwise that older response could later replace this choice.
  if (search.shouldApply(RadioCatalog::filterFor(settings), candidate, catalog != nullptr))
    requestDirectory(candidate, 0);
  stationCursor = 0;
  enter(View::Stations);
}
inline void chooseStationOption() {
  if (catalog && stationCursor < catalog->count) {
    startStation(catalog->stations[stationCursor]); enter(View::Listening); return;
  }
  if (search.pending) { notify("Search in progress"); return; }
  int action = stationCursor - (catalog ? catalog->count : 0);
  const Filter filter = catalog ? catalog->request.filter : RadioCatalog::filterFor(settings);
  if (catalog && catalog->request.offset) {
    if (action == 0) { requestDirectory(filter, catalog->request.offset - RadioModel::kPageSize); return; }
    --action;
  }
  if (catalog && catalog->hasNext) {
    if (action == 0) {
      if (catalog->request.offset > UINT16_MAX - RadioModel::kPageSize) notify("End of supported page range");
      else requestDirectory(filter, catalog->request.offset + RadioModel::kPageSize);
      return;
    }
    --action;
  }
  if (search.failed) requestDirectory(search.request.filter, search.request.offset, true);
  else requestDirectory(filter, catalog ? catalog->request.offset : 0, true);
}
inline int wrap(int current, int delta, int count) { return ((current + delta) % count + count) % count; }
inline void handleInput(const RadioInput::Event& event) {
  lastActionAt = millis();
  dirty = true;
  using RadioInput::EventType;
  if (event.type == EventType::ButtonDown) { eventButtonDown = true; gestureView = view; return; }
  if (event.type == EventType::ButtonUp) { eventButtonDown = false; return; }
  if ((event.type == EventType::ShortPress || event.type == EventType::LongPress) && gestureView != view) return;
  if (event.type == EventType::LongPress) {
    if (view == View::Listening) enter(View::Menu);
    else if (view == View::Menu) enter(View::Listening);
    else enter(View::Menu); // Draft is discarded; next entry always starts from committed settings.
    return;
  }
  if (event.type == EventType::Rotate) {
    // Use historical ordered edges here, not the live task snapshot. A held
    // turn may be processed after the user has already released the button.
    if (eventButtonDown) return;
    if (view == View::Listening) {
      Settings previous = settings;
      RadioModel::adjustVolume(settings, event.delta);
      if (previous.volume != settings.volume || previous.muted != settings.muted) {
        audio.volume(RadioModel::outputVolume(settings)); markSettingsDirty();
      }
    } else if (view == View::Menu) menuIndex = wrap(menuIndex, event.delta, visibleMenuCount());
    else if (view == View::Stations) stationCursor = wrap(stationCursor, event.delta, stationOptions());
    else if (view == View::Edit) {
      int count = editItem == Source ? 2 : editItem == Location ?
                  (editSettings.source == RadioModel::Source::Akashvani ? airLocationCount + 1 : RadioModel::kCountryCount) :
                  editItem == Genre ? RadioModel::kGenreCount : editItem == Codec ? RadioModel::kCodecCount : RadioConfig::BitrateCapCount;
      draft = wrap(draft, event.delta, count);
    }
    return;
  }
  if (event.type != EventType::ShortPress) return;
  if (view == View::Listening) {
    RadioModel::toggleMute(settings); audio.volume(RadioModel::outputVolume(settings)); markSettingsDirty();
  } else if (view == View::Stations) chooseStationOption();
  else if (view == View::Edit) applyDraft();
  else if (view == View::Menu) {
    const MenuItem item = visibleMenuItem(menuIndex);
    if (item == Stations) showStations();
    else if (item >= Source && item <= Bitrate) beginEdit(item);
    else if (item == Wifi) { wifi.startPortal(); enter(View::Wifi); }
    else if (item == Retry) {
      const Station& station = RadioModel::validStation(playback.requested) ? playback.requested : lastGood;
      if (RadioModel::validStation(station)) { startStation(station); enter(View::Listening); }
      else showStations();
    }
  }
}

inline void serviceDirectory() {
  uint32_t startedGeneration = 0;
  if (directoryReady && directory.pollStarted(startedGeneration)) search.started(startedGeneration, millis());
  RadioCatalog::Result* result = nullptr;
  if (directoryReady && directory.poll(result)) {
    if (!search.accepts(result->request.generation)) { delete result; return; }
    if (result->status == RadioCatalog::ResultStatus::Error) {
      search.fail(result->request.generation, millis());
      notify(result->message, 10000);
      delete result;
    } else {
      search.succeed(result->request.generation);
      delete catalog;
      catalog = result;
      const bool changedSource = settings.source != result->request.filter.source;
      settings.source = result->request.filter.source;
      if (settings.source == RadioModel::Source::Akashvani) {
        strlcpy(settings.airLocation, result->request.filter.location, sizeof(settings.airLocation));
        if (!(view == View::Edit && editItem == Location && editSettings.source == RadioModel::Source::Akashvani))
          rememberAirLocations(*result);
      } else {
        // AIR's normalized filter contains no RB genre/codec/country choices.
        // Keep those preferences intact while browsing the other provider.
        settings.country = result->request.filter.country;
        settings.genre = result->request.filter.genre;
        settings.codec = result->request.filter.codec;
        settings.bitrateCap = result->request.filter.bitrateCap;
      }
      if (changedSource) menuIndex = 0;
      markSettingsDirty();
      const Station& current = RadioModel::validStation(playback.confirmed) ? playback.confirmed : playback.requested;
      stationCursor = max(0, RadioModel::findStation(catalog->stations, catalog->count, current));
      if (storageReady && !storage.saveCache(*catalog, settings)) notify("Cannot queue station cache");
      if (autoStartFirst && catalog->count && !RadioModel::validStation(playback.requested)) startStation(catalog->stations[0]);
      else notify(catalog->message);
    }
    dirty = true;
  }
  const uint32_t now = millis();
  if (search.expire(now)) {
    char message[80];
    snprintf(message, sizeof(message), "%s timeout; previous list kept", sourceLabel(search.request.filter.source));
    notify(message, 10000);
  }
  if (search.retryDue(online, now)) requestDirectory(search.request.filter, search.request.offset, search.request.refresh);
}
inline void serviceAudio() {
  RadioAudio::Snapshot update;
  if (audioReady && audio.poll(update)) {
    if (update.fatal) {
      strlcpy(initializationError, update.detail, sizeof(initializationError));
      playback = update;
      audioReady = false;
      dirty = true;
    } else if (update.generation >= audioGeneration && memcmp(&playback, &update, sizeof(update))) {
      playback = update;
      dirty = true;
    }
  }
  if (playback.state == RadioAudio::State::Playing && RadioModel::validStation(playback.confirmed) &&
      (playback.confirmed.source != lastGood.source || strcmp(playback.confirmed.id, lastGood.id) ||
       strcmp(playback.confirmed.url, lastGood.url))) {
    if (!candidateSince || confirmationCandidate.source != playback.confirmed.source ||
        strcmp(confirmationCandidate.id, playback.confirmed.id) || strcmp(confirmationCandidate.url, playback.confirmed.url)) {
      confirmationCandidate = playback.confirmed;
      candidateSince = millis();
    } else if (millis() - candidateSince >= 4000) {
      lastGood = confirmationCandidate;
      markSettingsDirty();
      candidateSince = 0;
    }
  } else candidateSince = 0;
}
inline void printDiagnostics() {
  Serial.printf("GTS Radio: uptime=%lu s heap=%u PSRAM=%u Wi-Fi=%s audio=%s\n",
                static_cast<unsigned long>(millis() / 1000), ESP.getFreeHeap(), ESP.getPsramSize(),
                online ? "connected" : "offline", RadioAudio::stateName(playback.state));
  Serial.printf("audio: %s; %s metadata callbacks\n", RadioAudio::pcmApiName(),
                RadioAudio::MetadataCallbacks<Audio>::modern ? "event" : "legacy");
  const Station& selected = RadioModel::validStation(playback.requested) ? playback.requested : lastGood;
  Serial.printf("catalog: %s; %u visible stations%s; playing source: %s\n", sourceLabel(settings.source),
                catalog ? catalog->count : 0, catalog && catalog->cached ? " (cached)" : "",
                RadioModel::validStation(selected) ? sourceLabel(selected.source) : "none");
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.printf("Serial: native USB CDC; monitor 115200; send ? for status\n");
#else
  Serial.printf("Serial: UART0 USB-to-UART connector; 115200; send ? for status\n");
#endif
}
inline void setup() {
  Serial.begin(115200);
  SPI.begin(RadioConfig::TftClock, -1, RadioConfig::TftMosi, RadioConfig::TftCs);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(4, 12);
  tft.print("GTS Radio starting...");
  if (!canvas.getBuffer()) {
    tft.setCursor(4, 32); tft.print("Display memory failed");
    strlcpy(initializationError, "Display allocation failed", sizeof(initializationError));
    return;
  }
  RadioStorage::WifiCredentials credentials;
  storage.load(settings, lastGood, credentials);
  autoStartFirst = !RadioModel::validStation(lastGood);
  catalog = storage.loadCache(RadioCatalog::filterFor(settings));
  storageReady = storage.begin();
  inputReady = input.begin(RadioConfig::EncoderClock, RadioConfig::EncoderData, RadioConfig::EncoderButton);
  audioReady = audio.begin();
  directoryReady = directory.begin();
  if (!storageReady || !inputReady || !audioReady || !directoryReady)
    strlcpy(initializationError, "Worker/queue initialization failed", sizeof(initializationError));
  if (!wifi.begin(credentials, WIFI_SSID, WIFI_PASSWORD)) notify("Wi-Fi startup failed; retrying", 10000);
  portalWasActive = wifi.portalActive();
  if (wifi.portalActive()) enter(View::Wifi);
  if (audioReady) {
    audio.volume(RadioModel::outputVolume(settings));
    if (RadioModel::validStation(lastGood)) startStation(lastGood);
  }
  markSettingsDirty(); // Persist validated defaults or migrated data once, outside startup.
  printDiagnostics();
  draw();
}
inline void loop() {
  // A monitor opened after startup can request diagnostics without rebooting or
  // interrupting playback. Coalesce pasted requests and limit output to one
  // report per second so serial transmission cannot dominate UI handling.
  static bool diagnosticsPending = false;
  static uint32_t diagnosticsAt = 0;
  for (int i = 0; i < 16 && Serial.available(); ++i)
    if (Serial.read() == '?') diagnosticsPending = true;
  if (diagnosticsPending && millis() - diagnosticsAt >= 1000) {
    diagnosticsPending = false;
    diagnosticsAt = millis();
    printDiagnostics();
  }
  if (!canvas.getBuffer()) { delay(20); return; }
  wifi.loop();
  if (strcmp(lastWifiStatus, wifi.status())) {
    strlcpy(lastWifiStatus, wifi.status(), sizeof(lastWifiStatus));
    dirty = true;
  }
  if (portalWasActive != wifi.portalActive()) {
    portalWasActive = wifi.portalActive();
    if (portalWasActive) {
      notify("Wi-Fi setup available in menu", 10000);
      if (!RadioModel::validStation(playback.requested)) enter(View::Wifi);
    }
    else if (view == View::Wifi) enter(View::Listening);
    dirty = true;
  }
  RadioStorage::WifiCredentials credentials;
  if (wifi.pollCredentials(credentials) && storageReady && !storage.saveWifi(credentials)) notify("Cannot queue Wi-Fi settings");
  const bool connected = wifi.online();
  if (connected != online) {
    online = connected;
    audio.network(online);
    if (online) {
      notify("Wi-Fi connected");
      if (!search.pending) {
        if (search.failed) requestDirectory(search.request.filter, search.request.offset, search.request.refresh);
        else requestDirectory(RadioCatalog::filterFor(settings), 0, true);
      }
    } else notify("Wi-Fi offline; reconnecting", 10000);
    dirty = true;
  }
  RadioInput::Event event;
  for (int i = 0; inputReady && i < 8 && input.poll(event); ++i) handleInput(event);
  if (inputReady && input.overflowCount() != lastOverflow) {
    lastOverflow = input.overflowCount();
    notify("Input overflow; slow rotation");
  }
  serviceDirectory();
  serviceAudio();
  RadioStorage::Status storeStatus;
  if (storageReady && storage.poll(storeStatus)) {
    notify(storeStatus.message, 10000);
    if (storeStatus.kind == RadioStorage::Kind::Settings) { settingsDirty = true; dirtySince = millis() + 3500; }
  }
  const uint32_t now = millis();
  if (storageReady && settingsDirty && static_cast<int32_t>(now - dirtySince) >= static_cast<int32_t>(RadioConfig::SaveDelayMs)) {
    if (storage.save(settings, lastGood)) settingsDirty = false;
    else { dirtySince = now; notify("Settings queue unavailable"); }
  }
  const uint32_t timeout = view == View::Edit ? RadioConfig::EditTimeoutMs : RadioConfig::MenuTimeoutMs;
  const uint32_t activity = inputReady ? input.lastActivity() : lastActionAt;
  // Raw edges refresh activity during debounce; held gestures cannot change mode on timeout.
  if (view != View::Listening && view != View::Wifi && (!inputReady || !input.buttonDown()) &&
      now - lastActionAt > timeout && static_cast<int32_t>(now - activity) > static_cast<int32_t>(timeout) &&
      now - viewEnteredAt > timeout) enter(View::Listening);
  const uint32_t interval = inputReady && input.buttonDown() ? 50 : 250;
  if ((dirty && now - lastDrawAt >= 40) || (animate && now - lastDrawAt >= interval)) draw();
  delay(2);
}
}
