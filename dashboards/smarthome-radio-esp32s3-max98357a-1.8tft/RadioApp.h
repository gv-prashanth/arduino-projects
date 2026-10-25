#pragma once

#ifdef RADIO_APP_HOST_TEST
#include "tests/app_stubs/RadioRuntime.h"
#else
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "AudioService.h"
#include "SettingsService.h"
#include "WifiPortal.h"
#include "RotaryInput.h"
#endif
#include "ListeningScreen.h"
#include "NavigationState.h"

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
using RadioNavigation::View;
using RadioNavigation::FilterMode;
using RadioNavigation::Field;
using RadioNavigation::Action;
constexpr const char* FieldLabels[] = {"Source", "Location", "Genre", "Codec", "Bitrate"};
enum class Purpose : uint8_t { Background, Page, Filter, Entry, EntryRefresh };

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
RadioNavigation::State ui;
View& view = ui.view;
Field& editItem = ui.field;
int& draft = ui.draft;
int& stationCursor = ui.station;
RadioNavigation::Origin requestOrigin;
Purpose requestPurpose = Purpose::Background;
char directoryError[80] = {};
// The official location menu is a snapshot, so turning the encoder never starts
// network requests or depends on a worker-owned catalog's lifetime.
uint8_t airLocationCount = 0;
char airLocations[64][64] = {};
bool audioReady = false, directoryReady = false, storageReady = false, inputReady = false;
bool dirty = true, animate = false, settingsDirty = false;
bool online = false, autoStartFirst = true;
bool backgroundRefreshDue = false;
bool portalWasActive = false;
bool eventButtonDown = false;
View gestureView = View::Listening;
RadioNavigation::State gestureState;
uint32_t gestureSearch = 0;
bool gesturePending = false, gestureFailed = false;
uint32_t audioGeneration = 0;
uint32_t dirtySince = 0, lastDrawAt = 0, viewEnteredAt = 0, lastActionAt = 0;
uint32_t candidateSince = 0, lastOverflow = 0;
RadioUi::NoticeCenter notices, restorationNotice;
RadioDisplay::ListeningMotion listeningMotion;
uint32_t restoredGeneration = 0;
char initializationError[80] = {};
char lastWifiStatus[80] = {};

inline bool due(uint32_t now, uint32_t deadline) { return static_cast<int32_t>(now - deadline) >= 0; }
inline String displayText(const char* raw);
inline void notify(const char* message, uint32_t duration = 5000,
                   RadioUi::NoticePriority priority = RadioUi::NoticePriority::Warning) {
  if (priority == RadioUi::NoticePriority::Warning)
    duration = RadioUi::warningReadDuration(duration,
        RadioDisplay::textWidth(*RadioFonts::Body, displayText(message).c_str()));
  notices.post(message, duration, millis(), priority);
  Serial.printf("radio: %s\n", message);
  dirty = true;
}
inline void markSettingsDirty() { settingsDirty = true; dirtySince = millis(); }
inline void enter(View next) {
  if (next == View::Listening) listeningMotion = RadioDisplay::ListeningMotion{};
  view = next; viewEnteredAt = lastActionAt = millis(); dirty = true;
}
inline const char* sourceLabel(RadioModel::Source source) {
  return source == RadioModel::Source::Akashvani ? "Akashvani" : "Radio Browser";
}
inline void rememberAirLocations(const RadioCatalog::Result& result) {
  if (result.request.filter.source != RadioModel::Source::Akashvani || result.cached) return;
  airLocationCount = min<unsigned>(result.locationCount, sizeof(airLocations) / sizeof(airLocations[0]));
  memset(airLocations, 0, sizeof(airLocations));
  for (int i = 0; i < airLocationCount; ++i)
    strlcpy(airLocations[i], result.locations[i], sizeof(airLocations[i]));
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
inline void label(int x, int y, int width, const char* raw, uint16_t color = RadioDisplay::White,
                  const GFXfont* font = RadioDisplay::RowFont) {
  const String value = displayText(raw);
  RadioDisplay::drawText(canvas, *font, x, y, width, value.c_str(), color, false, 0);
}
inline void header(const char* context) { RadioDisplay::brandHeader(canvas, settings, context); }
inline void footer(const char* hint) {
  canvas.drawFastHLine(4, 110, 152, RadioDisplay::RuleColor);
  label(4, 114, 152, hint, RadioDisplay::Cyan, RadioDisplay::ChromeFont);
}
inline const char* compactLocation(const char* location) {
  return RadioDisplay::compactLocationLabel(location);
}
inline String filterLabel(const Filter& filter) {
  if (filter.source == RadioModel::Source::Akashvani)
    return String("Akashvani / ") + compactLocation(filter.location);
  String value = String("RB / ") + (filter.country ? RadioConfig::Countries[filter.country].code : "All");
  if(filter.genre) value += String(" / ") + RadioConfig::Genres[filter.genre].name;
  return value;
}
inline int stationOptions() { return RadioNavigation::choiceCount(catalog, search.failed); }
inline Action stationAction() { return RadioNavigation::actionAt(stationCursor, catalog, search.failed); }
inline const Station& displayedStation() {
  return RadioModel::validStation(playback.requested) ? playback.requested : lastGood;
}
inline bool enrichLocation(Station& station) {
  if(!catalog || !RadioModel::validStation(station)) return false;
  const int index = RadioModel::findStation(catalog->stations, catalog->count, station);
  if(index < 0 || !catalog->stations[index].location[0] ||
      !strcmp(station.location, catalog->stations[index].location)) return false;
  strlcpy(station.location, catalog->stations[index].location, sizeof(station.location));
  return true;
}
inline RadioDisplay::Icon audioIcon() {
  using RadioAudio::State;
  if(!online || playback.state == State::Offline) return RadioDisplay::Icon::Offline;
  if(playback.state == State::Connecting || playback.state == State::Retrying || playback.state == State::Buffering)
    return RadioDisplay::Icon::Spinner;
  if(playback.state == State::Failed) return RadioDisplay::Icon::Warning;
  if(settings.muted || settings.volume == 0) return RadioDisplay::Icon::Muted;
  return RadioDisplay::Icon::Play;
}
inline void drawListening() {
  const Station& displayed = displayedStation();
  const uint32_t now = millis();
  const RadioUi::Message message = RadioUi::messageFor(playback, online, wifi.status(), settings.muted,
                                                      notices, restorationNotice.active(), settings.volume == 0);
  const String name = displayText(displayed.name[0] ? displayed.name : "Choose a station");
  const String messageText = displayText(message.text);
  RadioDisplay::ListeningFrame frame{settings, displayed, playback, name.c_str(), messageText.c_str(), message.tone, now};
  frame.online = online;
  frame.buttonDown = inputReady && input.buttonDown();
  frame.heldMs = frame.buttonDown ? now - input.pressedAt() : 0;
  animate |= RadioDisplay::drawListeningScreen(canvas, frame, listeningMotion);
}
inline void drawSettings() {
  using RadioDisplay::Icon;
  header("Settings");
  ui.setting = constrain(ui.setting, 0, 2);
  const char* labels[] = {"Wi-Fi setup", "Retry audio", settings.muted ? "Unmute audio" : "Mute audio"};
  const Icon icons[] = {Icon::Wifi, Icon::Retry, settings.muted ? Icon::Volume : Icon::Muted};
  for(int index = 0; index < 3; ++index) {
    const int y = 25 + index * 19;
    if(index == ui.setting) RadioDisplay::focusRow(canvas, y - 2, 18);
    RadioDisplay::drawIcon(canvas, icons[index], 7, y, 13, RadioDisplay::White);
    label(26, y, 127, labels[index]);
  }
  RadioDisplay::footer(canvas, Icon::Check, "Select", Icon::Back, "Back");
}
inline const char* draftLabel(String& scratch) {
  if(editItem == Field::Source) return sourceLabel(static_cast<RadioModel::Source>(draft));
  if(editItem == Field::Location) {
    if(editSettings.source == RadioModel::Source::Akashvani)
      return draft > 0 && draft <= airLocationCount ? airLocations[draft - 1] : "All locations";
    return RadioConfig::Countries[draft].name;
  }
  if(editItem == Field::Genre) return RadioConfig::Genres[draft].name;
  if(editItem == Field::Codec) return draft ? RadioConfig::Codecs[draft] : "All";
  scratch = draft ? String(RadioConfig::BitrateCaps[draft]) + " kbps" : "No cap";
  return scratch.c_str();
}
inline String fieldValue(Field field) {
  if(field == Field::Source) return sourceLabel(settings.source);
  if(field == Field::Location)
    return settings.source == RadioModel::Source::Akashvani ?
        (settings.airLocation[0] ? settings.airLocation : "All locations") : RadioConfig::Countries[settings.country].name;
  if(field == Field::Genre) return RadioConfig::Genres[settings.genre].name;
  if(field == Field::Codec) return settings.codec ? RadioConfig::Codecs[settings.codec] : "All";
  return settings.bitrateCap ? String(settings.bitrateCap) + " kbps" : String("No cap");
}
inline void drawStations() {
  using RadioDisplay::Icon;
  const bool busy = search.pending, problem = search.failed;
  const bool editing = ui.mode != FilterMode::Browse && !busy && !problem;
  const Filter filter = busy || problem ? search.request.filter :
      catalog ? catalog->request.filter : RadioCatalog::filterFor(settings);
  const uint16_t offset = busy || problem ? search.request.offset : catalog ? catalog->request.offset : 0;
  char context[24];
  if(editing) strlcpy(context, ui.mode == FilterMode::Value ? FieldLabels[static_cast<int>(editItem)] : "Filters", sizeof(context));
  else if(catalog && catalog->cached && !busy && !problem) strlcpy(context, "Cached", sizeof(context));
  else if(catalog && catalog->totalKnown && catalog->totalCount && !busy && !problem)
    snprintf(context, sizeof(context), "p%u/%u", offset / RadioModel::kPageSize + 1,
             (catalog->totalCount + RadioModel::kPageSize - 1) / RadioModel::kPageSize);
  else snprintf(context, sizeof(context), "p%u", offset / RadioModel::kPageSize + 1);
  header(context);
  stationCursor = constrain(stationCursor, 0, stationOptions() - 1);
  const Action action = stationAction();
  if(editing || (!busy && action == Action::Filters)) RadioDisplay::focusRow(canvas, 19, 14);
  if(editing && ui.mode == FilterMode::Value) {
    String scratch;
    RadioDisplay::drawIcon(canvas, Icon::Prev, 6, 22, 9, RadioDisplay::Cyan);
    label(20, 21, 120, draftLabel(scratch), RadioDisplay::Cyan, RadioDisplay::ChromeFont);
    RadioDisplay::drawIcon(canvas, Icon::Next, 145, 22, 9, RadioDisplay::Cyan);
  } else {
    static const Icon fieldIcons[] = {Icon::Source, Icon::Location, Icon::Genre, Icon::Codec, Icon::Bitrate};
    const Icon symbol = editing ? fieldIcons[ui.fieldCursor] : Icon::Filters;
    RadioDisplay::drawIcon(canvas, symbol, 6, 22, 10, RadioDisplay::Green);
    const String summary = editing ? String(FieldLabels[ui.fieldCursor]) + ": " + fieldValue(static_cast<Field>(ui.fieldCursor)) : filterLabel(filter);
    label(20, 21, 124, summary.c_str(), RadioDisplay::Green, RadioDisplay::ChromeFont);
    RadioDisplay::drawIcon(canvas, editing ? Icon::ChevronUpDown : Icon::ChevronDown, 145, 22, 10, RadioDisplay::Green);
  }
  if(busy) {
    RadioDisplay::drawIcon(canvas, Icon::Spinner, 68, 43, 24, RadioDisplay::Cyan, millis());
    animate = true;
    char loading[32]; snprintf(loading, sizeof(loading), "Loading page %u", offset / RadioModel::kPageSize + 1);
    label(18, 76, 140, loading);
    footer("Hold Back");
    return;
  }
  if(problem) {
    RadioDisplay::drawIcon(canvas, online ? Icon::Warning : Icon::Offline, 69, 39, 22, RadioDisplay::Yellow);
    label(5, 65, 150, online ? "Could not load page" : "Wi-Fi unavailable", RadioDisplay::Yellow);
    label(5, 83, 150, catalog ? "Previous page kept" : "Change filters or retry", RadioDisplay::MutedInk, RadioDisplay::ChromeFont);
    if(action == Action::Retry) canvas.fillRect(3, 112, 78, 14, RadioDisplay::FocusBackground);
    RadioDisplay::footer(canvas, action == Action::Filters ? Icon::Filters : Icon::Retry,
                         action == Action::Filters ? "Filters" : "Retry", Icon::Back, "Back");
    return;
  }
  if(!catalog || !catalog->count) {
    RadioDisplay::drawIcon(canvas, Icon::Stations, 69, 39, 22, RadioDisplay::MutedInk);
    label(4, 65, 152, catalog && catalog->hasNext ? "No playable rows" : offset ? "End of results" : "No matching stations");
    if(catalog && catalog->hasNext) label(30, 83, 125, "Try next page", RadioDisplay::Cyan, RadioDisplay::ChromeFont);
  } else {
    if(!editing && action == Action::Station) ui.lastStation = stationCursor - 1;
    const int selected = !editing && action == Action::Station ? stationCursor - 1 : ui.lastStation;
    const int shown = min<int>(5, catalog->count);
    const int first = constrain(selected - 1, 0, max(0, catalog->count - shown));
    for(int row = 0; row < shown; ++row) {
      const int index = first + row, y = 34 + row * 15;
      if(!editing && action == Action::Station && stationCursor - 1 == index)
        RadioDisplay::focusRow(canvas, y - 1);
      if(RadioModel::findStation(&catalog->stations[index], 1, displayedStation()) == 0) {
        const Icon symbol = audioIcon();
        const uint16_t color = symbol == Icon::Play ? RadioDisplay::Green :
            symbol == Icon::Warning ? RadioDisplay::Red : RadioDisplay::Yellow;
        RadioDisplay::drawIcon(canvas, symbol, 6, y + 1, 10, color, millis());
        if(symbol == Icon::Spinner) animate = true;
      }
      label(20, y, 135, catalog->stations[index].name, editing ? RadioDisplay::MutedInk : RadioDisplay::White);
    }
    if(catalog->count > shown) {
      const int track = 73, thumb = track * shown / catalog->count;
      canvas.fillRect(158, 34, 2, track, RadioDisplay::RuleColor);
      canvas.fillRect(158, 34 + (track - thumb) * first / (catalog->count - shown), 2, thumb, RadioDisplay::MutedInk);
    }
  }
  canvas.drawFastHLine(4, 110, 152, RadioDisplay::RuleColor);
  RadioDisplay::navButton(canvas, false, Icon::Prev, "Prev", !editing && action == Action::Previous,
                           editing || !catalog || !catalog->request.offset);
  RadioDisplay::navButton(canvas, true, Icon::Next, "Next", !editing && action == Action::Next,
                           editing || !catalog || !catalog->hasNext);
  if(editing && ui.mode == FilterMode::Value) {
    canvas.fillRect(52, 112, 56, 14, RadioDisplay::FocusBackground);
    RadioDisplay::drawIcon(canvas, Icon::Check, 58, 114, 10, RadioDisplay::Green);
    label(73, 114, 34, "Set", RadioDisplay::Green, RadioDisplay::ChromeFont);
  } else label(52, 114, 56, !editing && action == Action::Filters ? "Edit filters" : "Hold Back",
               !editing && action == Action::Filters ? RadioDisplay::Green : RadioDisplay::MutedInk,
               RadioDisplay::ChromeFont);
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
    case View::Settings: drawSettings(); break;
    case View::Stations: drawStations(); break;
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
  restorationNotice = RadioUi::NoticeCenter{};
  autoStartFirst = false;
  dirty = true;
}
inline bool requestDirectory(const Filter& filter, uint16_t offset, bool refresh = false,
                             Purpose purpose = Purpose::Background) {
  if(purpose == Purpose::Background && (requestOrigin.valid ||
      (view == View::Stations && ui.mode != FilterMode::Browse))) return false;
  backgroundRefreshDue = false;
  if(purpose != Purpose::Background) {
    requestOrigin.capture(ui, editSettings);
    autoStartFirst = false;
  }
  requestPurpose = purpose;
  const RadioCatalog::Request request = search.begin(filter, offset, millis(), refresh);
  directoryError[0] = 0;
  if(purpose != Purpose::Background) { ui.mode = FilterMode::Browse; enter(View::Stations); }
  if(!directoryReady || !directory.request(request)) {
    search.fail(request.generation, millis());
    strlcpy(directoryError, "Station service unavailable", sizeof(directoryError));
    stationCursor = 1;
    notify(directoryError);
    return false;
  }
  dirty = true;
  return true;
}
inline void cancelDirectoryToOrigin() {
  if(directoryReady) directory.cancel(search.request.generation);
  search.cancel();
  directoryError[0] = 0;
  if(requestOrigin.valid) requestOrigin.restore(ui, editSettings);
  else enter(View::Listening);
  requestPurpose = Purpose::Background;
  listeningMotion = RadioDisplay::ListeningMotion{};
  dirty = true;
}
inline void showStations() {
  autoStartFirst = false;
  if(search.pending || search.failed) {
    requestOrigin.capture(ui, editSettings);
    requestPurpose = Purpose::EntryRefresh;
    ui.mode = FilterMode::Browse;
    stationCursor = search.failed ? 1 : 0;
    enter(View::Stations);
    return;
  }
  if(!catalog || (catalog->cached && online)) {
    requestDirectory(catalog ? catalog->request.filter : RadioCatalog::filterFor(settings),
                     0, false, Purpose::EntryRefresh);
    return;
  }
  const RadioNavigation::EntryFocus focus = RadioNavigation::entryFocus(catalog, displayedStation());
  if(focus.nextPage && catalog->request.offset <= UINT16_MAX - RadioModel::kPageSize) {
    requestDirectory(catalog->request.filter, catalog->request.offset + RadioModel::kPageSize, false, Purpose::Entry);
    return;
  }
  ui.mode = FilterMode::Browse;
  stationCursor = focus.cursor;
  ui.lastStation = focus.lastStation;
  enter(View::Stations);
}
inline void beginFields() {
  if(search.pending || search.failed) {
    if(directoryReady) directory.cancel(search.request.generation);
    search.cancel();
    if(requestOrigin.valid) requestOrigin.restore(ui, editSettings);
    requestPurpose = Purpose::Background;
  }
  autoStartFirst = false;
  editSettings = settings;
  ui.mode = FilterMode::Fields;
  ui.fieldCursor = 0;
  stationCursor = 0;
  enter(View::Stations);
}
inline void beginEdit(Field item) {
  editSettings = settings;
  editItem = item;
  if(item == Field::Source) draft = static_cast<int>(editSettings.source);
  else if(item == Field::Location) {
    if(editSettings.source == RadioModel::Source::Akashvani) {
      if(catalog) rememberAirLocations(*catalog);
      draft = 0;
      for(int index = 0; index < airLocationCount; ++index)
        if(!strcmp(editSettings.airLocation, airLocations[index])) draft = index + 1;
      // A remembered official group is still selectable before the full session
      // snapshot is online. Never silently replace it with All on a new boot.
      if(!draft && editSettings.airLocation[0] && airLocationCount < 64) {
        strlcpy(airLocations[airLocationCount], editSettings.airLocation, sizeof(airLocations[0]));
        draft = ++airLocationCount;
      }
    } else draft = editSettings.country;
  } else if(item == Field::Genre) draft = editSettings.genre;
  else if(item == Field::Codec) draft = editSettings.codec;
  else {
    draft = 0;
    for(int index = 0; index < RadioConfig::BitrateCapCount; ++index)
      if(editSettings.bitrateCap == RadioConfig::BitrateCaps[index]) draft = index;
  }
  ui.mode = FilterMode::Value;
  enter(View::Stations);
}
inline void applyDraft() {
  Filter candidate = RadioCatalog::filterFor(editSettings);
  if(editItem == Field::Source) {
    Settings selected = editSettings;
    selected.source = static_cast<RadioModel::Source>(draft);
    candidate = RadioCatalog::filterFor(selected);
  } else if(editItem == Field::Location) {
    if(editSettings.source == RadioModel::Source::Akashvani)
      strlcpy(candidate.location, draft > 0 && draft <= airLocationCount ? airLocations[draft - 1] : "", sizeof(candidate.location));
    else candidate.country = draft;
  } else if(editItem == Field::Genre) candidate.genre = draft;
  else if(editItem == Field::Codec) candidate.codec = draft;
  else candidate.bitrateCap = RadioConfig::BitrateCaps[draft];
  candidate = RadioCatalog::normalizeFilter(candidate);
  if(catalog && candidate == RadioCatalog::filterFor(settings)) {
    ui.mode = FilterMode::Browse; // Same value: keep the accepted page/cursor, no hidden Refresh.
    dirty = true;
    return;
  }
  requestDirectory(candidate, 0, false, Purpose::Filter);
}
inline void chooseStationOption() {
  if(search.pending) return;
  if(ui.mode == FilterMode::Fields) { beginEdit(static_cast<Field>(ui.fieldCursor)); return; }
  if(ui.mode == FilterMode::Value) { applyDraft(); return; }
  const Action action = stationAction();
  if(action == Action::Filters) { beginFields(); return; }
  if(action == Action::Station && catalog) {
    startStation(catalog->stations[stationCursor - 1]); enter(View::Listening); return;
  }
  if(action == Action::Retry) {
    const RadioCatalog::Request attempted = search.request;
    requestDirectory(attempted.filter, attempted.offset, attempted.refresh,
                     requestOrigin.valid ? requestPurpose : Purpose::Page);
    return;
  }
  if(!catalog) return;
  if(action == Action::Previous) requestDirectory(catalog->request.filter,
      catalog->request.offset - RadioModel::kPageSize, false, Purpose::Page);
  else if(action == Action::Next) {
    if(catalog->request.offset > UINT16_MAX - RadioModel::kPageSize)
      notify("End of supported page range", 5000, RadioUi::NoticePriority::Info);
    else requestDirectory(catalog->request.filter,
      catalog->request.offset + RadioModel::kPageSize, false, Purpose::Page);
  }
}
inline int wrap(int current, int delta, int count) {
  return count > 0 ? ((current + delta) % count + count) % count : 0;
}
inline void handleInput(const RadioInput::Event& event) {
  lastActionAt = millis();
  dirty = true;
  using RadioInput::EventType;
  if(event.type == EventType::ButtonDown) {
    eventButtonDown = true; gestureView = view; gestureState = ui;
    gestureSearch = search.generation; gesturePending = search.pending; gestureFailed = search.failed;
    return;
  }
  if(event.type == EventType::ButtonUp) { eventButtonDown = false; return; }
  const bool press = event.type == EventType::ShortPress || event.type == EventType::LongPress;
  if(press && (!RadioNavigation::sameControlContext(gestureState, ui) || gestureView != view ||
      (view == View::Stations && (gestureSearch != search.generation ||
        gesturePending != search.pending || gestureFailed != search.failed)))) return;
  if(event.type == EventType::LongPress) {
    if(view == View::Listening) { autoStartFirst = false; ui.setting = 0; enter(View::Settings); }
    else if(view == View::Settings) enter(View::Listening);
    else if(view == View::Wifi) enter(View::Settings);
    else if(search.pending || search.failed) cancelDirectoryToOrigin();
    else if(ui.mode == FilterMode::Value) { ui.mode = FilterMode::Fields; }
    else if(ui.mode == FilterMode::Fields) { ui.mode = FilterMode::Browse; stationCursor = 0; }
    else enter(View::Listening);
    return;
  }
  if(event.type == EventType::Rotate) {
    if(eventButtonDown) return;
    if(view == View::Listening) {
      Settings previous = settings;
      RadioModel::adjustVolume(settings, event.delta);
      if(previous.volume != settings.volume || previous.muted != settings.muted) {
        audio.volume(RadioModel::outputVolume(settings)); markSettingsDirty();
      }
    } else if(view == View::Settings) ui.setting = wrap(ui.setting, event.delta, 3);
    else if(view == View::Stations && !search.pending) {
      if(ui.mode == FilterMode::Fields)
        ui.fieldCursor = wrap(ui.fieldCursor, event.delta, RadioNavigation::fieldCount(settings.source));
      else if(ui.mode == FilterMode::Value) {
        const int count = editItem == Field::Source ? 2 : editItem == Field::Location ?
            (editSettings.source == RadioModel::Source::Akashvani ? airLocationCount + 1 : RadioModel::kCountryCount) :
            editItem == Field::Genre ? RadioModel::kGenreCount : editItem == Field::Codec ? RadioModel::kCodecCount : RadioConfig::BitrateCapCount;
        draft = wrap(draft, event.delta, count);
      } else stationCursor = wrap(stationCursor, event.delta, stationOptions());
    }
    return;
  }
  if(event.type != EventType::ShortPress) return;
  if(view == View::Listening) showStations();
  else if(view == View::Stations) chooseStationOption();
  else if(view == View::Settings) {
    if(ui.setting == 0) { wifi.startPortal(); enter(View::Wifi); }
    else if(ui.setting == 1) {
      const Station& station = displayedStation();
      if(RadioModel::validStation(station)) startStation(station);
      else notify("Choose a station first", 5000, RadioUi::NoticePriority::Info);
      enter(View::Listening);
    } else {
      RadioModel::toggleMute(settings);
      audio.volume(RadioModel::outputVolume(settings));
      markSettingsDirty();
    }
  }
}

inline void serviceDirectory() {
  uint32_t startedGeneration = 0;
  if(directoryReady && directory.pollStarted(startedGeneration)) search.started(startedGeneration, millis());
  RadioCatalog::Result* result = nullptr;
  if(directoryReady && directory.poll(result)) {
    if(!search.accepts(result->request.generation)) { delete result; return; }
    if(result->status == RadioCatalog::ResultStatus::Error) {
      search.fail(result->request.generation, millis());
      strlcpy(directoryError, result->message, sizeof(directoryError));
      stationCursor = 1;
      notify(result->message, 10000);
      delete result;
    } else {
      const Purpose completedPurpose = requestPurpose;
      search.succeed(result->request.generation);
      delete catalog;
      catalog = result;
      enrichLocation(playback.requested);
      enrichLocation(playback.confirmed);
      if(enrichLocation(lastGood)) markSettingsDirty();
      settings.source = result->request.filter.source;
      if(settings.source == RadioModel::Source::Akashvani) {
        strlcpy(settings.airLocation, result->request.filter.location, sizeof(settings.airLocation));
        rememberAirLocations(*result);
      } else {
        settings.country = result->request.filter.country;
        settings.genre = result->request.filter.genre;
        settings.codec = result->request.filter.codec;
        settings.bitrateCap = result->request.filter.bitrateCap;
      }
      ui.fieldCursor = 0;
      markSettingsDirty();
      ui.mode = FilterMode::Browse;
      ui.lastStation = 0;
      stationCursor = catalog->count ? 1 : max(0, RadioNavigation::cursorFor(Action::Next, catalog));
      if(storageReady && !storage.saveCache(*catalog, settings)) notify("Cannot queue station cache");
      if(completedPurpose == Purpose::EntryRefresh) {
        const RadioNavigation::EntryFocus focus = RadioNavigation::entryFocus(catalog, displayedStation());
        stationCursor = focus.cursor; ui.lastStation = focus.lastStation;
        if(focus.nextPage && catalog->request.offset <= UINT16_MAX - RadioModel::kPageSize) {
          requestDirectory(catalog->request.filter, catalog->request.offset + RadioModel::kPageSize, false, Purpose::Entry);
          return; // Keep the original player rollback across the entry's full-page refresh.
        }
      }
      requestOrigin.valid = false;
      requestPurpose = Purpose::Background;
      directoryError[0] = 0;
      if(autoStartFirst && catalog->count && !RadioModel::validStation(playback.requested))
        startStation(catalog->stations[0]);
      else notify(catalog->message, 5000, RadioUi::NoticePriority::Info);
    }
    dirty = true;
  }
  const uint32_t now = millis();
  if(search.expire(now)) {
    snprintf(directoryError, sizeof(directoryError), "%s timeout", sourceLabel(search.request.filter.source));
    stationCursor = 1;
    notify(directoryError, 10000);
  }
  // Visible user operations wait for Retry or Back. Background startup/network
  // refreshes can back off automatically, but cancellation clears their token.
  if(backgroundRefreshDue && online && !search.pending && !requestOrigin.valid &&
      !(view == View::Stations && ui.mode != FilterMode::Browse))
    requestDirectory(RadioCatalog::filterFor(settings), 0, true);
  else if(!requestOrigin.valid && search.retryDue(online, now))
    requestDirectory(search.request.filter, search.request.offset, search.request.refresh);
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
      if (update.state == RadioAudio::State::Playing && update.generation != restoredGeneration &&
          !strcmp(update.detail, "Previous station restored")) {
        restoredGeneration = update.generation;
        restorationNotice.post("Previous station restored", 5000, millis(), RadioUi::NoticePriority::Warning);
      }
      dirty = true;
    }
  }
  enrichLocation(playback.requested);
  enrichLocation(playback.confirmed);
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
  if (!wifi.begin(credentials, WIFI_SSID, WIFI_PASSWORD))
    notify("Wi-Fi startup failed; retrying", 10000, RadioUi::NoticePriority::Info);
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
      notify("Wi-Fi setup available in Settings", 10000, RadioUi::NoticePriority::Info);
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
      backgroundRefreshDue = true;
      notify("Wi-Fi connected", 5000, RadioUi::NoticePriority::Info);
      if (!search.pending && !requestOrigin.valid) {
        if (search.failed) requestDirectory(search.request.filter, search.request.offset, search.request.refresh);
        else requestDirectory(RadioCatalog::filterFor(settings), 0, true);
      }
    } else notify("Wi-Fi offline; reconnecting", 10000, RadioUi::NoticePriority::Info);
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
  // Browsing/settings/editing stay open until the user goes Back. In particular
  // a slow directory request cannot change the destination of a held gesture.
  // Static short text needs no animation, but notices still expire and redraw.
  // Hidden warnings/restoration feedback keep their remaining visible time.
  const RadioUi::Message message = RadioUi::messageFor(playback, online, wifi.status(), settings.muted,
                                                      notices, restorationNotice.active(), settings.volume == 0);
  if (notices.update(now, view == View::Listening && !initializationError[0] && message.usesNotice)) dirty = true;
  if (restorationNotice.update(now, view == View::Listening && !initializationError[0] && message.usesFallback)) dirty = true;
  const uint32_t interval = inputReady && input.buttonDown() ? 50 : 250;
  if ((dirty && now - lastDrawAt >= 40) || (animate && now - lastDrawAt >= interval)) draw();
  delay(2);
}
}
