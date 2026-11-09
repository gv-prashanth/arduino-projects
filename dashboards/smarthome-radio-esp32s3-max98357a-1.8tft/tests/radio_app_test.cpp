#define RADIO_APP_HOST_TEST
#include "../RadioApp.h"

#include <cassert>
#include <cstdio>
#include <string>

namespace {
using namespace RadioApp;
using RadioModel::Source;
using RadioInput::EventType;

void reset() {
  delete catalog; catalog = nullptr;
  directory.clear(); directory = RadioCatalog::CatalogService{};
  audio = RadioAudio::AudioService{};
  storage = RadioStorage::SettingsService{};
  wifi = RadioWifi::WifiManager{};
  input = RadioInput::RotaryInput{};
  settings = Settings{}; editSettings = Settings{};
  lastGood = Station{}; confirmationCandidate = Station{}; playback = RadioAudio::Snapshot{};
  search = RadioCatalog::Transaction{}; ui = RadioNavigation::State{};
  requestOrigin = RadioNavigation::Origin{}; requestPurpose = Purpose::Background;
  directoryError[0] = initializationError[0] = lastWifiStatus[0] = 0;
  airLocationCount = 0; std::memset(airLocations, 0, sizeof(airLocations));
  backgroundRefreshDue = false;
  audioReady = directoryReady = storageReady = inputReady = true;
  radioDisplay = RadioDisplay::DisplayHardware{};
  displayReady = radioDisplay.begin();
  dirty = true; animate = settingsDirty = false; online = true; autoStartFirst = false;
  portalWasActive = eventButtonDown = false;
  gestureView = View::Listening; gestureState = RadioNavigation::State{};
  gestureSearch = 0; gesturePending = gestureFailed = false;
  audioGeneration = 0; dirtySince = lastDrawAt = viewEnteredAt = lastActionAt = 0;
  candidateSince = lastOverflow = restoredGeneration = 0;
  notices = RadioUi::NoticeCenter{}; restorationNotice = RadioUi::NoticeCenter{};
  listeningMotion = RadioDisplay::ListeningMotion{};
  filterValueMotion = RadioDisplay::RowMotion{}; wifiStatusMotion = RadioDisplay::RowMotion{};
  RadioHost::now = 1000;
  canvas.accepting = true;
  canvas.fillScreen(0); canvas.clipped = 0;
}
Station station(unsigned index, Source source = Source::RadioBrowser) {
  Station value; value.source = source;
  char id[32], name[64], url[128];
  std::snprintf(id, sizeof(id), "station-%u", index);
  std::snprintf(name, sizeof(name), "Station %u", index + 1);
  std::snprintf(url, sizeof(url), "https://example.org/%u/live.m3u8", index);
  RadioModel::copyText(value.id, id); RadioModel::copyText(value.name, name);
  RadioModel::copyText(value.url, url); RadioModel::copyText(value.country, "IN");
  RadioModel::copyText(value.location, source == Source::Akashvani ? "Andhra Pradesh" : "India");
  assert(RadioModel::validStation(value)); return value;
}
RadioCatalog::Result* page(unsigned count = 8, bool next = true, uint16_t offset = 0,
                           Source source = Source::RadioBrowser) {
  auto* result = new RadioCatalog::Result;
  Settings filterSettings = settings; filterSettings.source = source;
  result->request.filter = RadioCatalog::filterFor(filterSettings);
  result->request.offset = offset; result->status = count ? RadioCatalog::ResultStatus::Success : RadioCatalog::ResultStatus::Empty;
  result->count = static_cast<uint8_t>(count); result->hasNext = next;
  for (unsigned index = 0; index < count; ++index) result->stations[index] = station(index + offset, source);
  RadioModel::copyText(result->message, "Stations loaded");
  return result;
}
void installPage(unsigned count = 8, bool next = true, uint16_t offset = 0,
                 Source source = Source::RadioBrowser) {
  delete catalog; catalog = page(count, next, offset, source);
}
void playing(const Station& value) {
  lastGood = value; playback.requested = playback.confirmed = value;
  playback.state = RadioAudio::State::Playing;
  RadioModel::copyText(playback.codec, "AAC"); playback.bitrateKbps = 128;
  RadioModel::copyText(playback.detail, "Decoded audio ready");
}
void event(EventType type, int delta = 0) {
  handleInput({type, static_cast<int16_t>(delta), millis()});
}
void click() { event(EventType::ButtonDown); event(EventType::ButtonUp); event(EventType::ShortPress); }
void hold() { event(EventType::ButtonDown); RadioHost::now += 700; event(EventType::LongPress); event(EventType::ButtonUp); }
void deliver(RadioCatalog::Result* result) { directory.results.push_back(result); serviceDirectory(); }
void succeed(unsigned count = 8, bool next = true) {
  RadioCatalog::Result* result = page(count, next, search.request.offset, search.request.filter.source);
  result->request = search.request; deliver(result);
}
void fail(const char* reason = "Directory unavailable") {
  auto* result = new RadioCatalog::Result; result->request = search.request;
  RadioModel::copyText(result->message, reason); deliver(result);
}
void sameRegion(const std::vector<uint16_t>& expected, int x, int y, int width, int height) {
  for (int py = y; py < y + height; ++py) for (int px = x; px < x + width; ++px)
    assert(canvas.at(px, py) == expected[static_cast<unsigned>(py * RadioDisplay::ScreenWidth + px)]);
}
void render() {
  canvas.clipped = 0; draw(); assert(canvas.clipped == 0);
  assert(radioDisplay.pixels == canvas.pixels);
}

void testPlayerClickAndLastRowEntry() {
  reset(); installPage(); playing(catalog->stations[2]);
  click();
  assert(view == View::Stations && stationCursor == 4 && ui.lastStation == 2);
  assert(audio.startCalls == 0 && directory.requests.empty());
  click();
  assert(view == View::Listening && audio.startCalls == 1);
  assert(std::strcmp(audio.lastStation.id, catalog->stations[3].id) == 0);
  reset(); installPage(20); playing(catalog->stations[19]);
  const Station previous = playback.requested;
  click();
  assert(view == View::Stations && search.pending && search.request.offset == 20);
  assert(audio.startCalls == 0 && std::strcmp(playback.requested.id, previous.id) == 0);
  succeed();
  assert(!search.pending && stationCursor == 1 && audio.startCalls == 0);
  assert(std::strcmp(playback.requested.id, previous.id) == 0);
  reset(); installPage(20); playing(catalog->stations[19]);
  const RadioNavigation::State playerOrigin = ui;
  click(); assert(search.pending); hold();
  assert(ui == playerOrigin && view == View::Listening && !search.pending && audio.startCalls == 0);
}
void testExactCancelOriginAndLateResults() {
  reset(); installPage(); playing(catalog->stations[0]); enter(View::Stations);
  beginFields(); ui.fieldCursor = 1; beginEdit(Field::Location); event(EventType::Rotate, 2);
  const RadioNavigation::State origin = ui; const Settings editor = editSettings;
  const Filter accepted = RadioCatalog::filterFor(settings); auto* previousPage = catalog;
  click();
  assert(search.pending && search.request.filter.country == 2);
  const RadioCatalog::Request cancelled = search.request;
  settings.volume = 15; // Audio/volume are independent of request rollback.
  hold();
  assert(!search.pending && !search.failed && ui == origin);
  assert(RadioCatalog::filterFor(editSettings) == RadioCatalog::filterFor(editor));
  assert(RadioCatalog::filterFor(settings) == accepted && settings.volume == 15 && catalog == previousPage);
  assert(directory.cancellations.back() == cancelled.generation);
  auto* late = page(); late->request = cancelled; deliver(late);
  assert(catalog == previousPage && ui == origin && audio.startCalls == 0);
  click(); // The exact retained candidate can be retried.
  assert(search.pending && search.request.generation != cancelled.generation);
  late = page(); late->request = cancelled; deliver(late);
  assert(search.pending && catalog == previousPage);
  succeed(); assert(!search.pending && settings.country == 2);
}
void testSetStartsSearchAndErrorRetainsAcceptedFilters() {
  reset(); settings.country = 2; settings.genre = 4; installPage();
  playing(catalog->stations[0]); enter(View::Stations); beginFields(); beginEdit(Field::Source);
  event(EventType::Rotate, 1); const RadioNavigation::State picker = ui;
  const Filter accepted = RadioCatalog::filterFor(settings); auto* previousPage = catalog;
  click();
  assert(search.pending && directory.requests.size() == 1);
  assert(search.request.filter.source == Source::Akashvani);
  assert(RadioCatalog::filterFor(settings) == accepted); // Accepted filter/page commit together.
  fail();
  assert(search.failed && !search.pending && catalog == previousPage);
  assert(RadioCatalog::filterFor(settings) == accepted && audio.startCalls == 0);
  render();
  hold(); assert(ui == picker && draft == 1 && RadioCatalog::filterFor(settings) == accepted);
  click(); succeed();
  assert(settings.source == Source::Akashvani && settings.country == 2 && settings.genre == 4);
  assert(catalog->request.filter.source == Source::Akashvani && ui.mode == FilterMode::Browse);
}
void testSameValueAndEmptyPageNext() {
  reset(); installPage(); enter(View::Stations); beginFields(); beginEdit(Field::Source);
  click(); assert(directory.requests.empty() && !search.pending && ui.mode == FilterMode::Browse);
  reset(); installPage(0, true, 20); enter(View::Stations);
  stationCursor = RadioNavigation::cursorFor(Action::Next, catalog);
  assert(stationCursor >= 0 && stationAction() == Action::Next);
  render(); click();
  assert(search.pending && search.request.offset == 40 && audio.startCalls == 0);
  succeed(0, false);
  assert(!search.pending && !catalog->count && !catalog->hasNext);
  assert(RadioNavigation::cursorFor(Action::Next, catalog) == -1);
  render();
}
void testSettingsAndHeldGestures() {
  reset(); installPage(); playing(catalog->stations[0]);
  hold(); assert(view == View::Settings && ui.setting == 0);
  event(EventType::Rotate, 8); assert(ui.setting == 2);
  click(); assert(settings.muted && view == View::Settings && directory.requests.empty());
  event(EventType::Rotate, 1); assert(ui.setting == 0);
  click(); assert(view == View::Wifi && wifi.portalActive());
  hold(); assert(view == View::Settings);
  event(EventType::Rotate, 1); click();
  assert(view == View::Listening && audio.startCalls == 1);
  assert(std::strcmp(audio.lastStation.id, playback.requested.id) == 0);
  reset(); hold(); event(EventType::Rotate, 1); click();
  assert(view == View::Listening && audio.startCalls == 0 && notices.active());
  // A press that began on Loading cannot select a newly arrived station.
  reset(); installPage(20); playing(catalog->stations[19]); click();
  event(EventType::ButtonDown); succeed(); event(EventType::ButtonUp); event(EventType::ShortPress);
  assert(view == View::Stations && audio.startCalls == 0);
  // Rendering updates scroll memory, but does not invalidate the same control.
  reset(); installPage(); enter(View::Stations); stationCursor = 4; ui.lastStation = 0;
  event(EventType::ButtonDown); render(); assert(ui.lastStation == 3);
  event(EventType::ButtonUp); event(EventType::ShortPress);
  assert(audio.startCalls == 1 && view == View::Listening);
  assert(std::strcmp(audio.lastStation.id, catalog->stations[3].id) == 0);
}
void testMetadataAndPixelScreens(const std::string& prefix) {
  reset(); installPage(8, true, 20, Source::Akashvani);
  settings.source = Source::Akashvani;
  RadioModel::copyText(settings.airLocation, "Andhra Pradesh");
  RadioModel::copyText(catalog->request.filter.location, settings.airLocation);
  const char* names[] = {"Vijayawada", "Kurnool", "Kadapa", "Tirupati", "Visakhapatnam", "Raagam", "FM Gold", "AIR News"};
  for (int index = 0; index < 8; ++index) RadioModel::copyText(catalog->stations[index].name, names[index]);
  playing(catalog->stations[1]); enter(View::Stations); stationCursor = 3;
  render();
  for (int row = 0; row < RadioDisplay::VisibleStationRows; ++row)
    assert(canvas.contains(20, RadioDisplay::FirstStationTop + row * RadioDisplay::StationRowHeight,
                           RadioDisplay::ScreenWidth - 25, RadioDisplay::fontHeight(*RadioDisplay::RowFont),
                           RadioDisplay::White));
  if (!prefix.empty()) canvas.writePpm(prefix + "stations.ppm");
  beginFields(); render(); if (!prefix.empty()) canvas.writePpm(prefix + "filters.ppm");
  beginEdit(Field::Location); render(); if (!prefix.empty()) canvas.writePpm(prefix + "location-picker.ppm");
  applyDraft();
  // Same-value Set leaves accepted rows visible without a request.
  assert(!search.pending);
  requestDirectory(catalog->request.filter, 40, false, Purpose::Page);
  render(); if (!prefix.empty()) canvas.writePpm(prefix + "loading.ppm");
  fail(); render(); if (!prefix.empty()) canvas.writePpm(prefix + "directory-error.ppm");
  cancelDirectoryToOrigin();
  enter(View::Settings); ui.setting = 1; render();
  for (int row = 0; row < 3; ++row)
    assert(canvas.contains(26, (RadioDisplay::LargeDisplay ? 56 : 25) + row * (RadioDisplay::LargeDisplay ? 40 : 19),
                           RadioDisplay::ScreenWidth - 33, RadioDisplay::fontHeight(*RadioDisplay::RowFont),
                           RadioDisplay::White));
  if (!prefix.empty()) canvas.writePpm(prefix + "settings.ppm");
  enter(View::Listening); notices = RadioUi::NoticeCenter{}; render();
  if (!prefix.empty()) canvas.writePpm(prefix + "player.ppm");
  const auto actualStationFrame = canvas.pixels;
  settings.source = Source::RadioBrowser; settings.country = 3; settings.genre = 2;
  render(); assert(canvas.pixels == actualStationFrame);
  GFXcanvas16 expected(RadioDisplay::ScreenWidth, RadioDisplay::ScreenHeight);
  RadioDisplay::drawText(expected, *RadioDisplay::ChromeFont, RadioDisplay::ContentLeft,
                         RadioDisplay::FilterTop, RadioDisplay::ContentWidth,
                         RadioDisplay::LargeDisplay ? "Akashvani / Andhra Pradesh" : "Akashvani / AP",
                         RadioDisplay::Green, false, 0);
  sameRegion(expected.pixels, 0, RadioDisplay::FilterTop, RadioDisplay::ScreenWidth,
             RadioDisplay::fontHeight(*RadioDisplay::ChromeFont));
  playback.state = RadioAudio::State::Failed;
  RadioModel::copyText(playback.detail, "Stream unavailable");
  render(); if (!prefix.empty()) canvas.writePpm(prefix + "playback-error.ppm");
}
void testAdditionalScreenStates(const std::string& prefix) {
  reset();
  enter(View::Wifi); wifi.portal = true; render();
  if (!prefix.empty()) canvas.writePpm(prefix + "wifi.ppm");
  wifi.portal = false; wifi.connected = false; render();
  if (!prefix.empty()) canvas.writePpm(prefix + "wifi-restarting.ppm");
  RadioModel::copyText(initializationError, "Worker/queue initialization failed");
  render(); if (!prefix.empty()) canvas.writePpm(prefix + "startup-error.ppm");
  reset(); installPage(0, false); enter(View::Stations); render();
  if (!prefix.empty()) canvas.writePpm(prefix + "empty.ppm");
  catalog->hasNext = true; render();
  if (!prefix.empty()) canvas.writePpm(prefix + "empty-next.ppm");
  reset(); installPage(); enter(View::Stations); beginFields();
  for (Field field : {Field::Source, Field::Location, Field::Genre, Field::Codec, Field::Bitrate}) {
    beginEdit(field); render(); hold();
  }
  reset(); installPage(); playing(catalog->stations[0]);
  RadioModel::copyText(playback.requested.name, "A long station name with words that must fit safely inside the display");
  for (RadioAudio::State state : {RadioAudio::State::Idle, RadioAudio::State::Connecting,
       RadioAudio::State::Playing, RadioAudio::State::Buffering, RadioAudio::State::Retrying,
       RadioAudio::State::Failed, RadioAudio::State::Offline}) {
    playback.state = state; render();
    if (!prefix.empty()) canvas.writePpm(prefix + "player-state-" + std::to_string(static_cast<int>(state)) + ".ppm");
  }
  settings.muted = true; playback.state = RadioAudio::State::Playing; render();
  if (!prefix.empty()) canvas.writePpm(prefix + "muted.ppm");
  settings.muted = false; settings.volume = 0; render();
  if (!prefix.empty()) canvas.writePpm(prefix + "volume-zero.ppm");
}
void testLargeWarningSurvivesUntilItsEndCanBeRead() {
  if (!RadioDisplay::LargeDisplay) return;
  reset(); installPage(); playing(catalog->stations[0]);
  const std::string warning(175, 'W');
  notify(warning.c_str(), 1000, RadioUi::NoticePriority::Warning);
  notices.update(millis(), true);
  render();
  // The real 18px message font must reach its final characters before the
  // warning disappears, with a trailing reading pause after that travel.
  const int overflow = RadioDisplay::textWidth(*RadioDisplay::DetailFont, notices.message()) -
                       RadioDisplay::ContentWidth;
  assert(overflow > 0);
  RadioHost::now += 1500 + static_cast<uint32_t>(overflow) * 40;
  assert(!notices.update(millis(), true) && notices.warning());
  render();
  assert(canvas.contains(RadioDisplay::ContentLeft + RadioDisplay::ContentWidth - 25,
                         RadioDisplay::PlayerMessageTop, 25,
                         RadioDisplay::fontHeight(*RadioDisplay::DetailFont), RadioDisplay::Yellow));
  RadioHost::now += 1499;
  assert(!notices.update(millis(), true) && notices.warning());
  ++RadioHost::now;
  assert(notices.update(millis(), true) && !notices.active());
}
void testSetupAndLoopBoundaries() {
  reset(); online = false; setup();
  assert(audioReady && directoryReady && storageReady && inputReady && initializationError[0] == 0);
  loop();
  // A network transition queues one startup request; no station starts before a valid result.
  assert(search.pending && directory.requests.size() == 1 && audio.startCalls == 0);
  autoStartFirst = true; succeed(); assert(audio.startCalls == 1);
}
void testDisplayStartupFailureStopsRendering() {
  reset(); radioDisplay.accepting = false; setup();
  assert(!displayReady && initializationError[0]);
  const auto presentations = radioDisplay.presentCalls;
  loop();
  assert(radioDisplay.presentCalls == presentations && !search.pending);
  reset(); canvas.accepting = false; setup();
  assert(initializationError[0] && !radioDisplay.lastDetail.empty());
  const auto allocationFailurePresentations = radioDisplay.presentCalls;
  loop();
  assert(radioDisplay.presentCalls == allocationFailurePresentations && !search.pending);
}
void testReconnectDefersWhileEditing() {
  reset(); installPage(); enter(View::Stations); beginFields();
  const Filter accepted = RadioCatalog::filterFor(settings);
  assert(!requestDirectory(accepted, 0, true));
  assert(!search.pending && ui.mode == FilterMode::Fields && directory.requests.empty());
  beginEdit(Field::Location); event(EventType::Rotate, 2);
  const RadioNavigation::State picker = ui;
  assert(!requestDirectory(accepted, 0, true));
  assert(!search.pending && ui == picker && directory.requests.empty());
  wifi.connected = false; loop(); wifi.connected = true; loop();
  assert(online && backgroundRefreshDue && !search.pending && ui == picker);
  assert(directory.requests.empty());
  hold(); assert(ui.mode == FilterMode::Fields);
  serviceDirectory(); assert(!search.pending && directory.requests.empty());
  hold(); assert(ui.mode == FilterMode::Browse);
  serviceDirectory();
  assert(search.pending && search.request.refresh && directory.requests.size() == 1);
  assert(search.request.filter == accepted && !backgroundRefreshDue);
}
void testAudioDiagnosticsDoNotRepaintUnchangedScreen() {
  reset(); installPage(); playing(catalog->stations[0]);
  dirty = false;
  auto update = playback;
  update.diagnostics.inputFilledBytes = 42000;
  update.diagnostics.inputCapacityBytes = 655350;
  update.diagnostics.maxPcmGapMs = 140;
  audio.updates.push_back(update);
  serviceAudio();
  assert(playback.diagnostics.inputFilledBytes == 42000);
  assert(playback.diagnostics.maxPcmGapMs == 140 && !dirty);
  RadioModel::copyText(update.title, "New programme title");
  audio.updates.push_back(update);
  serviceAudio();
  assert(dirty && !std::strcmp(playback.title, "New programme title"));
}
} // namespace

int main(int argc, char** argv) {
  const std::string prefix = argc > 1 ? argv[1] : "";
  testPlayerClickAndLastRowEntry();
  testExactCancelOriginAndLateResults();
  testSetStartsSearchAndErrorRetainsAcceptedFilters();
  testSameValueAndEmptyPageNext();
  testSettingsAndHeldGestures();
  testMetadataAndPixelScreens(prefix);
  testAdditionalScreenStates(prefix);
  testLargeWarningSurvivesUntilItsEndCanBeRead();
  testSetupAndLoopBoundaries();
  testDisplayStartupFailureStopsRendering();
  testReconnectDefersWhileEditing();
  testAudioDiagnosticsDoNotRepaintUnchangedScreen();
  reset();
  std::puts("RadioApp integration and pixel tests passed");
}
