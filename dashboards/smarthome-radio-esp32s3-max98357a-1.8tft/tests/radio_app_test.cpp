#define RADIO_APP_HOST_TEST
#include "../RadioApp.h"

#include <cassert>
#include <cstdio>
#include <string>
#include <utility>

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
  touchReady = false; touchContact = TouchContact{}; touchInputRevision = touchViewRevision = 0;
  touchHardwareReady = touchPolled = touchSavePending = touchSaveAttempted = false;
  touchPollAt = touchSessionRevision = touchSaveAt = touchReadErrors = 0;
  touchPanel = RadioTouch::TouchHardware{};
  touchStore = RadioTouch::CalibrationStore{};
  touchSession = RadioTouch::Session{};
  Serial.incoming.clear();
  dirty = true; animate = settingsDirty = false; online = true; autoStartFirst = false;
  portalWasActive = eventButtonDown = gestureArmed = false;
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
void touchDown(int x, int y) { handleTouchEvent({RadioTouch::Type::Down, int16_t(x), int16_t(y)}); }
void touchTap(int x, int y) {
  touchDown(x, y);
  handleTouchEvent({RadioTouch::Type::Tap, int16_t(x), int16_t(y)});
}
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
void testTouchGestureDebounceAndRelease() {
  RadioTouch::Gesture gesture; RadioTouch::Event result;
  assert(!gesture.sample(true, 100, 80, 0, result));
  assert(!gesture.sample(true, 100, 80, 19, result));
  assert(gesture.sample(true, 100, 80, 20, result) && result.type == RadioTouch::Type::Down);
  assert(gesture.sample(true, 100, 80, 5000, result) && result.type == RadioTouch::Type::Move); // No repeated Down/action.
  assert(!gesture.sample(false, 0, 0, 5010, result));
  assert(!gesture.sample(false, 0, 0, 5029, result));
  assert(gesture.sample(false, 0, 0, 5030, result) && result.type == RadioTouch::Type::Tap);
  assert(result.x == 100 && result.y == 80);
  assert(!gesture.sample(false, 0, 0, 5100, result));

  assert(!gesture.sample(true, 100, 100, 6000, result));
  assert(gesture.sample(true, 100, 100, 6020, result));
  assert(gesture.sample(true, 101, 60, 6040, result) && result.type == RadioTouch::Type::Move);
  assert(!gesture.sample(false, 0, 0, 6060, result));
  assert(gesture.sample(false, 0, 0, 6080, result) && result.type == RadioTouch::Type::SwipeUp);
  assert(result.x == 100 && result.y == 100); // Swipes retain the contact origin.

  assert(!gesture.sample(true, 100, 100, 7000, result));
  assert(gesture.sample(true, 100, 100, 7020, result));
  assert(gesture.sample(true, 120, 100, 7040, result) && result.type == RadioTouch::Type::Move);
  assert(gesture.sample(true, 100, 100, 7060, result) && result.type == RadioTouch::Type::Move);
  assert(!gesture.sample(false, 0, 0, 7080, result));
  assert(gesture.sample(false, 0, 0, 7100, result) && result.type == RadioTouch::Type::Release); // App owns target boundaries.

  assert(!gesture.sample(true, 10, 10, UINT32_MAX - 10, result));
  assert(gesture.sample(true, 10, 10, 9, result) && result.type == RadioTouch::Type::Down);
  gesture.cancel();
  assert(!gesture.sample(true, 10, 10, 20, result));
  assert(!gesture.sample(false, 0, 0, 30, result));
  assert(gesture.sample(false, 0, 0, 50, result) && result.type == RadioTouch::Type::Cancel);
  assert(!gesture.sample(true, 10, 10, 100, result));
  assert(!gesture.sample(false, 0, 0, 110, result)); // Noise shorter than debounce is ignored.
  assert(!gesture.sample(false, 0, 0, 200, result));
}
void testTouchPlayerSettingsAndGates() {
  reset(); installPage(); playing(catalog->stations[0]);
  touchTap(60, 225); assert(view == View::Listening); // Hardware not ready.
  touchReady = true;
  if (!RadioDisplay::LargeDisplay) { touchTap(60, 120); assert(view == View::Listening); return; }
  touchTap(60, 225);
  assert(view == View::Stations && stationCursor == 2 && audio.startCalls == 0);
  touchTap(150, 225); assert(view == View::Listening);
  touchTap(250, 225); assert(view == View::Settings && ui.setting == 0);
  render();
  touchTap(100, 140); assert(settings.muted && audio.lastVolume == 0 && view == View::Settings);
  handleTouchEvent({RadioTouch::Type::Tap, 100, 140}); // Duplicate release does not toggle back.
  assert(settings.muted);
  touchTap(100, 60); assert(view == View::Wifi && wifi.portalActive());
  render();
  touchTap(150, 225); assert(view == View::Settings && wifi.portalActive());
  touchTap(100, 100); assert(view == View::Listening && audio.startCalls == 1);
  const int oldVolume = settings.volume;
  event(EventType::Rotate, 1); assert(settings.volume == oldVolume + 1 && !settings.muted);
  hold(); assert(view == View::Settings); // Knob hold behavior remains intact.
  click(); assert(view == View::Wifi); // Current setting starts at Wi-Fi.
  hold(); assert(view == View::Settings);
  hold(); assert(view == View::Listening);
  RadioModel::copyText(initializationError, "Startup failed");
  touchTap(250, 225); assert(view == View::Listening);
}

void assertTouchRectangle(const TouchTarget& expected, int x, int y, int width, int height) {
  for (int py = y; py < y + height; ++py) for (int px = x; px < x + width; ++px)
    assert(touchTargetAt(px, py) == expected);
}
void assertPaintedTouchCoverage(const TouchTarget& expected, int x, int y, int width, int height) {
  unsigned painted = 0;
  for (int py = y; py < y + height; ++py) for (int px = x; px < x + width; ++px) {
    const uint16_t pixel = canvas.at(px, py);
    if (pixel == RadioDisplay::Black || pixel == RadioDisplay::RuleColor) continue;
    ++painted;
    assert(touchTargetAt(px, py) == expected);
  }
  assert(painted); // Exercise real rendered icons/text, not only empty coordinates.
}
void assertFocusedTouchCoverage(const TouchTarget& expected) {
  int left = RadioDisplay::ScreenWidth, top = RadioDisplay::ScreenHeight, right = -1, bottom = -1;
  for (int y = 0; y < RadioDisplay::ScreenHeight; ++y)
    for (int x = 0; x < RadioDisplay::ScreenWidth; ++x)
      if (canvas.at(x, y) == RadioDisplay::FocusBackground) {
        left = min(left, x); right = max(right, x); top = min(top, y); bottom = max(bottom, y);
      }
  assert(right >= left && bottom >= top);
  // Derive these bounds from the actual focus rectangle, independently of the
  // hit-test implementation. Include glyph holes and blank button padding.
  assertTouchRectangle(expected, left, top, right - left + 1, bottom - top + 1);
}
void testTouchCoversPaintedControls() {
  if (!RadioDisplay::LargeDisplay) return;
  using namespace RadioDisplay;
  const int footerHeight = ScreenHeight - FooterTop;
  const auto twoActionFooter = [&](TouchAction left, TouchAction right) {
    assertTouchRectangle(TouchTarget(left), 0, FooterTop, ScreenWidth / 2, footerHeight);
    assertTouchRectangle(TouchTarget(right), ScreenWidth / 2, FooterTop, ScreenWidth / 2, footerHeight);
    assertPaintedTouchCoverage(TouchTarget(left), 0, FooterTop, ScreenWidth / 2, footerHeight);
    assertPaintedTouchCoverage(TouchTarget(right), ScreenWidth / 2, FooterTop, ScreenWidth / 2, footerHeight);
  };

  reset(); touchReady = true; installPage(); playing(catalog->stations[0]); render();
  twoActionFooter(TouchAction::Stations, TouchAction::Settings);
  assert(touchTargetAt(20, FooterTop - 1).action == TouchAction::None);

  enter(View::Settings);
  for (int row = 0; row < 3; ++row) {
    ui.setting = row; render();
    const TouchTarget expected(TouchAction::SettingRow, row);
    assertFocusedTouchCoverage(expected);
    // The focused row is 34px tall, including its icon, text and six pixels of
    // top padding; sweep the full background, not just a label's centre.
    const int rowTop = 56 + row * 40 - 6;
    assertTouchRectangle(expected, 8, rowTop, ScreenWidth - 16, 34);
    assertPaintedTouchCoverage(expected, 0, rowTop, ScreenWidth, 34);
    assert(touchTargetAt(80, rowTop - 1).action == TouchAction::None);
    assert(touchTargetAt(80, rowTop + 34).action == TouchAction::None);
  }
  twoActionFooter(TouchAction::SelectSetting, TouchAction::Back);

  enter(View::Wifi); wifi.portal = true; render();
  assertTouchRectangle(TouchTarget(TouchAction::Back), 0, FooterTop, ScreenWidth, footerHeight);
  assertPaintedTouchCoverage(TouchTarget(TouchAction::Back), 0, FooterTop, ScreenWidth, footerHeight);

  reset(); touchReady = true; installPage(20, true, RadioModel::kPageSize); enter(View::Stations);
  for (int selected : {1, 9, 20}) {
    stationCursor = selected; ui.lastStation = selected - 1; render();
    const int first = firstVisibleStation(false, stationAction());
    for (int row = 0; row < VisibleStationRows; ++row) {
      const TouchTarget expected(TouchAction::Station, first + row);
      const int rowTop = FirstStationTop - 2 + row * StationRowHeight;
      assertTouchRectangle(expected, 8, rowTop, ScreenWidth - 16, StationRowHeight);
      assertPaintedTouchCoverage(expected, 0, rowTop, ScreenWidth - 8, StationRowHeight);
      // Adjacent rows meet at one boundary; neither may steal the other's last
      // pixel when the viewport has scrolled to the middle or end of a page.
      if (row + 1 < VisibleStationRows)
        assert(touchTargetAt(100, rowTop + StationRowHeight) ==
               TouchTarget(TouchAction::Station, first + row + 1));
    }
    assertFocusedTouchCoverage(TouchTarget(TouchAction::Station, selected - 1));
  }
  for (Action action : {Action::Previous, Action::Next}) {
    stationCursor = RadioNavigation::cursorFor(action, catalog); render();
    assertFocusedTouchCoverage(TouchTarget(action == Action::Previous ?
                                           TouchAction::PreviousPage : TouchAction::NextPage));
  }
  assertTouchRectangle(TouchTarget(TouchAction::PreviousPage), 8, FooterControlTop, 84, FooterControlHeight);
  assertTouchRectangle(TouchTarget(TouchAction::NextPage), ScreenWidth - 92, FooterControlTop, 84, FooterControlHeight);
  assertPaintedTouchCoverage(TouchTarget(TouchAction::PreviousPage), 0, FooterTop, 96, footerHeight);
  assertPaintedTouchCoverage(TouchTarget(TouchAction::Back), 96, FooterTop, 128, footerHeight);
  assertPaintedTouchCoverage(TouchTarget(TouchAction::NextPage), 224, FooterTop, 96, footerHeight);
  assertTouchRectangle(TouchTarget(TouchAction::Back), 96, FooterControlTop, 128, FooterControlHeight);
  assertTouchRectangle(TouchTarget(TouchAction::Filters), ContentLeft, FilterFocusTop,
                       ContentWidth, FilterHeight);
  assertPaintedTouchCoverage(TouchTarget(TouchAction::Filters), 0, FilterFocusTop, ScreenWidth, FilterHeight);

  beginFields(); render();
  const auto filterControls = [&](TouchAction previous, TouchAction centre, TouchAction next) {
    for (const auto& column : {std::pair<int, int>{ContentLeft, 48}, {48, 272}, {272, ScreenWidth - ContentLeft}}) {
      const TouchAction action = column.first == ContentLeft ? previous : column.first == 48 ? centre : next;
      assertTouchRectangle(TouchTarget(action), column.first, FilterFocusTop,
                           column.second - column.first, FilterHeight);
      assertPaintedTouchCoverage(TouchTarget(action), column.first, FilterFocusTop,
                                column.second - column.first, FilterHeight);
    }
  };
  filterControls(TouchAction::FieldPrevious, TouchAction::EditField, TouchAction::FieldNext);
  assertPaintedTouchCoverage(TouchTarget(TouchAction::Back), 0, FooterTop, 96, footerHeight);
  assertPaintedTouchCoverage(TouchTarget(TouchAction::Back), 96, FooterTop, 128, footerHeight);
  assertTouchRectangle(TouchTarget(), 228, FooterControlTop, 84, FooterControlHeight); // Disabled Next.
  beginEdit(Field::Source); render();
  filterControls(TouchAction::ValuePrevious, TouchAction::SetValue, TouchAction::ValueNext);
  assertTouchRectangle(TouchTarget(TouchAction::SetValue), 96, FooterControlTop, 128, FooterControlHeight);
  assertPaintedTouchCoverage(TouchTarget(TouchAction::SetValue), 96, FooterTop, 128, footerHeight);

  reset(); touchReady = true; installPage(20, false); enter(View::Stations); render();
  for (int left : {8, ScreenWidth - 92}) {
    assertTouchRectangle(TouchTarget(), left, FooterControlTop, 84, FooterControlHeight);
    assertPaintedTouchCoverage(TouchTarget(), left, FooterTop, 84, footerHeight);
  }
  requestDirectory(catalog->request.filter, RadioModel::kPageSize, false, Purpose::Page); render();
  assertTouchRectangle(TouchTarget(TouchAction::Back), 0, FooterTop, ScreenWidth, footerHeight);
  assertPaintedTouchCoverage(TouchTarget(TouchAction::Back), 0, FooterTop, ScreenWidth, footerHeight);
  assertTouchRectangle(TouchTarget(), ContentLeft, FilterFocusTop, ContentWidth, FilterHeight);
  assertTouchRectangle(TouchTarget(), 8, FirstStationTop - 2, ScreenWidth - 16,
                       VisibleStationRows * StationRowHeight);
  fail(); stationCursor = RadioNavigation::cursorFor(Action::Retry, catalog, true); render();
  twoActionFooter(TouchAction::Retry, TouchAction::Back);
  assertTouchRectangle(TouchTarget(TouchAction::Filters), ContentLeft, FilterFocusTop, ContentWidth, FilterHeight);
  stationCursor = RadioNavigation::cursorFor(Action::Filters, catalog, true); render();
  twoActionFooter(TouchAction::Filters, TouchAction::Back);
}
void testTouchStationViewportAndBoundaries() {
  if (!RadioDisplay::LargeDisplay) return;
  reset(); touchReady = true; installPage(20); enter(View::Stations);
  stationCursor = 9; ui.lastStation = 8;
  render();
  assert(firstVisibleStation(false, stationAction()) == 7);
  const Station visibleFirst = catalog->stations[7];
  touchTap(100, 80);
  assert(view == View::Listening && audio.startCalls == 1 &&
         !std::strcmp(audio.lastStation.id, visibleFirst.id));

  reset(); touchReady = true; installPage(20, false); enter(View::Stations);
  touchTap(30, 225); touchTap(280, 225); // Both page controls disabled.
  touchTap(94, 225); touchTap(100, 211); // Footer gap and list/footer gap.
  touchTap(317, 80); // A scrollbar tap does not play the station underneath it.
  assert(!search.pending && audio.startCalls == 0 && view == View::Stations);
  touchDown(100, 97); // Last pixel of first hit row.
  handleTouchEvent({RadioTouch::Type::Tap, 100, 98}); // Release in a different row.
  assert(audio.startCalls == 0);
  touchTap(100, 202); assert(audio.startCalls == 0); // Below fifth row.
  touchTap(100, 201); assert(audio.startCalls == 1); // Last pixel of fifth row.
}
void testTouchFilterDraftAndCancellation() {
  if (!RadioDisplay::LargeDisplay) return;
  reset(); touchReady = true; installPage(20); enter(View::Stations);
  touchTap(150, 50); assert(ui.mode == FilterMode::Fields);
  touchTap(300, 50); assert(ui.fieldCursor == 1);
  render();
  touchTap(150, 50); assert(ui.mode == FilterMode::Value && editItem == Field::Location && draft == 0);
  touchTap(300, 50); assert(draft == 1);
  touchTap(20, 50); assert(draft == 0);
  touchTap(300, 50); assert(draft == 1);
  render();
  touchTap(30, 225); // Back is distinct from Set.
  assert(ui.mode == FilterMode::Fields && settings.country == 0 && directory.requests.empty());
  touchTap(150, 50); assert(draft == 0);
  touchTap(300, 50); touchTap(150, 225);
  assert(search.pending && search.request.filter.country == 1 && settings.country == 0);
  succeed(); assert(settings.country == 1 && ui.mode == FilterMode::Browse && audio.startCalls == 0);
  touchTap(150, 50); touchTap(150, 50); // Source field, then its value.
  touchTap(150, 225); // Same value preserves accepted page without refresh.
  assert(ui.mode == FilterMode::Browse && !search.pending && directory.requests.size() == 1);
}
void testTouchLoadingRetryAndContextChanges() {
  if (!RadioDisplay::LargeDisplay) return;
  reset(); touchReady = true; installPage(20); enter(View::Stations);
  const auto* accepted = catalog;
  touchTap(280, 225); assert(search.pending && search.request.offset == 20);
  const auto request = search.request;
  touchTap(100, 80); touchTap(150, 50); // Old rows/filter are inert while loading.
  assert(search.pending && audio.startCalls == 0 && directory.requests.size() == 1);
  render();
  touchTap(150, 225);
  assert(!search.pending && catalog == accepted && view == View::Stations);
  auto* late = page(20); late->request = request; deliver(late);
  assert(catalog == accepted && audio.startCalls == 0);
  touchTap(280, 225); fail(); render();
  assert(search.failed);
  touchTap(60, 225); assert(search.pending); // Visible Retry footer.
  touchDown(150, 225); succeed();
  handleTouchEvent({RadioTouch::Type::Tap, 150, 225}); // Loading Back cannot target the new page.
  assert(view == View::Stations && !search.pending && catalog != accepted);

  touchDown(100, 80); ++catalog->request.generation;
  handleTouchEvent({RadioTouch::Type::Tap, 100, 80}); assert(audio.startCalls == 0);
  enter(View::Listening);
  touchDown(250, 225); event(EventType::Rotate, 1);
  handleTouchEvent({RadioTouch::Type::Tap, 250, 225}); assert(view == View::Listening);
  touchDown(250, 225); enter(View::Settings); enter(View::Listening);
  handleTouchEvent({RadioTouch::Type::Tap, 250, 225}); assert(view == View::Listening);
  event(EventType::ButtonDown); touchTap(250, 225);
  assert(view == View::Listening); event(EventType::ButtonUp);
  touchDown(250, 225); handleTouchEvent({RadioTouch::Type::Cancel, 250, 225});
  handleTouchEvent({RadioTouch::Type::Tap, 250, 225}); assert(view == View::Listening);
}
void testTouchSwipesStayWithinCurrentPage() {
  if (!RadioDisplay::LargeDisplay) return;
  reset(); touchReady = true; installPage(20); enter(View::Stations);
  touchDown(100, 80); handleTouchEvent({RadioTouch::Type::SwipeUp, 100, 80});
  assert(stationCursor == 6 && firstVisibleStation(false, stationAction()) == 4);
  assert(audio.startCalls == 0 && directory.requests.empty());
  touchDown(100, 80); handleTouchEvent({RadioTouch::Type::SwipeDown, 100, 80});
  assert(stationCursor == 1);
  stationCursor = 20; ui.lastStation = 19;
  touchDown(100, 80); handleTouchEvent({RadioTouch::Type::SwipeUp, 100, 80});
  assert(stationCursor == 20 && directory.requests.empty()); // No wrap to Next/Filters.
  touchDown(100, 50);
  handleTouchEvent({RadioTouch::Type::Move, 100, 80}); // Real swipe crosses out of the filter control first.
  handleTouchEvent({RadioTouch::Type::SwipeDown, 100, 50});
  assert(stationCursor == 20); // Swipes must begin within the station list.
  render();
}

RadioTouch::RawPoint rawTouchAt(int x, int y) { return RadioTouch::RawPoint{200 + x * 10, 300 + y * 14}; }
void sampleTouch(bool down, RadioTouch::RawPoint raw = RadioTouch::RawPoint{0, 0}, bool valid = true) {
  RadioHost::now += 20;
  touchPanel.samples.push_back({raw, down, valid});
  serviceTouch();
}
void releaseTouch() { for (int i = 0; i < 4; ++i) sampleTouch(false); }
void calibratePanel(bool failSave = false) {
  assert(calibratingTouch());
  touchStore.writeOK = !failSave;
  releaseTouch();
  for (unsigned index = 0; index < 5; ++index) {
    assert(touchSession.targetIndex() == index);
    render();
    const auto target = RadioTouch::calibrationTarget(index, 320, 240);
    for (int i = 0; i < 8; ++i) sampleTouch(true, rawTouchAt(target.x, target.y));
    releaseTouch();
  }
  assert(touchReady && !calibratingTouch());
  releaseTouch();
}
void testTouchHardwareSessionStorageAndLoop() {
  reset(); beginTouch();
  if (!RadioDisplay::LargeDisplay) {
    assert(!touchReady && !touchHardwareReady && !calibratingTouch()); return;
  }
  installPage(20); playing(catalog->stations[0]);
  const Station playingBefore = playback.requested;
  assert(calibratingTouch() && !touchReady);
  const int volume = settings.volume;
  event(EventType::Rotate, 3); click();
  assert(calibratingTouch() && settings.volume == volume && audio.startCalls == 0);
  calibratePanel(true);
  assert(touchSavePending && !touchStore.hasSaved && touchStore.saveCalls == 1);
  assert(view == View::Listening && audio.startCalls == 0 &&
         !std::strcmp(playback.requested.id, playingBefore.id));
  touchStore.writeOK = true; RadioHost::now += 5000; sampleTouch(false);
  assert(!touchSavePending && touchStore.hasSaved && touchStore.saveCalls == 2);
  const auto saved = touchStore.saved;
  // Actual raw polling -> calibration transform -> gesture -> existing action.
  sampleTouch(true, rawTouchAt(250, 225)); sampleTouch(true, rawTouchAt(250, 225));
  sampleTouch(false); sampleTouch(false);
  assert(view == View::Settings);
  const unsigned reads = touchPanel.reads;
  serviceTouch(); assert(touchPanel.reads == reads); // Never sample every 2 ms UI iteration.
  hold(); assert(view == View::Listening);
  event(EventType::Rotate, 1); assert(settings.volume == volume + 1);
  // An SPI read failure during a tap may not release/click the old target.
  sampleTouch(true, rawTouchAt(250, 225)); sampleTouch(true, rawTouchAt(250, 225));
  sampleTouch(true, rawTouchAt(250, 225), false);
  releaseTouch(); assert(view == View::Listening && !touchContact.active);
  // Saved calibration is loaded at startup without another setup overlay.
  beginTouch(); assert(touchReady && !calibratingTouch());
  releaseTouch();
  event(EventType::ButtonDown); assert(eventButtonDown && gestureArmed);
  Serial.incoming.push_back('t'); loop();
  assert(calibratingTouch() && !touchReady && !touchContact.active);
  event(EventType::ButtonUp); assert(!eventButtonDown && !gestureArmed);
  render(); hold(); assert(touchReady && !calibratingTouch() && view == View::Listening);
  assert(touchStore.saved.checksum == saved.checksum);
  requestTouchCalibration();
  event(EventType::ButtonDown); // This press belongs to the calibration overlay.
  calibratePanel();
  event(EventType::ButtonUp); event(EventType::ShortPress);
  assert(view == View::Listening); // Releasing it cannot open Stations afterward.
  click(); assert(view == View::Stations); hold(); assert(view == View::Listening);
  reset(); beginTouch(); assert(calibratingTouch()); hold();
  assert(!calibratingTouch() && !touchReady && view == View::Listening);
  event(EventType::Rotate, 1); assert(settings.volume == 11); // Knob remains usable after skip.
  reset(); touchPanel.beginOK = false; beginTouch();
  assert(!touchHardwareReady && !touchReady && !calibratingTouch());
  hold(); assert(view == View::Settings);
}
// Exercise physical polling, mapping, gesture recognition and action dispatch
// together; direct Down/Tap injections cannot catch movement-related failures.
void prepareCalibratedControls() {
  reset();
  RadioTouch::RawPoint raw[5];
  for (unsigned index = 0; index < 5; ++index) {
    const auto target = RadioTouch::calibrationTarget(index, 320, 240);
    raw[index] = rawTouchAt(target.x, target.y);
  }
  assert(RadioTouch::fitCalibration(raw, touchPanel.signature(), 320, 240, touchStore.saved));
  touchStore.hasSaved = true;
  beginTouch(); assert(touchReady && !calibratingTouch());
  releaseTouch();
  installPage(20); playing(catalog->stations[0]);
}
void beginControlTouch(int x, int y) {
  sampleTouch(true, rawTouchAt(x, y));
  sampleTouch(true, rawTouchAt(x, y));
}
void testWholeControlTapsThroughRawPolling() {
  if (!RadioDisplay::LargeDisplay) return;
  // Tap from the icon end, middle and text end of both wide footer actions.
  for (int x : {14, 70, 138, 176, 235, 294}) {
    prepareCalibratedControls();
    beginControlTouch(x, 225);
    sampleTouch(true, rawTouchAt(x + 14, 225));
    assert(view == View::Listening && audio.startCalls == 0);
    releaseTouch();
    assert(view == (x < 160 ? View::Stations : View::Settings));
  }
  // A finger can roll vertically inside the same short footer button, too.
  prepareCalibratedControls(); beginControlTouch(250, 215);
  sampleTouch(true, rawTouchAt(250, 235)); releaseTouch();
  assert(view == View::Settings);
  // Settings rows are controls across their full width, without autorepeat.
  prepareCalibratedControls(); openSettings(); beginControlTouch(16, 145);
  sampleTouch(true, rawTouchAt(150, 145));
  sampleTouch(true, rawTouchAt(300, 145));
  assert(!settings.muted);
  releaseTouch(); assert(settings.muted);
  releaseTouch(); assert(settings.muted);
  prepareCalibratedControls(); openSettings(); beginControlTouch(100, 132);
  sampleTouch(true, rawTouchAt(100, 160)); releaseTouch();
  assert(view == View::Settings && settings.muted); // Vertical roll stays in one row, not a list swipe.
  // Small finger movement on any of the five station rows plays that row.
  for (int row = 0; row < 5; ++row) {
    prepareCalibratedControls(); enter(View::Stations);
    const int y = RadioDisplay::FirstStationTop + row * RadioDisplay::StationRowHeight + 8;
    const Station expected = catalog->stations[row];
    beginControlTouch(50, y); sampleTouch(true, rawTouchAt(68, y + 3));
    releaseTouch();
    assert(audio.startCalls == 1 && !std::strcmp(audio.lastStation.id, expected.id));
    assert(view == View::Listening);
  }
  // Arrow labels/icons and wide filter text use the same target rule.
  prepareCalibratedControls(); enter(View::Stations); beginFields();
  beginControlTouch(290, 50); sampleTouch(true, rawTouchAt(305, 50)); releaseTouch();
  assert(ui.fieldCursor == 1 && ui.mode == FilterMode::Fields);
  beginControlTouch(60, 50); sampleTouch(true, rawTouchAt(250, 50)); releaseTouch();
  assert(ui.mode == FilterMode::Value);
}
void testDraggedContactsDoNotRetargetActions() {
  if (!RadioDisplay::LargeDisplay) return;
  for (bool returnToStart : {false, true}) {
    prepareCalibratedControls(); beginControlTouch(140, 225);
    sampleTouch(true, rawTouchAt(180, 225));
    if (returnToStart) sampleTouch(true, rawTouchAt(140, 225));
    releaseTouch();
    assert(view == View::Listening && !touchContact.active);
  }
  prepareCalibratedControls(); enter(View::Stations); beginControlTouch(60, 82);
  sampleTouch(true, rawTouchAt(60, 112));
  sampleTouch(true, rawTouchAt(60, 82)); releaseTouch();
  assert(view == View::Stations && audio.startCalls == 0);
  // A real vertical list swipe still scrolls instead of playing a row.
  beginControlTouch(100, 185); sampleTouch(true, rawTouchAt(100, 145));
  sampleTouch(true, rawTouchAt(100, 110)); releaseTouch();
  assert(stationCursor == 6 && audio.startCalls == 0 && directory.requests.empty());
  // The knob still cancels a touch in progress even if all points stay inside.
  prepareCalibratedControls(); beginControlTouch(240, 225);
  sampleTouch(true, rawTouchAt(255, 225)); event(EventType::Rotate, 1);
  sampleTouch(true, rawTouchAt(265, 225)); releaseTouch();
  assert(view == View::Listening && settings.volume == 11);
  // A transport failure remains a cancellation, never a release candidate.
  prepareCalibratedControls(); beginControlTouch(240, 225);
  sampleTouch(true, rawTouchAt(255, 225));
  sampleTouch(true, rawTouchAt(260, 225), false); releaseTouch();
  assert(view == View::Listening && !touchContact.active);
  // Ignore initial pressure settling before the debounced Down is emitted.
  prepareCalibratedControls(); sampleTouch(true, rawTouchAt(150, 225));
  sampleTouch(true, rawTouchAt(180, 225)); sampleTouch(true, rawTouchAt(194, 225));
  releaseTouch(); assert(view == View::Settings);
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
  testTouchGestureDebounceAndRelease();
  testTouchPlayerSettingsAndGates();
  testTouchCoversPaintedControls();
  testTouchStationViewportAndBoundaries();
  testTouchFilterDraftAndCancellation();
  testTouchLoadingRetryAndContextChanges();
  testTouchSwipesStayWithinCurrentPage();
  testTouchHardwareSessionStorageAndLoop();
  testWholeControlTapsThroughRawPolling();
  testDraggedContactsDoNotRetargetActions();
  reset();
  std::puts("RadioApp integration and pixel tests passed");
}
