#pragma once

#include "CatalogTypes.h"

namespace RadioNavigation {
enum class View : uint8_t { Listening, Settings, Stations, Wifi };
enum class FilterMode : uint8_t { Browse, Fields, Value };
enum class Field : uint8_t { Source, Location, Genre, Codec, Bitrate };
enum class Action : uint8_t { None, Filters, Station, Previous, Next, Retry };

struct State {
  View view = View::Listening;
  FilterMode mode = FilterMode::Browse;
  Field field = Field::Source;
  int station = 1, lastStation = 0, setting = 0, fieldCursor = 0, draft = 0;
};
inline bool operator==(const State& a, const State& b) {
  return a.view == b.view && a.mode == b.mode && a.field == b.field &&
      a.station == b.station && a.lastStation == b.lastStation && a.setting == b.setting &&
      a.fieldCursor == b.fieldCursor && a.draft == b.draft;
}
inline bool sameControlContext(const State& a, const State& b) {
  if(a.view != b.view) return false;
  if(a.view == View::Settings) return a.setting == b.setting;
  if(a.view == View::Stations) return a.mode == b.mode && a.field == b.field &&
      a.station == b.station && a.fieldCursor == b.fieldCursor && a.draft == b.draft;
  return true; // Background catalog changes do not change player/Wi-Fi controls.
}
inline int fieldCount(RadioModel::Source source) {
  return source == RadioModel::Source::Akashvani ? 2 : 5;
}
inline int choiceCount(const RadioCatalog::Result* page, bool failed) {
  return failed ? 2 : 1 + (page ? page->count + (page->request.offset ? 1 : 0) + (page->hasNext ? 1 : 0) : 0);
}
inline Action actionAt(int cursor, const RadioCatalog::Result* page, bool failed) {
  if(cursor < 0 || cursor >= choiceCount(page, failed)) return Action::None;
  if(!cursor) return Action::Filters;
  if(failed) return Action::Retry;
  if(!page) return Action::None;
  if(cursor <= page->count) return Action::Station;
  int action = cursor - page->count - 1;
  if(page->request.offset) { if(!action) return Action::Previous; --action; }
  return page->hasNext && !action ? Action::Next : Action::None;
}
inline int cursorFor(Action wanted, const RadioCatalog::Result* page, bool failed = false) {
  for(int cursor = 0; cursor < choiceCount(page, failed); ++cursor)
    if(actionAt(cursor, page, failed) == wanted) return cursor;
  return -1;
}
struct EntryFocus { int cursor = 0, lastStation = 0; bool nextPage = false; };
inline EntryFocus entryFocus(const RadioCatalog::Result* page, const RadioModel::Station& playing,
                             bool failed = false) {
  EntryFocus focus;
  if(failed) { focus.cursor = 1; return focus; }
  if(!page || !page->count) return focus;
  const int found = RadioModel::findStation(page->stations, page->count, playing);
  if(found < 0) { focus.cursor = 1; return focus; }
  focus.lastStation = found;
  if(found + 1 < page->count) focus.cursor = found + 2;
  else { focus.cursor = found + 1; focus.nextPage = page->hasNext; }
  return focus;
}

// Only UI state is rolled back. Audio, volume, Wi-Fi and accepted catalog/settings
// are owned independently and must never be copied back by a cancelled request.
struct Origin {
  bool valid = false;
  State state;
  RadioModel::Settings editor;
  void capture(const State& ui, const RadioModel::Settings& edit) {
    if(valid) return;
    valid = true; state = ui; editor = edit;
  }
  void restore(State& ui, RadioModel::Settings& edit) {
    if(valid) { ui = state; edit = editor; }
    valid = false;
  }
};
} // namespace RadioNavigation
