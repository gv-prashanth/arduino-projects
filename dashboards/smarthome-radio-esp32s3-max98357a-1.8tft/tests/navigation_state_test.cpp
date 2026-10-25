#include "../NavigationState.h"
#include <cassert>
#include <cstdio>

using namespace RadioNavigation;

static RadioCatalog::Result page() {
  RadioCatalog::Result result;
  result.count = 20;
  result.hasNext = true;
  for(int index = 0; index < result.count; ++index) {
    std::snprintf(result.stations[index].id, sizeof(result.stations[index].id), "id-%d", index);
    result.stations[index].source = RadioModel::Source::Akashvani;
  }
  return result;
}
int main() {
  RadioCatalog::Result result = page();
  assert(fieldCount(RadioModel::Source::Akashvani) == 2);
  assert(fieldCount(RadioModel::Source::RadioBrowser) == 5);
  assert(choiceCount(&result, false) == 22);
  assert(actionAt(0, &result, false) == Action::Filters);
  for(int index = 1; index <= 20; ++index) assert(actionAt(index, &result, false) == Action::Station);
  assert(actionAt(21, &result, false) == Action::Next);
  assert(cursorFor(Action::Previous, &result) == -1);
  result.request.offset = 20;
  assert(choiceCount(&result, false) == 23);
  assert(actionAt(21, &result, false) == Action::Previous);
  assert(actionAt(22, &result, false) == Action::Next);
  const EntryFocus successor = entryFocus(&result, result.stations[0]);
  assert(successor.cursor == 2 && !successor.nextPage);
  const EntryFocus nextPage = entryFocus(&result, result.stations[19]);
  assert(nextPage.nextPage && nextPage.lastStation == 19);
  result.hasNext = false;
  const EntryFocus end = entryFocus(&result, result.stations[19]);
  assert(!end.nextPage && end.cursor == 20);
  RadioModel::Station unrelated = result.stations[0];
  unrelated.source = RadioModel::Source::RadioBrowser;
  assert(entryFocus(&result, unrelated).cursor == 1);
  assert(entryFocus(&result, unrelated, true).cursor == 1);
  assert(choiceCount(&result, true) == 2);
  assert(actionAt(1, &result, true) == Action::Retry);
  result.count = 0;
  result.hasNext = true;
  assert(cursorFor(Action::Next, &result) == 2); // Filters, Prev, Next, no fake rows.
  assert(actionAt(1, &result, true) == Action::Retry);
  assert(entryFocus(&result, unrelated).cursor == 0);
  assert(choiceCount(nullptr, false) == 1 && actionAt(0, nullptr, false) == Action::Filters);

  State ui;
  RadioModel::Settings editor, volume;
  Origin origin;
  origin.capture(ui, editor);
  ui.view = View::Stations; ui.mode = FilterMode::Value; ui.draft = 3;
  origin.capture(ui, editor); // Retry never overwrites the first origin.
  volume.volume = 17;
  origin.restore(ui, editor);
  assert(ui.view == View::Listening && ui.mode == FilterMode::Browse && !origin.valid);
  assert(volume.volume == 17); // Back restores UI/editor state, never global volume/audio.
  ui.view = View::Stations; ui.mode = FilterMode::Value; ui.draft = 2;
  origin.capture(ui, editor);
  ui.mode = FilterMode::Browse;
  origin.restore(ui, editor);
  assert(ui.mode == FilterMode::Value && ui.draft == 2);

  State before = ui, after = ui;
  after.lastStation = 19;
  assert(sameControlContext(before, after)); // Repainting cannot invalidate a held press.
  after.mode = FilterMode::Fields;
  assert(!sameControlContext(before, after));
  before.view = after.view = View::Listening;
  after.station = 8;
  assert(sameControlContext(before, after)); // Background catalog focus is not a root control.
  before.view = after.view = View::Settings;
  after.setting = 2;
  assert(!sameControlContext(before, after));
  std::puts("Navigation state tests passed");
}
