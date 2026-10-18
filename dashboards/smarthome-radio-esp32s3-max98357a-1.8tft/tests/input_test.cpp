// Run: c++ -std=c++11 -Wall -Wextra -Werror tests/input_test.cpp -o /tmp/radio-input-test && /tmp/radio-input-test
#define RADIO_INPUT_HOST_TEST
#include "../RotaryInput.h"

#include <cassert>
#include <iostream>
#include <vector>

using RadioInput::Event;
using RadioInput::EventType;
using RadioInput::InputModel;

static std::vector<Event> drain(InputModel& model) {
  std::vector<Event> events;
  Event event;
  while (model.poll(event)) events.push_back(event);
  return events;
}

static void rotate(InputModel& model, int direction, uint32_t& now, bool buttonHigh = true) {
  static const uint8_t cw[] = {1, 0, 2, 3};
  static const uint8_t ccw[] = {2, 0, 1, 3};
  for (uint8_t state : (direction > 0 ? cw : ccw)) {
    model.sample((state & 2) != 0, (state & 1) != 0, buttonHigh, ++now);
  }
}

static void testShortAndLongWithBounce() {
  InputModel model;
  model.begin(true, true, true, 0);
  // Press bounce must not produce a click or an early debounced down.
  model.sample(true, true, false, 10);
  assert(model.lastActivity() == 10 && !model.buttonDown());
  model.sample(true, true, true, 12);
  model.sample(true, true, false, 15);
  model.sample(true, true, false, 39);
  assert(!model.buttonDown() && drain(model).empty());
  model.sample(true, true, false, 40);
  assert(model.buttonDown() && model.pressedAt() == 40);
  auto events = drain(model);
  assert(events.size() == 1 && events[0].type == EventType::ButtonDown);
  model.sample(true, true, true, 100);
  model.sample(true, true, false, 102);
  model.sample(true, true, true, 105);
  model.sample(true, true, true, 129);
  assert(model.buttonDown() && drain(model).empty());
  model.sample(true, true, true, 130);
  events = drain(model);
  assert(!model.buttonDown() && events.size() == 2);
  assert(events[0].type == EventType::ButtonUp && events[1].type == EventType::ShortPress);

  model.sample(true, true, false, 200);
  model.sample(true, true, true, 203);
  model.sample(true, true, false, 205);
  model.sample(true, true, false, 230);
  events = drain(model);
  assert(events.size() == 1 && events[0].type == EventType::ButtonDown);
  model.sample(true, true, false, 929);
  assert(drain(model).empty());
  model.sample(true, true, false, 930);
  events = drain(model);
  assert(events.size() == 1 && events[0].type == EventType::LongPress && events[0].atMs == 930);
  model.sample(true, true, false, 2000);
  assert(drain(model).empty());
  model.sample(true, true, true, 2010);
  model.sample(true, true, false, 2012);
  model.sample(true, true, true, 2015);
  model.sample(true, true, true, 2040);
  events = drain(model);
  assert(events.size() == 1 && events[0].type == EventType::ButtonUp);
}

static void testQuadratureAndOrdering() {
  InputModel model;
  uint32_t now = 0;
  model.begin(true, true, true, now);
  rotate(model, 1, now);
  rotate(model, 1, now);
  rotate(model, -1, now);
  auto events = drain(model);
  assert(events.size() == 2);
  assert(events[0].type == EventType::Rotate && events[0].delta == 2);
  assert(events[1].type == EventType::Rotate && events[1].delta == -1);

  // Bounce/backtracking within a detent cancels instead of counting clicks.
  model.sample(false, true, true, ++now);  // 11 -> 01
  model.sample(true, true, true, ++now);   // 01 -> 11
  model.sample(false, true, true, ++now);  // 11 -> 01
  model.sample(false, false, true, ++now); // 01 -> 00
  model.sample(false, true, true, ++now);  // reverse
  model.sample(true, true, true, ++now);
  assert(drain(model).empty());

  // Invalid two-bit change requires resynchronization at the next detent.
  model.sample(false, false, true, ++now); // 11 -> 00 invalid
  model.sample(true, false, true, ++now);
  model.sample(true, true, true, ++now);
  assert(drain(model).empty());
  rotate(model, 1, now);
  events = drain(model);
  assert(events.size() == 1 && events[0].delta == 1);

  rotate(model, 1, now);
  model.sample(true, true, false, ++now);
  now += 25;
  model.sample(true, true, false, now);
  rotate(model, 1, now, false);
  events = drain(model);
  assert(events.size() == 3);
  assert(events[0].type == EventType::Rotate && events[0].delta == 1);
  assert(events[1].type == EventType::ButtonDown);
  assert(events[2].type == EventType::Rotate && events[2].delta == 1);
}

static void testRollover() {
  InputModel model;
  const uint32_t start = UINT32_MAX - 30;
  model.begin(true, true, true, start);
  model.sample(true, true, false, start + 5);
  model.sample(true, true, false, start + 29);
  assert(!model.buttonDown());
  model.sample(true, true, false, start + 30);
  assert(model.buttonDown());
  drain(model);
  model.sample(true, true, false, start + 729);
  assert(drain(model).empty());
  model.sample(true, true, false, start + 730);
  auto events = drain(model);
  assert(events.size() == 1 && events[0].type == EventType::LongPress);
  model.sample(true, true, true, start + 800);
  model.sample(true, true, true, start + 825);
  events = drain(model);
  assert(events.size() == 1 && events[0].type == EventType::ButtonUp);
}

static void testOverflowReservesButtons() {
  InputModel model;
  uint32_t now = 0;
  model.begin(true, true, true, now);
  // Alternating directions cannot coalesce. Rotations stop at 56 events.
  for (int i = 0; i < 80; ++i) rotate(model, i % 2 ? -1 : 1, now);
  // At capacity, further CCW movement still coalesces with the CCW tail;
  // the twelve intervening CW events are the only dropped detents.
  assert(model.overflowCount() == 12);
  model.sample(true, true, false, ++now);
  now += 25;
  model.sample(true, true, false, now);
  model.sample(true, true, true, ++now);
  now += 25;
  model.sample(true, true, true, now);
  auto events = drain(model);
  assert(events.size() == 59);
  assert(events[56].type == EventType::ButtonDown);
  assert(events[57].type == EventType::ButtonUp);
  assert(events[58].type == EventType::ShortPress);
  assert(model.overflowCount() == 12);

  // Sustained turns compact without losing detents or overflowing int16_t.
  model.begin(true, true, true, now);
  for (int i = 0; i < 40000; ++i) rotate(model, 1, now);
  events = drain(model);
  assert(events.size() == 2);
  assert(events[0].delta == INT16_MAX);
  assert(events[1].delta == 40000 - INT16_MAX);
  assert(model.overflowCount() == 0);
}

static void testButtonsEvictRotationsWithoutReordering() {
  InputModel model;
  uint32_t now = 0;
  model.begin(true, true, true, now);
  for (int i = 0; i < 56; ++i) rotate(model, i % 2 ? -1 : 1, now);
  for (int i = 0; i < 3; ++i) {
    model.sample(true, true, false, ++now);
    now += 25;
    model.sample(true, true, false, now);
    model.sample(true, true, true, ++now);
    now += 25;
    model.sample(true, true, true, now);
  }
  auto events = drain(model);
  assert(events.size() == 64 && model.overflowCount() == 1);
  assert(events.front().type == EventType::Rotate && events.front().delta == -1);
  for (std::size_t i = 55; i < events.size(); i += 3) {
    assert(events[i].type == EventType::ButtonDown);
    assert(events[i + 1].type == EventType::ButtonUp);
    assert(events[i + 2].type == EventType::ShortPress);
  }
  // Exercise the ring after consuming its previous contents.
  rotate(model, 1, now);
  events = drain(model);
  assert(events.size() == 1 && events[0].delta == 1);
}

static void testStartsBetweenDetentsAndWithButtonHeld() {
  InputModel model;
  model.begin(false, true, false, 100);
  model.sample(false, false, false, 101);
  model.sample(true, false, false, 102);
  model.sample(true, true, false, 103);
  assert(drain(model).empty());  // Do not count a partial first detent.
  model.sample(true, true, false, 125);
  auto events = drain(model);
  assert(events.size() == 1 && events[0].type == EventType::ButtonDown);
  assert(model.pressedAt() == 125 && model.buttonDown());
  model.sample(true, true, false, 825);
  events = drain(model);
  assert(events.size() == 1 && events[0].type == EventType::LongPress);
}

int main() {
  static_assert(InputModel::kTicksPerDetent == 4, "default decoder tests use four ticks");
  testShortAndLongWithBounce();
  testQuadratureAndOrdering();
  testRollover();
  testOverflowReservesButtons();
  testButtonsEvictRotationsWithoutReordering();
  testStartsBetweenDetentsAndWithButtonHeld();
  std::cout << "input tests passed\n";
}
