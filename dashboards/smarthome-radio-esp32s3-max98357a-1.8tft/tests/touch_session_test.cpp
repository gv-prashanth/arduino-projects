#include "../TouchSession.h"

#include <cassert>
#include <cstdio>
#include <cstdint>

using namespace RadioTouch;

namespace {
const uint32_t signature = 0x23456789;
const RawPoint nowhere = {0, 0};

struct Mapping {
  bool swap, reverseX, reverseY;
  RawPoint raw(ScreenPoint point) const {
    int32_t x = 300 + point.x * 10, y = 400 + point.y * 13;
    if (reverseX) x = 4095 - x;
    if (reverseY) y = 4095 - y;
    return swap ? RawPoint{y, x} : RawPoint{x, y};
  }
};

void releaseGate(Session& session, uint32_t& now) {
  Event event;
  assert(!session.sample(false, nowhere, now, event));
  now += Session::ReleaseMs;
  assert(!session.sample(false, nowhere, now, event));
  ++now;
}

void capture(Session& session, const RawPoint& raw, uint32_t& now) {
  Event event;
  for (unsigned index = 0; index <= 5; ++index) {
    assert(!session.sample(true, raw, now, event));
    now += 20;
  }
  assert(session.status() == SessionStatus::Release);
  assert(!session.sample(false, nowhere, now, event));
  now += Session::ReleaseMs;
  assert(!session.sample(false, nowhere, now, event));
  ++now;
}

Calibration knownCalibration() {
  RawPoint raw[5];
  const Mapping mapping = {false, false, false};
  for (unsigned index = 0; index < 5; ++index) raw[index] = mapping.raw(calibrationTarget(index, 320, 240));
  Calibration result;
  assert(fitCalibration(raw, signature, 320, 240, result));
  return result;
}

void five_targets_calibrate_swapped_and_reversed_panels() {
  for (unsigned swap = 0; swap < 2; ++swap)
    for (unsigned reverseX = 0; reverseX < 2; ++reverseX)
      for (unsigned reverseY = 0; reverseY < 2; ++reverseY) {
        const Mapping mapping = {bool(swap), bool(reverseX), bool(reverseY)};
        Session session;
        session.begin(signature, 320, 240);
        assert(session.calibrating() && !session.ready());
        assert(!session.takeCalibrationChanged());
        uint32_t now = 100;
        releaseGate(session, now);
        for (unsigned target = 0; target < 5; ++target) {
          assert(session.targetIndex() == target);
          capture(session, mapping.raw(calibrationTarget(target, 320, 240)), now);
          assert(session.ready() == (target == 4));
        }
        assert(!session.calibrating() && session.ready());
        assert(session.takeCalibrationChanged());
        assert(!session.takeCalibrationChanged());
        assert(session.calibration().valid(signature, 320, 240));
        // Calibration's last contact cannot be forwarded as an application tap.
        Event event;
        const RawPoint point = mapping.raw(ScreenPoint{100, 100});
        assert(!session.sample(true, point, now, event));
        now += 200;
        assert(!session.sample(true, point, now, event));
        releaseGate(session, now);
        assert(!session.sample(true, point, now, event));
        now += 20;
        assert(session.sample(true, point, now, event) && event.type == Type::Down);
        assert(event.x >= 99 && event.x <= 100 && event.y >= 99 && event.y <= 100);
        now += 20;
        assert(!session.sample(false, nowhere, now, event));
        now += 20;
        assert(session.sample(false, nowhere, now, event) && event.type == Type::Tap);
      }
}

void holds_and_release_bounce_capture_only_once() {
  Session session;
  session.begin(signature, 320, 240);
  uint32_t now = 0;
  releaseGate(session, now);
  const RawPoint point = {600, 800};
  Event event;
  for (unsigned index = 0; index < 100; ++index, now += 20)
    assert(!session.sample(true, point, now, event));
  assert(session.targetIndex() == 0 && !session.takeCalibrationChanged());
  assert(!session.sample(false, nowhere, now, event));
  now += 20;
  assert(!session.sample(true, point, now, event));
  now += 20;
  assert(!session.sample(false, nowhere, now, event));
  now += 39;
  assert(!session.sample(false, nowhere, now, event));
  assert(session.targetIndex() == 0);
  ++now;
  assert(!session.sample(false, nowhere, now, event));
  assert(session.targetIndex() == 1);
  now += 1000;
  assert(!session.sample(false, nowhere, now, event));
  assert(session.targetIndex() == 1);
}

void movement_and_insufficient_samples_do_not_capture() {
  Session session;
  session.begin(signature, 320, 240);
  uint32_t now = 100;
  releaseGate(session, now);
  Event event;
  const RawPoint point = {600, 800};
  assert(!session.sample(true, point, now, event));
  now += 100;
  assert(!session.sample(true, point, now, event)); // Time alone is insufficient.
  assert(session.status() == SessionStatus::HoldStill);
  now += 20;
  assert(!session.sample(false, nowhere, now, event));
  now += 40;
  assert(!session.sample(false, nowhere, now, event));
  assert(session.targetIndex() == 0);
  ++now;
  assert(!session.sample(true, point, now, event));
  for (unsigned index = 0; index < 8; ++index) {
    now += 20;
    const RawPoint moving = {600 + (index % 2 ? 0 : 200), 800};
    assert(!session.sample(true, moving, now, event));
  }
  assert(session.status() == SessionStatus::HoldStill);
  now += 20;
  assert(!session.sample(true, RawPoint{-1, 800}, now, event));
  now += 20;
  assert(!session.sample(false, nowhere, now, event));
  now += 40;
  assert(!session.sample(false, nowhere, now, event));
  assert(session.targetIndex() == 0 && !session.takeCalibrationChanged());
}

void invalid_geometry_retries_and_cancel_preserves_saved_calibration() {
  Session session;
  const Calibration saved = knownCalibration();
  session.begin(signature, 320, 240, &saved);
  assert(session.ready() && !session.calibrating());
  uint32_t now = 100;
  session.requestCalibration();
  assert(!session.ready() && session.calibrating());
  releaseGate(session, now);
  for (unsigned index = 0; index < 5; ++index) capture(session, RawPoint{2000, 2000}, now);
  assert(session.calibrating() && !session.ready() && session.targetIndex() == 0);
  assert(session.status() == SessionStatus::Retry && !session.takeCalibrationChanged());
  assert(session.calibration().checksum == saved.checksum);
  session.cancelCalibration();
  assert(session.ready() && !session.calibrating());
  assert(session.calibration().checksum == saved.checksum);
  Event event;
  assert(!session.sample(true, RawPoint{1000, 1000}, now, event));
  now += 100;
  assert(!session.sample(true, RawPoint{1000, 1000}, now, event));
  releaseGate(session, now);
  assert(!session.sample(true, RawPoint{1000, 1000}, now, event));
  now += 20;
  assert(session.sample(true, RawPoint{1000, 1000}, now, event) && event.type == Type::Down);
}

void missing_or_wrong_hardware_calibration_starts_setup_and_can_skip() {
  const Calibration saved = knownCalibration();
  Session session;
  session.begin(signature + 1, 320, 240, &saved);
  assert(session.calibrating() && !session.ready());
  session.cancelCalibration();
  assert(!session.calibrating() && !session.ready() && session.status() == SessionStatus::Inactive);
  Event event;
  assert(!session.sample(true, RawPoint{1000, 1000}, 0, event));
  assert(!session.sample(true, RawPoint{1000, 1000}, 1000, event));
  session.requestCalibration();
  assert(session.calibrating());
  session.begin(signature, 160, 128, &saved);
  assert(session.calibrating() && !session.ready());
  session.begin(signature, 0, 240, &saved);
  assert(!session.calibrating() && !session.ready());
  session.requestCalibration();
  assert(!session.calibrating());
}

void invalid_mapped_contact_is_cancelled_until_real_release() {
  const Calibration saved = knownCalibration();
  const Mapping mapping = {false, false, false};
  Session session;
  session.begin(signature, 320, 240, &saved);
  uint32_t now = 100;
  releaseGate(session, now);
  Event event;
  const RawPoint valid = mapping.raw(ScreenPoint{100, 100});
  assert(!session.sample(true, valid, now, event));
  now += 20;
  assert(session.sample(true, valid, now, event) && event.type == Type::Down);
  now += 20;
  assert(session.sample(true, RawPoint{-1, 3000}, now, event) && event.type == Type::Cancel);
  now += 500;
  assert(!session.sample(true, valid, now, event));
  releaseGate(session, now);
  assert(!session.sample(true, valid, now, event));
  now += 20;
  assert(session.sample(true, valid, now, event) && event.type == Type::Down);
  now += 20;
  assert(!session.sample(false, nowhere, now, event));
  now += 20;
  assert(session.sample(false, nowhere, now, event) && event.type == Type::Tap);
}

void timing_survives_millis_rollover() {
  Session session;
  session.begin(signature, 320, 240);
  uint32_t now = UINT32_MAX - 60;
  releaseGate(session, now);
  const Mapping mapping = {false, false, false};
  capture(session, mapping.raw(calibrationTarget(0, 320, 240)), now);
  assert(now < 200 && session.targetIndex() == 1);
}
} // namespace

int main() {
  five_targets_calibrate_swapped_and_reversed_panels();
  holds_and_release_bounce_capture_only_once();
  movement_and_insufficient_samples_do_not_capture();
  invalid_geometry_retries_and_cancel_preserves_saved_calibration();
  missing_or_wrong_hardware_calibration_starts_setup_and_can_skip();
  invalid_mapped_contact_is_cancelled_until_real_release();
  timing_survives_millis_rollover();
  std::puts("Touch calibration session tests passed");
}
