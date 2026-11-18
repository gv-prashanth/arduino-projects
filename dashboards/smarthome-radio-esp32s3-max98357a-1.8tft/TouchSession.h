#ifndef GTS_RADIO_TOUCH_SESSION_H
#define GTS_RADIO_TOUCH_SESSION_H

#include "TouchCalibration.h"
#include "TouchInteraction.h"

namespace RadioTouch {

enum class SessionStatus : uint8_t { Inactive, Ready, TouchTarget, HoldStill, Release, Retry };

// Owns the transition from raw panel samples to calibrated application events.
// It has no GPIO, persistence or drawing dependencies, and never blocks.
class Session {
 public:
  static constexpr uint32_t StableMs = 100, ReleaseMs = 40;
  static constexpr unsigned MinimumSamples = 4, MaximumSamples = 64;
  static constexpr int32_t StableSpan = 120;

  void begin(uint32_t signature, int width, int height, const Calibration* saved = nullptr) {
    signature_ = signature; width_ = width; height_ = height;
    configured_ = width >= 96 && width <= 1024 && height >= 96 && height <= 1024;
    haveCalibration_ = configured_ && saved && saved->valid(signature, uint32_t(width), uint32_t(height));
    calibration_ = haveCalibration_ ? *saved : Calibration{};
    changed_ = false; target_ = 0; retry_ = false;
    calibrating_ = configured_ && !haveCalibration_;
    resetContact(); armFreshRelease();
    setStatus(!configured_ ? SessionStatus::Inactive : calibrating_ ?
              SessionStatus::TouchTarget : SessionStatus::Ready);
    ++revision_;
  }

  void requestCalibration() {
    if (!configured_) return;
    calibrating_ = true; target_ = 0; retry_ = false;
    resetContact(); armFreshRelease();
    setStatus(SessionStatus::TouchTarget);
    ++revision_;
  }

  void cancelCalibration() {
    if (!calibrating_) return;
    calibrating_ = false;
    resetContact(); armFreshRelease();
    setStatus(haveCalibration_ ? SessionStatus::Ready : SessionStatus::Inactive);
    ++revision_;
  }

  bool calibrating() const { return calibrating_; }
  bool ready() const { return configured_ && !calibrating_ && haveCalibration_; }
  unsigned targetIndex() const { return target_; }
  SessionStatus status() const { return status_; }
  uint32_t revision() const { return revision_; }
  const Calibration& calibration() const { return calibration_; }
  bool takeCalibrationChanged() { const bool changed = changed_; changed_ = false; return changed; }
  // A hardware read failure is unknown contact state, never a release/click.
  void cancelContact() {
    resetContact(); armFreshRelease();
    if (calibrating_) setStatus(idleStatus());
  }

  // Returns true only for an application gesture event. Calibration contacts
  // are consumed completely, including their final release.
  bool sample(bool pressed, const RawPoint& raw, uint32_t now, Event& out) {
    if (!configured_ || (!calibrating_ && !haveCalibration_)) return false;
    if (awaitRelease_) {
      if (pressed) {
        gateReleaseArmed_ = false;
        if (calibrating_) setStatus(SessionStatus::Release);
      } else {
        if (!gateReleaseArmed_) { gateReleaseArmed_ = true; gateReleaseAt_ = now; }
        if (uint32_t(now - gateReleaseAt_) >= ReleaseMs) {
          awaitRelease_ = false; gateReleaseArmed_ = false;
          if (calibrating_) setStatus(idleStatus());
        }
      }
      return false;
    }
    if (calibrating_) { sampleCalibration(pressed, raw, now); return false; }
    if (!pressed) return gesture_.sample(false, 0, 0, now, out);
    ScreenPoint point;
    if (!calibration_.map(raw, point)) {
      // An invalid point is a drag outside the calibrated panel, not a finger
      // release that could click the last valid target. Block until real up.
      armFreshRelease();
      out = Event(Type::Cancel);
      return true;
    }
    return gesture_.sample(true, point.x, point.y, now, out);
  }

 private:
  uint32_t signature_ = 0, revision_ = 0;
  int width_ = 0, height_ = 0;
  Calibration calibration_;
  Gesture gesture_;
  RawPoint targets_[5] = {};
  bool configured_ = false, haveCalibration_ = false, calibrating_ = false, changed_ = false;
  bool retry_ = false, contact_ = false, window_ = false, captureReady_ = false;
  bool releasing_ = false, awaitRelease_ = false, gateReleaseArmed_ = false;
  uint32_t stableAt_ = 0, releaseAt_ = 0, gateReleaseAt_ = 0;
  int32_t minimumX_ = 0, maximumX_ = 0, minimumY_ = 0, maximumY_ = 0;
  int64_t sumX_ = 0, sumY_ = 0;
  unsigned count_ = 0, target_ = 0;
  SessionStatus status_ = SessionStatus::Inactive;

  void setStatus(SessionStatus next) {
    if (status_ != next) { status_ = next; ++revision_; }
  }
  SessionStatus idleStatus() const { return retry_ ? SessionStatus::Retry : SessionStatus::TouchTarget; }
  void resetContact() {
    contact_ = window_ = captureReady_ = releasing_ = false;
    count_ = 0; sumX_ = sumY_ = 0;
  }
  void armFreshRelease() {
    gesture_.reset(); awaitRelease_ = true; gateReleaseArmed_ = false;
  }
  void startWindow(const RawPoint& raw, uint32_t now) {
    window_ = true; captureReady_ = false; stableAt_ = now;
    minimumX_ = maximumX_ = raw.x; minimumY_ = maximumY_ = raw.y;
    sumX_ = raw.x; sumY_ = raw.y; count_ = 1;
    setStatus(SessionStatus::HoldStill);
  }
  void sampleCalibration(bool pressed, const RawPoint& raw, uint32_t now) {
    if (pressed) {
      contact_ = true; releasing_ = false;
      if (raw.x < 0 || raw.x > 4095 || raw.y < 0 || raw.y > 4095) {
        window_ = captureReady_ = false; count_ = 0;
        setStatus(SessionStatus::HoldStill);
        return;
      }
      if (!window_) { startWindow(raw, now); return; }
      const int32_t lowX = raw.x < minimumX_ ? raw.x : minimumX_;
      const int32_t highX = raw.x > maximumX_ ? raw.x : maximumX_;
      const int32_t lowY = raw.y < minimumY_ ? raw.y : minimumY_;
      const int32_t highY = raw.y > maximumY_ ? raw.y : maximumY_;
      if (highX - lowX > StableSpan || highY - lowY > StableSpan) {
        startWindow(raw, now);
        return;
      }
      minimumX_ = lowX; maximumX_ = highX; minimumY_ = lowY; maximumY_ = highY;
      if (count_ < MaximumSamples) { sumX_ += raw.x; sumY_ += raw.y; ++count_; }
      if (count_ >= MinimumSamples && uint32_t(now - stableAt_) >= StableMs) {
        captureReady_ = true; setStatus(SessionStatus::Release);
      }
      return;
    }
    if (!contact_) return;
    if (!releasing_) { releasing_ = true; releaseAt_ = now; return; }
    if (uint32_t(now - releaseAt_) < ReleaseMs) return;
    if (!captureReady_ || !count_) { resetContact(); setStatus(idleStatus()); return; }
    targets_[target_] = RawPoint{int32_t(sumX_ / count_), int32_t(sumY_ / count_)};
    resetContact();
    ++target_; retry_ = false; ++revision_;
    if (target_ < 5) { setStatus(SessionStatus::TouchTarget); return; }
    Calibration candidate;
    if (fitCalibration(targets_, signature_, width_, height_, candidate)) {
      calibration_ = candidate; haveCalibration_ = true; calibrating_ = false; changed_ = true;
      setStatus(SessionStatus::Ready);
    } else {
      target_ = 0; retry_ = true; setStatus(SessionStatus::Retry);
    }
    armFreshRelease();
  }
};

} // namespace RadioTouch

#endif // GTS_RADIO_TOUCH_SESSION_H
