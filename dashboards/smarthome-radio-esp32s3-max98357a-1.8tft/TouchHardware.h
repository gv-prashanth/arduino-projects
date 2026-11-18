#ifndef GTS_RADIO_TOUCH_HARDWARE_H
#define GTS_RADIO_TOUCH_HARDWARE_H

#include "TouchConfig.h"
#include "TouchCalibration.h"

#if RADIO_TOUCH_ENABLED
#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#endif

namespace RadioTouch {

#if RADIO_TOUCH_ENABLED
// The UI task owns this reader and the TFT. Call begin after display setup and
// read only between display transfers. An ADC unit remains allocated while
// gpio_config and config_channel switch the shared pads between digital/analog.
class TouchHardware {
 public:
  struct Pins {
    int xp, xm, yp, ym;
    Pins(int xPlus = RADIO_TOUCH_XP, int xMinus = RADIO_TOUCH_XM,
         int yPlus = RADIO_TOUCH_YP, int yMinus = RADIO_TOUCH_YM)
        : xp(xPlus), xm(xMinus), yp(yPlus), ym(yMinus) {}
  };
  explicit TouchHardware(Pins pins = Pins()) : pins_(pins) {}
  TouchHardware(const TouchHardware&) = delete;
  TouchHardware& operator=(const TouchHardware&) = delete;
  ~TouchHardware() { if (adc_) adc_oneshot_del_unit(adc_); }

  bool begin() {
    if (ready_) return true;
    if (!validPins() || !chooseSense(pins_.yp, pins_.ym, senseX_, channelX_) ||
        !chooseSense(pins_.xp, pins_.xm, senseY_, channelY_)) {
      error_ = "Touch pins need two distinct ADC1-capable axes";
      return false;
    }
    if (!adc_) {
      adc_oneshot_unit_init_cfg_t config{};
      config.unit_id = ADC_UNIT_1;
      config.ulp_mode = ADC_ULP_MODE_DISABLE;
      if (adc_oneshot_new_unit(&config, &adc_) != ESP_OK) {
        adc_ = nullptr;
        error_ = "Touch ADC1 unavailable";
        return false;
      }
    }
    ready_ = restore();
    if (ready_) error_ = "";
    return ready_;
  }

  // false means an invalid sample, not a finger release. Leave outputs intact
  // in that case so callers cannot turn ADC errors into accidental tap events.
  bool read(RawPoint& point, bool& pressed) {
    if (!ready_) return false;
    Restore guard(*this);
    if (!idleControls()) return fail("Touch LCD deselect failed");
    bool contact = false;
    if (!contactDown(contact)) return fail("Touch contact GPIO failed");
    if (!contact) {
      if (!guard.finish()) return false;
      pressed = false;
      error_ = "";
      return true;
    }
    RawPoint sample{};
    if (!readAxis(pins_.xp, pins_.xm, senseX_, channelX_, sample.x) ||
        !readAxis(pins_.yp, pins_.ym, senseY_, channelY_, sample.y))
      return fail("Touch ADC sample invalid");
    // A finger lifting during the two axis groups must not produce a final
    // apparently-valid coordinate from two different electrical states.
    if (!contactDown(contact)) return fail("Touch contact GPIO failed");
    if (!guard.finish()) return false;
    pressed = contact;
    if (contact) point = sample;
    error_ = "";
    return true;
  }

  uint32_t signature() const {
    uint32_t hash = 2166136261UL;
    const uint32_t words[] = {0x34575231UL, uint32_t(pins_.xp), uint32_t(pins_.xm),
                              uint32_t(pins_.yp), uint32_t(pins_.ym), 12, 1};
    for (unsigned i = 0; i < sizeof(words) / sizeof(words[0]); ++i)
      for (unsigned shift = 0; shift < 32; shift += 8) {
        hash ^= (words[i] >> shift) & 255U;
        hash *= 16777619UL;
      }
    return hash;
  }
  const char* error() const { return error_; }

 private:
  Pins pins_;
  adc_oneshot_unit_handle_t adc_ = nullptr;
  adc_channel_t channelX_{}, channelY_{};
  int senseX_ = -1, senseY_ = -1;
  bool ready_ = false;
  const char* error_ = "";

  class Restore {
   public:
    explicit Restore(TouchHardware& owner) : owner_(owner) {}
    ~Restore() { if (pending_) owner_.restore(); }
    bool finish() { pending_ = false; return owner_.restore(); }
   private:
    TouchHardware& owner_;
    bool pending_ = true;
  };
  static uint64_t pinMask(int pin) { return uint64_t(1) << pin; }
  uint64_t touchMask() const { return pinMask(pins_.xp) | pinMask(pins_.xm) | pinMask(pins_.yp) | pinMask(pins_.ym); }
  static bool sharedPin(int pin) {
    return pin == TFT_D0 || pin == TFT_D1 || pin == TFT_D2 || pin == TFT_D3 ||
        pin == TFT_D4 || pin == TFT_D5 || pin == TFT_D6 || pin == TFT_D7 ||
        pin == TFT_DC || pin == TFT_WR;
  }
  bool validPins() const {
    const int values[] = {pins_.xp, pins_.xm, pins_.yp, pins_.ym};
    for (unsigned i = 0; i < 4; ++i) {
      if (values[i] < 0 || values[i] >= 49 || values[i] == TFT_CS ||
          values[i] == TFT_RD || values[i] == TFT_RST || !sharedPin(values[i])) return false;
      for (unsigned j = 0; j < i; ++j) if (values[i] == values[j]) return false;
    }
    return true;
  }
  static bool chooseSense(int first, int second, int& pin, adc_channel_t& channel) {
    const int choices[] = {first, second};
    for (unsigned i = 0; i < 2; ++i) {
      adc_unit_t unit{};
      adc_channel_t candidate{};
      if (adc_oneshot_io_to_channel(choices[i], &unit, &candidate) == ESP_OK && unit == ADC_UNIT_1) {
        pin = choices[i]; channel = candidate; return true;
      }
    }
    return false;
  }
  static bool mode(uint64_t mask, gpio_mode_t direction, bool pullup = false) {
    gpio_config_t config{};
    config.pin_bit_mask = mask;
    config.mode = direction;
    config.pull_up_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&config) == ESP_OK;
  }
  static bool level(int pin, int value) { return gpio_set_level(gpio_num_t(pin), value) == ESP_OK; }
  bool idleControls() { return level(TFT_CS, 1) && level(TFT_RD, 1); }
  bool restore() {
    // Preload output latches before enabling outputs. CS remains high while
    // shared control/data pins change. TFT_eSPI supplies data on its next write.
    bool ok = level(TFT_CS, 1);
    ok = level(pins_.xp, 0) && ok; ok = level(pins_.xm, 0) && ok;
    ok = level(pins_.yp, 0) && ok; ok = level(pins_.ym, 0) && ok;
    ok = level(TFT_CS, 1) && ok; ok = level(TFT_RD, 1) && ok;
    ok = level(TFT_WR, 1) && ok; ok = level(TFT_DC, 1) && ok;
    ok = mode(touchMask() | pinMask(TFT_CS) | pinMask(TFT_RD) | pinMask(TFT_WR) | pinMask(TFT_DC), GPIO_MODE_OUTPUT) && ok;
    if (!ok) { ready_ = false; error_ = "Touch cannot restore TFT pins"; }
    return ok;
  }
  bool fail(const char* message) { error_ = message; return false; }
  bool contactDown(bool& down) {
    if (!mode(touchMask(), GPIO_MODE_INPUT)) return false;
    if (!level(pins_.xp, 0) || !level(pins_.xm, 0) ||
        !mode(pinMask(pins_.xp) | pinMask(pins_.xm), GPIO_MODE_OUTPUT) ||
        !mode(pinMask(pins_.yp), GPIO_MODE_INPUT, true)) return false;
    delayMicroseconds(600);
    down = gpio_get_level(gpio_num_t(pins_.yp)) == 0;
    return true;
  }
  bool readAxis(int high, int low, int sense, adc_channel_t channel, int32_t& value) {
    if (!mode(touchMask(), GPIO_MODE_INPUT) || !level(high, 1) || !level(low, 0) ||
        !mode(pinMask(high) | pinMask(low), GPIO_MODE_OUTPUT)) return false;
    adc_oneshot_chan_cfg_t config{};
    config.bitwidth = ADC_BITWIDTH_12;
    config.atten = ADC_ATTEN_DB_12;
    // Required every axis group: the previous digital restore changed pad mux.
    if (adc_oneshot_config_channel(adc_, channel, &config) != ESP_OK) return false;
    (void)sense; // channel was selected from this validated ADC1 electrode.
    delayMicroseconds(80);
    int discarded = 0, samples[3] = {};
    if (adc_oneshot_read(adc_, channel, &discarded) != ESP_OK) return false;
    for (unsigned i = 0; i < 3; ++i)
      if (adc_oneshot_read(adc_, channel, &samples[i]) != ESP_OK || samples[i] < 0 || samples[i] > 4095)
        return false;
    for (unsigned i = 0; i < 3; ++i)
      for (unsigned j = i + 1; j < 3; ++j)
        if (samples[j] < samples[i]) { const int t = samples[i]; samples[i] = samples[j]; samples[j] = t; }
    if (samples[2] - samples[0] > 160) return false;
    value = samples[1];
    return true;
  }
};
#else
class TouchHardware {
 public:
  bool begin() { return false; }
  bool read(RawPoint&, bool&) { return false; }
  uint32_t signature() const { return 0; }
  const char* error() const { return "Touch disabled for this display"; }
};
#endif

} // namespace RadioTouch
#endif // GTS_RADIO_TOUCH_HARDWARE_H
