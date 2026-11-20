#ifndef GTS_RADIO_TOUCH_RESISTIVE_H
#define GTS_RADIO_TOUCH_RESISTIVE_H

#include "TouchCalibration.h"
#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>

namespace RadioTouch {

// Direct four-wire panel on a parallel UNO shield. The default electrodes come
// from the user's working Uno touch sketch, translated through the known LCD
// wiring. The UI task owns this reader and every TFT transfer.
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
  ~TouchHardware() {
    for (unsigned index = 0; index < 2; ++index)
      if (adc_[index]) adc_oneshot_del_unit(adc_[index]);
  }

  bool begin() {
    if (ready_) return true;
    if (!validPins()) return fail("Resistive touch needs two separate LCD control/data pairs; CS and WR cannot both be shared");
    // Match TouchScreen::getPoint() in the working Uno sketch exactly. On a
    // buffered shield the opposite (LCD data) end is not interchangeable with
    // the control-side ADC input. In particular, keep D0/D1 out of analog mode.
    if (!chooseSense(pins_.yp, senseX_) || !chooseSense(pins_.xm, senseY_))
      return fail("Resistive touch YP and XM must be ADC-capable");
    Restore guard(*this);
    if (!guard.finish()) return false;
    ready_ = true; error_ = "";
    return true;
  }

  // A transport/conversion failure leaves both outputs untouched. The UI must
  // cancel the contact rather than turn an unknown state into a release/tap.
  bool read(RawPoint& point, bool& pressed) {
    if (!ready_) return false;
    Restore guard(*this);
    bool down = false;
    if (!contact(down)) return false;
    RawPoint next{};
    if (down) {
      if (!axis(pins_.xp, pins_.xm, senseX_, next.x) ||
          !axis(pins_.yp, pins_.ym, senseY_, next.y) || !contact(down)) return false;
    }
    if (!guard.finish()) return false;
    if (down) point = next;
    pressed = down; error_ = "";
    return true;
  }

  const Pins& pins() const { return pins_; }
  const char* error() const { return error_; }
  uint32_t signature() const {
    uint32_t hash = 2166136261UL;
    const uint32_t words[] = {0x34575233UL, uint32_t(pins_.xp), uint32_t(pins_.xm),
        uint32_t(pins_.yp), uint32_t(pins_.ym), 12, 3};
    for (unsigned index = 0; index < sizeof(words) / sizeof(words[0]); ++index)
      for (unsigned shift = 0; shift < 32; shift += 8) {
        hash ^= (words[index] >> shift) & 255U; hash *= 16777619UL;
      }
    return hash;
  }

 private:
  struct Sense { int pin = -1; adc_unit_t unit{}; adc_channel_t channel{}; };
  Pins pins_;
  Sense senseX_, senseY_;
  adc_oneshot_unit_handle_t adc_[2] = {};
  bool ready_ = false;
  const char* error_ = "";

  class Restore {
   public:
    explicit Restore(TouchHardware& owner) : owner_(owner) {}
    ~Restore() { if (pending_) owner_.restore(); }
    bool finish() { const bool ok = owner_.restore(); pending_ = !ok; return ok; }
   private:
    TouchHardware& owner_;
    bool pending_ = true;
  };

  static uint64_t pinMask(int pin) { return uint64_t(1) << pin; }
  static uint64_t dataMask() {
    return pinMask(TFT_D0) | pinMask(TFT_D1) | pinMask(TFT_D2) | pinMask(TFT_D3) |
        pinMask(TFT_D4) | pinMask(TFT_D5) | pinMask(TFT_D6) | pinMask(TFT_D7);
  }
  uint64_t touchMask() const { return pinMask(pins_.xp) | pinMask(pins_.xm) | pinMask(pins_.yp) | pinMask(pins_.ym); }
  static bool dataPin(int pin) {
    return pin == TFT_D0 || pin == TFT_D1 || pin == TFT_D2 || pin == TFT_D3 ||
        pin == TFT_D4 || pin == TFT_D5 || pin == TFT_D6 || pin == TFT_D7;
  }
  static bool controlPin(int pin) { return pin == TFT_WR || pin == TFT_DC || pin == TFT_CS; }
  bool shares(int pin) const { return pins_.xp == pin || pins_.xm == pin || pins_.yp == pin || pins_.ym == pin; }
  bool validPins() const {
    const int values[] = {pins_.xp, pins_.xm, pins_.yp, pins_.ym};
    for (unsigned index = 0; index < 4; ++index) {
      if (!dataPin(values[index]) && !controlPin(values[index])) return false;
      for (unsigned previous = 0; previous < index; ++previous)
        if (values[index] == values[previous]) return false;
    }
    return dataPin(pins_.xp) != dataPin(pins_.xm) &&
        dataPin(pins_.yp) != dataPin(pins_.ym) && !(shares(TFT_CS) && shares(TFT_WR));
  }
  bool fail(const char* message) { error_ = message; return false; }
  static bool mode(uint64_t mask, gpio_mode_t direction, bool pullup = false) {
    gpio_config_t config{};
    config.pin_bit_mask = mask; config.mode = direction;
    config.pull_up_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE; config.intr_type = GPIO_INTR_DISABLE;
    // gpio_config also takes ADC pads back out of RTC/analog mode. Arduino's
    // cached pinMode ownership cannot restore pads changed by TFT fast writes.
    return gpio_config(&config) == ESP_OK;
  }
  static bool level(int pin, int value) { return gpio_set_level(gpio_num_t(pin), value) == ESP_OK; }
  static bool output(int pin, int value) { return level(pin, value) && mode(pinMask(pin), GPIO_MODE_OUTPUT); }

  bool restore() {
    // Select high must become a driven GPIO BEFORE WR can acquire a rising
    // edge. This also protects shields whose touch panel shares LCD CS.
    if (!output(TFT_CS, 1)) {
      ready_ = false;
      return fail("Resistive touch could not restore LCD select");
    }
    bool ok = output(TFT_RD, 1);
    const uint64_t pads = touchMask() & dataMask();
    const int data[] = {TFT_D0, TFT_D1, TFT_D2, TFT_D3, TFT_D4, TFT_D5, TFT_D6, TFT_D7};
    for (unsigned index = 0; index < 8; ++index)
      if (pads & pinMask(data[index])) ok = level(data[index], 0) && ok;
    if (pads) ok = mode(pads, GPIO_MODE_OUTPUT) && ok;
    ok = output(TFT_DC, 1) && ok;
    ok = output(TFT_WR, 1) && ok;
    if (!ok) { ready_ = false; error_ = "Resistive touch could not restore LCD GPIO"; }
    return ok;
  }

  bool allocate(const Sense& sense) {
    const unsigned index = sense.unit == ADC_UNIT_1 ? 0U : 1U;
    if (!adc_[index]) {
      adc_oneshot_unit_init_cfg_t config{};
      config.unit_id = sense.unit; config.ulp_mode = ADC_ULP_MODE_DISABLE;
      if (adc_oneshot_new_unit(&config, &adc_[index]) != ESP_OK) {
        adc_[index] = nullptr; return false;
      }
    }
    return true;
  }
  bool chooseSense(int pin, Sense& result) {
    Sense candidate;
    candidate.pin = pin;
    if (adc_oneshot_io_to_channel(pin, &candidate.unit, &candidate.channel) != ESP_OK ||
        (candidate.unit != ADC_UNIT_1 && candidate.unit != ADC_UNIT_2) || !allocate(candidate))
      return false;
    result = candidate;
    return true;
  }
  bool configure(const Sense& sense) {
    adc_oneshot_chan_cfg_t config{};
    config.bitwidth = ADC_BITWIDTH_12;
    config.atten = ADC_ATTEN_DB_12;
    const unsigned index = sense.unit == ADC_UNIT_1 ? 0U : 1U;
    if (adc_oneshot_config_channel(adc_[index], sense.channel, &config) != ESP_OK)
      return fail("Resistive touch ADC configuration failed");
    // Reconfigure after every digital/analog transition, including an already
    // configured channel: this restores its pad mux and disables pull resistors.
    return true;
  }
  bool median(const Sense& sense, int& value, bool stable = false) {
    const unsigned index = sense.unit == ADC_UNIT_1 ? 0U : 1U;
    int discarded = 0, values[3] = {};
    if (adc_oneshot_read(adc_[index], sense.channel, &discarded) != ESP_OK || discarded < 0 || discarded > 4095)
      return fail("Resistive touch ADC read failed");
    for (unsigned sample = 0; sample < 3; ++sample)
      if (adc_oneshot_read(adc_[index], sense.channel, &values[sample]) != ESP_OK ||
          values[sample] < 0 || values[sample] > 4095)
        return fail("Resistive touch ADC sample invalid");
    for (unsigned i = 0; i < 3; ++i)
      for (unsigned j = i + 1; j < 3; ++j)
        if (values[j] < values[i]) { const int swap = values[i]; values[i] = values[j]; values[j] = swap; }
    if (stable && values[2] - values[0] > 160) return fail("Resistive touch sample is unstable");
    value = values[1]; return true;
  }

  bool neutral() {
    if (!output(TFT_CS, 1) || !output(TFT_RD, 1)) return fail("Touch LCD deselect failed");
    // The verified shield shares CS, but not WR. Keep the write strobe inactive
    // throughout every contact/ADC phase, even while CS is floating or analog.
    // For a WR-sharing variant CS remains driven high instead. Never assert
    // WR just to park the bus while changing CS/DC and the electrode voltages.
    if (!shares(TFT_WR) && !output(TFT_WR, 1)) return fail("Touch LCD write guard failed");
    if (!mode(touchMask(), GPIO_MODE_INPUT)) return fail("Touch electrode GPIO failed");
    return true;
  }
  bool contactOne(int first, int second, const Sense& sense, bool& down) {
    if (!neutral() || !level(first, 0) || !level(second, 0) ||
        !mode(pinMask(first) | pinMask(second), GPIO_MODE_OUTPUT) ||
        !mode(pinMask(sense.pin), GPIO_MODE_INPUT, true))
      return fail("Resistive touch contact GPIO failed");
    delayMicroseconds(80);
    down = gpio_get_level(gpio_num_t(sense.pin)) == 0;
    return true;
  }
  bool contact(bool& down) {
    bool first = false, second = false;
    if (!contactOne(pins_.xp, pins_.xm, senseX_, first) ||
        !contactOne(pins_.yp, pins_.ym, senseY_, second)) return false;
    if (first != second) return fail("Resistive touch contact changed during sample");
    down = first; return true;
  }
  bool axis(int high, int low, const Sense& sense, int32_t& value) {
    if (!neutral() || !level(high, 1) || !level(low, 0) ||
        !mode(pinMask(high) | pinMask(low), GPIO_MODE_OUTPUT) || !configure(sense))
      return fail("Resistive touch axis configuration failed");
    delayMicroseconds(80);
    int result = 0;
    if (!median(sense, result, true)) return false;
    value = result; return true;
  }
};
} // namespace RadioTouch

#endif // GTS_RADIO_TOUCH_RESISTIVE_H
