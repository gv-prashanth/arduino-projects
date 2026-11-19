#ifndef GTS_RADIO_TOUCH_HARDWARE_H
#define GTS_RADIO_TOUCH_HARDWARE_H

#include "TouchConfig.h"
#include "TouchCalibration.h"
#if RADIO_TOUCH_ENABLED
#include <Arduino.h>
#include <SPI.h>
#endif

namespace RadioTouch {
#if RADIO_TOUCH_ENABLED
// Differential 12-bit XPT2046 conversions. The parallel TFT remains under
// TFT_eSPI: this reader never remuxes display pads or claims an ESP32 ADC.
// The UI task owns this SPIClass and every call.
class TouchHardware {
 public:
  struct Pins {
    int cs, sclk, mosi, miso, irq;
    Pins(int chipSelect = RADIO_TOUCH_XPT2046_CS, int clock = RADIO_TOUCH_SPI_SCLK,
         int dataOut = RADIO_TOUCH_SPI_MOSI, int dataIn = RADIO_TOUCH_SPI_MISO,
         int interrupt = RADIO_TOUCH_XPT2046_IRQ)
        : cs(chipSelect), sclk(clock), mosi(dataOut), miso(dataIn), irq(interrupt) {}
  };
  explicit TouchHardware(Pins pins = Pins()) : pins_(pins), bus_(FSPI) {}
  TouchHardware(const TouchHardware&) = delete;
  TouchHardware& operator=(const TouchHardware&) = delete;

  bool begin() {
    if (ready_) return true;
    if (pins_.cs < 0 || pins_.sclk < 0 || pins_.mosi < 0 || pins_.miso < 0)
      return fail("Configure XPT2046 CS, SCLK, MOSI and MISO in TouchConfig.h");
    if (!validPins()) return fail("XPT2046 pins conflict with display, SD, audio or reserved GPIO");
    if (RADIO_TOUCH_SPI_FREQUENCY < 100000 || RADIO_TOUCH_SPI_FREQUENCY > 2000000 ||
        RADIO_TOUCH_PRESSURE_MIN < 1 || RADIO_TOUCH_PRESSURE_MIN > 4095)
      return fail("Invalid XPT2046 SPI frequency or pressure threshold");
    if (!started_) {
      deselect(TFT_CS); deselect(RADIO_SD_CS); deselect(pins_.cs);
      // Pressure polling is authoritative; an interrupt wire is unnecessary.
      if (pins_.irq >= 0) pinMode(pins_.irq, INPUT_PULLUP);
      bus_.begin(pins_.sclk, pins_.miso, pins_.mosi, pins_.cs);
      if (!bus_.bus()) return fail("XPT2046 SPI bus unavailable");
      started_ = true;
    }
    // There is no chip-ID register. Reject obvious absent/shorted responses;
    // a plausible response alone does not establish a board's physical wiring.
    Transaction transaction(*this);
    bool down = false;
    if (!pressure(down)) return false;
    ready_ = true; error_ = "";
    return true;
  }

  // Invalid samples preserve caller outputs; an SPI error must not click the
  // previous contact by fabricating a release.
  bool read(RawPoint& point, bool& pressed) {
    if (!ready_) return false;
    Transaction transaction(*this);
    bool down = false;
    if (!pressure(down)) return false;
    if (!down) { pressed = false; error_ = ""; return true; }
    int xs[3] = {}, ys[3] = {};
    for (unsigned index = 0; index < 3; ++index)
      if (!settled(0xD1, xs[index]) || !settled(0x91, ys[index])) return false;
    const int x = median(xs), y = median(ys);
    if (xs[2] - xs[0] > 160 || ys[2] - ys[0] > 160)
      return fail("XPT2046 sample is unstable");
    if (!pressure(down)) return false;
    if (down) { point.x = x; point.y = y; }
    pressed = down; error_ = "";
    return true;
  }

  uint32_t signature() const {
    uint32_t hash = 2166136261UL;
    const uint32_t words[] = {0x58505431UL, uint32_t(pins_.cs), uint32_t(pins_.sclk),
        uint32_t(pins_.mosi), uint32_t(pins_.miso), uint32_t(pins_.irq),
        uint32_t(RADIO_TOUCH_PRESSURE_MIN), 12, 0};
    for (unsigned index = 0; index < sizeof(words) / sizeof(words[0]); ++index)
      for (unsigned shift = 0; shift < 32; shift += 8) {
        hash ^= (words[index] >> shift) & 255U; hash *= 16777619UL;
      }
    return hash;
  }
  const char* error() const { return error_; }

 private:
  Pins pins_;
  SPIClass bus_;
  bool ready_ = false, started_ = false;
  const char* error_ = "";

  class Transaction {
   public:
    explicit Transaction(TouchHardware& owner) : owner_(owner) {
      digitalWrite(TFT_CS, HIGH); digitalWrite(RADIO_SD_CS, HIGH);
      digitalWrite(owner_.pins_.cs, HIGH);
      owner_.bus_.beginTransaction(SPISettings(RADIO_TOUCH_SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
      digitalWrite(owner_.pins_.cs, LOW);
    }
    ~Transaction() {
      // PD1:PD0=00 powers the ADC down and reenables PENIRQ on every exit.
      owner_.bus_.transfer(0xD0);
      owner_.bus_.transfer16(0);
      digitalWrite(owner_.pins_.cs, HIGH);
      owner_.bus_.endTransaction();
    }
   private:
    TouchHardware& owner_;
  };

  static void deselect(int pin) {
    // Arduino-ESP32 3.x digitalWrite ignores a newly configured pin until
    // pinMode has registered GPIO ownership with Peripheral Manager.
    pinMode(pin, OUTPUT); digitalWrite(pin, HIGH);
  }
  static bool available(int pin) {
    // ESP32-S3-WROOM N16R8: preserve flash/OPI PSRAM, USB, UART, strapping,
    // RGB LED, encoder and I2S. GPIO22..25 do not exist on this target.
    if (!((pin >= 1 && pin <= 21) || (pin >= 38 && pin <= 48))) return false;
    if (pin == 3 || pin == 19 || pin == 20 || pin == 43 || pin == 44 || pin == 45 ||
        pin == 46 || pin == 48 || pin == 1 || pin == 2 || pin == 4 || pin == 5 || pin == 6 || pin == 42)
      return false;
    if (pin == TFT_CS || pin == TFT_DC || pin == TFT_RST || pin == TFT_WR || pin == TFT_RD ||
        pin == TFT_D0 || pin == TFT_D1 || pin == TFT_D2 || pin == TFT_D3 ||
        pin == TFT_D4 || pin == TFT_D5 || pin == TFT_D6 || pin == TFT_D7 || pin == RADIO_SD_CS)
      return false;
    return true;
  }
  bool validPins() const {
    const int pins[] = {pins_.cs, pins_.sclk, pins_.mosi, pins_.miso, pins_.irq};
    for (unsigned index = 0; index < 5; ++index) {
      if (index == 4 && pins[index] == -1) continue;
      if (!available(pins[index])) return false;
      for (unsigned earlier = 0; earlier < index; ++earlier)
        if (pins[index] == pins[earlier]) return false;
    }
    return true;
  }
  bool fail(const char* message) { error_ = message; return false; }
  bool conversion(uint8_t command, int& value) {
    bus_.transfer(command); delayMicroseconds(3);
    const uint16_t word = bus_.transfer16(0);
    // A floating-high MISO has its dummy bit set; do not mask that evidence
    // into a plausible maximum-pressure 12-bit measurement.
    const unsigned raw = word >> 3;
    if (raw > 4095) return fail("XPT2046 invalid SPI response; verify wiring and touch CS");
    value = int(raw); return true;
  }
  bool settled(uint8_t command, int& value) {
    int discarded = 0;
    return conversion(command, discarded) && conversion(command, value);
  }
  bool pressure(bool& down) {
    int z1 = 0, z2 = 0;
    if (!settled(0xB1, z1) || !settled(0xC1, z2)) return false;
    if ((z1 == 0 && z2 == 0) || (z1 == 4095 && z2 == 4095) || z2 + 64 < z1)
      return fail("XPT2046 pressure response invalid; verify SPI pins and touch CS");
    down = z1 + (4095 - z2) >= RADIO_TOUCH_PRESSURE_MIN;
    return true;
  }
  static int median(int (&values)[3]) {
    for (unsigned i = 0; i < 3; ++i)
      for (unsigned j = i + 1; j < 3; ++j)
        if (values[j] < values[i]) { const int temporary = values[i]; values[i] = values[j]; values[j] = temporary; }
    return values[1];
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
