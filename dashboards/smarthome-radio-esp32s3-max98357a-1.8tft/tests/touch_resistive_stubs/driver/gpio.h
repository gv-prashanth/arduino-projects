#pragma once
#if !RADIO_TOUCH_ENABLED || RADIO_TOUCH_BACKEND != RADIO_TOUCH_RESISTIVE
#error "Disabled/compact touch must not initialize resistive GPIO/ADC hardware"
#endif
#include <Arduino.h>
typedef int esp_err_t;
constexpr int ESP_OK = 0, ESP_FAIL = -1, ESP_ERR_TIMEOUT = 1, ESP_ERR_NOT_FOUND = 2;
typedef int gpio_num_t;
typedef int gpio_mode_t;
enum { GPIO_MODE_INPUT = 1, GPIO_MODE_OUTPUT = 2, GPIO_PULLUP_DISABLE = 0,
       GPIO_PULLUP_ENABLE = 1, GPIO_PULLDOWN_DISABLE = 0, GPIO_INTR_DISABLE = 0 };
struct gpio_config_t {
  uint64_t pin_bit_mask = 0;
  gpio_mode_t mode = 0;
  int pull_up_en = 0, pull_down_en = 0, intr_type = 0;
};
inline esp_err_t gpio_config(const gpio_config_t* config) {
  auto& s = FakeResistive::state();
  if (++s.gpioCalls == s.gpioFailAt) return ESP_FAIL;
  const bool wr = FakeResistive::digitalLine(7);
  for (int pin = 0; pin < 49; ++pin) if (config->pin_bit_mask & (uint64_t(1) << pin)) {
    FakeResistive::checkPin(pin);
    auto& p = s.pins[pin]; p.mode = config->mode; p.pullup = config->pull_up_en != 0;
  }
  FakeResistive::checkWriteEdge(wr);
  return ESP_OK;
}
inline esp_err_t gpio_set_level(gpio_num_t pin, unsigned value) {
  auto& s = FakeResistive::state(); FakeResistive::checkPin(pin);
  const bool wr = FakeResistive::digitalLine(7);
  s.pins[pin].level = value != 0;
  FakeResistive::checkWriteEdge(wr);
  return ESP_OK;
}
inline int gpio_get_level(gpio_num_t pin) {
  auto& s = FakeResistive::state(); FakeResistive::checkPin(pin);
  assert(s.pins[pin].mode == FakeResistive::Input);
  ++s.contactReads;
  if (s.digitalReadFailAt == s.contactReads) return 1;
  return FakeResistive::voltage(pin) > 0.5 ? 1 : 0;
}
