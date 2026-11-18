#pragma once
#if RADIO_DISPLAY != RADIO_DISPLAY_TFT_SHIELD_24
#error "Compact display must not include touch GPIO/ADC drivers"
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
  auto& s = FakeTouch::state();
  assert((config->pin_bit_mask & ~s.allowed) == 0);
  assert(s.pins[s.cs].level == 1); // Display must remain deselected.
  if (++s.gpioCalls == s.gpioFailAt) return ESP_FAIL;
  for (unsigned i = 0; i < 49; ++i) if (config->pin_bit_mask & (uint64_t(1) << i)) {
    FakeTouch::checkPin(i);
    s.pins[i].mode = config->mode;
    s.pins[i].pullup = config->pull_up_en != 0;
  }
  return ESP_OK;
}
inline esp_err_t gpio_set_level(gpio_num_t pin, unsigned level) {
  auto& s = FakeTouch::state();
  FakeTouch::checkPin(pin);
  if (pin != s.cs) assert(s.pins[s.cs].level == 1);
  s.pins[pin].level = level;
  return ESP_OK;
}
inline int gpio_get_level(gpio_num_t pin) {
  auto& s = FakeTouch::state();
  FakeTouch::checkPin(pin);
  assert(pin == s.yp && s.pins[pin].mode == FakeTouch::Input && s.pins[pin].pullup);
  assert(s.pins[s.xp].mode == FakeTouch::Output && s.pins[s.xp].level == 0);
  assert(s.pins[s.xm].mode == FakeTouch::Output && s.pins[s.xm].level == 0);
  assert(s.pins[s.ym].mode == FakeTouch::Input && !s.pins[s.ym].pullup);
  ++s.contactCalls;
  bool contact = true;
  if (!s.contacts.empty()) { contact = s.contacts.front(); s.contacts.pop_front(); }
  return contact ? 0 : 1;
}
