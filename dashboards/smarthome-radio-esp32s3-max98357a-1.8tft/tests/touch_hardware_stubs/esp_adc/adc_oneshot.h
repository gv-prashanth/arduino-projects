#pragma once
#include <driver/gpio.h>
typedef int adc_unit_t;
typedef int adc_channel_t;
enum { ADC_UNIT_1 = 0, ADC_UNIT_2 = 1, ADC_ULP_MODE_DISABLE = 0,
       ADC_BITWIDTH_12 = 12, ADC_ATTEN_DB_12 = 3 };
struct adc_oneshot_unit_init_cfg_t { adc_unit_t unit_id = -1; int ulp_mode = -1; };
struct adc_oneshot_chan_cfg_t { int bitwidth = -1, atten = -1; };
struct FakeAdcHandle { int value; };
typedef FakeAdcHandle* adc_oneshot_unit_handle_t;
inline FakeAdcHandle& fakeAdcHandle() { static FakeAdcHandle value{}; return value; }
inline esp_err_t adc_oneshot_io_to_channel(int pin, adc_unit_t* unit, adc_channel_t* channel) {
  if (pin < 1 || pin > 20) return ESP_FAIL;
  *unit = pin <= 10 ? ADC_UNIT_1 : ADC_UNIT_2;
  *channel = (pin - 1) % 10;
  return ESP_OK;
}
inline esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t* config,
                                    adc_oneshot_unit_handle_t* handle) {
  auto& s = FakeTouch::state();
  ++s.newCalls;
  assert(config->unit_id == ADC_UNIT_1 && config->ulp_mode == ADC_ULP_MODE_DISABLE);
  if (s.denyUnit) return ESP_ERR_NOT_FOUND;
  *handle = &fakeAdcHandle();
  return ESP_OK;
}
inline esp_err_t adc_oneshot_del_unit(adc_oneshot_unit_handle_t handle) {
  assert(handle == &fakeAdcHandle());
  ++FakeTouch::state().delCalls;
  return ESP_OK;
}
inline esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t handle, adc_channel_t channel,
                                          const adc_oneshot_chan_cfg_t* config) {
  auto& s = FakeTouch::state();
  assert(handle == &fakeAdcHandle() && channel >= 0 && channel < 10);
  assert(config->bitwidth == 12 && config->atten == ADC_ATTEN_DB_12);
  if (++s.configureCalls == s.configureFailAt) return ESP_FAIL;
  const int pin = channel + 1;
  FakeTouch::checkPin(pin);
  s.pins[pin].mode = FakeTouch::Analog;
  s.pins[pin].pullup = false;
  s.configuredPins.push_back(pin);
  return ESP_OK;
}
inline esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t handle, adc_channel_t channel, int* value) {
  auto& s = FakeTouch::state();
  assert(handle == &fakeAdcHandle() && channel >= 0 && channel < 10);
  const int pin = channel + 1;
  assert(s.pins[pin].mode == FakeTouch::Analog); // Must remux after digital use.
  s.sampledPins.push_back(pin);
  if (++s.readCalls == s.readFailAt) return ESP_ERR_TIMEOUT;
  *value = 1000 + pin;
  if (!s.samples.empty()) { *value = s.samples.front(); s.samples.pop_front(); }
  return ESP_OK;
}
