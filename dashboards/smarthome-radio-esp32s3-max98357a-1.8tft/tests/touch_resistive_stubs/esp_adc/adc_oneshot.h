#pragma once
#include <driver/gpio.h>
typedef int adc_unit_t;
typedef int adc_channel_t;
enum { ADC_UNIT_1 = 0, ADC_UNIT_2 = 1, ADC_ULP_MODE_DISABLE = 0,
       ADC_BITWIDTH_12 = 12, ADC_ATTEN_DB_12 = 3 };
struct adc_oneshot_unit_init_cfg_t { adc_unit_t unit_id = -1; int ulp_mode = -1; };
struct adc_oneshot_chan_cfg_t { int bitwidth = -1, atten = -1; };
struct FakeAdcHandle { int unit; };
typedef FakeAdcHandle* adc_oneshot_unit_handle_t;
inline FakeAdcHandle& fakeAdcHandle(int unit) { static FakeAdcHandle handles[2] = {{0}, {1}}; return handles[unit]; }
inline esp_err_t adc_oneshot_io_to_channel(int pin, adc_unit_t* unit, adc_channel_t* channel) {
  if (pin < 1 || pin > 20) return ESP_FAIL;
  *unit = pin <= 10 ? ADC_UNIT_1 : ADC_UNIT_2; *channel = (pin - 1) % 10; return ESP_OK;
}
inline esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t* config, adc_oneshot_unit_handle_t* handle) {
  auto& s = FakeResistive::state();
  assert(config->unit_id >= 0 && config->unit_id < 2 && config->ulp_mode == ADC_ULP_MODE_DISABLE);
  ++s.newCalls[config->unit_id];
  if (s.denyUnit[config->unit_id]) return ESP_ERR_NOT_FOUND;
  *handle = &fakeAdcHandle(config->unit_id); return ESP_OK;
}
inline esp_err_t adc_oneshot_del_unit(adc_oneshot_unit_handle_t handle) {
  assert(handle && handle == &fakeAdcHandle(handle->unit)); ++FakeResistive::state().delCalls[handle->unit]; return ESP_OK;
}
inline esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t handle, adc_channel_t channel,
                                          const adc_oneshot_chan_cfg_t* config) {
  auto& s = FakeResistive::state();
  assert(handle && channel >= 0 && channel < 10);
  assert(config->bitwidth == 12 && config->atten == ADC_ATTEN_DB_12);
  if (++s.configureCalls == s.configureFailAt) return ESP_FAIL;
  const int pin = handle->unit * 10 + channel + 1; FakeResistive::checkPin(pin);
  // Like gpio_config, analog setup owns pad muxing. A digital restore means
  // the next axis group must configure its ADC pad again even if cached.
  s.pins[pin].mode = FakeResistive::Analog; s.pins[pin].pullup = false;
  s.configuredPins.push_back(pin); return ESP_OK;
}
inline esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t handle, adc_channel_t channel, int* value) {
  auto& s = FakeResistive::state();
  assert(handle && channel >= 0 && channel < 10);
  const int pin = handle->unit * 10 + channel + 1;
  assert(s.pins[pin].mode == FakeResistive::Analog);
  s.sampledPins.push_back(pin);
  if (++s.readCalls == s.readFailAt) return ESP_ERR_TIMEOUT;
  if (s.liftAtAdcRead && s.readCalls >= s.liftAtAdcRead) s.down = false;
  if (!s.samples.empty()) { *value = s.samples.front(); s.samples.pop_front(); return ESP_OK; }
  *value = int(std::lround(FakeResistive::voltage(pin) * 4095.0));
  if (*value > 4095) *value = 4095;
  if (s.noiseAmplitude) *value += (s.readCalls % 2 ? -s.noiseAmplitude : s.noiseAmplitude);
  return ESP_OK;
}
