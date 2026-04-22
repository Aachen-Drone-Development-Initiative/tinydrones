#include "adc.h"

#include "control.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hal/adc_types.h"
#include "sdkconfig.h"
#include "soc/adc_channel.h"

#include <stdbool.h>

// TODO: See if this can be moved to Kconfig
#define ADC_CHANNEL ADC1_CHANNEL_1_GPIO_NUM

#define BATTERY_GUARD CONFIG_BATTERY_GUARD
#define BATTERY_CHECK_PERIOD CONFIG_BATTERY_CHECK_PERIOD
#define BATTERY_MIN CONFIG_BATTERY_MIN

static adc_oneshot_unit_handle_t adc_handle;
static bool is_init = false;

void adc_init() {
  if (is_init) {
    adc_oneshot_del_unit(adc_handle);
  }

  is_init = true;

  adc_oneshot_unit_init_cfg_t unit_init_config = {.clk_src =
                                                      0, // uses default clock
                                                  .unit_id = ADC_UNIT_1,
                                                  .ulp_mode = false};

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_init_config, &adc_handle));

  // TODO: Figure out why ADC_BITWIDTH_12 doesnt work
  // Remeber to change get_batter calculation if changing bitwidth
  adc_oneshot_chan_cfg_t channel_config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_12,
  };

  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &channel_config));
}

float get_battery_voltage() {
  int raw;
  adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw);
  float end_res = (((float)raw) / 4095.0f) * 1.1f * 4.0f * 2; // scale by 2 for voltage divider
  return end_res;
}

void battery_check_task() {
  while (BATTERY_GUARD) {
    vTaskDelay(pdMS_TO_TICKS(BATTERY_CHECK_PERIOD));
    float battery_voltage = get_battery_voltage();
    ESP_LOGI("battery", "Battery Voltage: %f", battery_voltage);
    if (battery_voltage <= BATTERY_MIN) {
      ESP_LOGE("battery", "Voltage Dropped below BATTERY_MIN");
      shutdown();
    }
    vTaskDelete(NULL);
  }
}
