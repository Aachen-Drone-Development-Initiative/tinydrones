#include "adc.h"

#include "control.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hal/adc_types.h"
#include "sdkconfig.h"
#include "soc/adc_channel.h"

#include <stdbool.h>
#include <stdint.h>

// TODO: See if this can be moved to Kconfig
#define ADC_CHANNEL ADC_CHANNEL_1 // Should be GPIO2 according to schematic

#define BATTERY_GUARD CONFIG_BATTERY_GUARD
#define BATTERY_CHECK_PERIOD CONFIG_BATTERY_CHECK_PERIOD
#define BATTERY_MIN ((float) CONFIG_BATTERY_MIN) / 1000

static adc_oneshot_unit_handle_t adc_handle;
static bool is_init = false;
static adc_cali_handle_t handle;
static uint8_t strikes = 0;

static void createAdcCalibration(adc_oneshot_chan_cfg_t * channel_config, adc_oneshot_unit_init_cfg_t *unit_config) {
  adc_cali_line_fitting_config_t config = {
    .atten  = channel_config->atten,
    .bitwidth = channel_config->bitwidth,
    .unit_id = unit_config->unit_id
  };

  ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&config, &handle));

}


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

  // The Bitwidth might cause problems on other esp chips. For esp32s2, bitwidth is fixed 13 bit.
  // Remeber to change get_batter calculation if changing bitwidth
  adc_oneshot_chan_cfg_t channel_config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_13,
  };

  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &channel_config));

  createAdcCalibration(&channel_config, &unit_init_config);
}

float get_battery_voltage() {
  int raw;
  adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw);
  
  int battery_voltage_mv;
  adc_cali_raw_to_voltage(handle, raw, &battery_voltage_mv);

  return (((float) battery_voltage_mv ) / 1000.0f) * 2; // scale by 2 cause of voltage divider
}


void battery_check_task() {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(BATTERY_CHECK_PERIOD));
    float battery_voltage = get_battery_voltage();
    ESP_LOGD("battery", "Battery Voltage: %f", battery_voltage);
    if (battery_voltage <= BATTERY_MIN) {
      if (strikes >= 3) {
        ESP_LOGE("battery", "Voltage Dropped below BATTERY_MIN! Shutting Down.");
        if (BATTERY_GUARD) {
          shutdown();
        } else {
          ESP_LOGD("battery", "Drone would shutdown but BATTERY_GUARD isnt set");
        }
      } else {
        ESP_LOGW("battery", "Battery voltage measured low: %fV", battery_voltage);
        strikes++;
      }
    } else {
      strikes = 0;
    }
  }
}

  
