#include "adc.h"

#include "control.h"

#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/adc_channel.h"

#define ADC_CHANNEL ADC_BITWIDTH_12

#define BATTERY_MIN 3.1f
#define BATTERY_GUARD 1

adc_oneshot_unit_handle_t adc_handle;

void adc_init() {
    adc_oneshot_unit_init_cfg_t unit_init_config = {
        .unit_id = ADC_UNIT_1,
    };

    adc_oneshot_new_unit(&unit_init_config, &adc_handle);

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &channel_config);
}

float get_voltage() {
        int raw;
        adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw);
        float end_res = (((float) raw) / 4095.0f) * 1.1f * 4.0f * 1.66f; //last like 1.5 to 1.9
        return end_res;
}

void battery_check_task() {
    while (BATTERY_GUARD) {
        if (get_voltage() <= BATTERY_MIN)
            shutdown();
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
