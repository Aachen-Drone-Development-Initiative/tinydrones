#include "adc.h"

#include "control.h"

#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "soc/adc_channel.h"

#include <stdbool.h>

#define ADC_CHANNEL ADC1_GPIO35_CHANNEL

#define BATTERY_GUARD 1 //if battery voltage should be checked
#define BATTERY_MIN 3.1f //minimum voltage (in V)
#define CONTROL_GAP 5000 //time between the task checking the battery again (in ms)

adc_oneshot_unit_handle_t adc_handle;
bool is_init;

void adc_init() {
    if (is_init) {
        adc_oneshot_del_unit(adc_handle);
    }

    is_init = true;

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
        vTaskDelay(pdMS_TO_TICKS(CONTROL_GAP));
        if (get_voltage() <= BATTERY_MIN)
            shutdown();
    }
}
