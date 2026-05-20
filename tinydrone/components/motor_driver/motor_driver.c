#include "motor_driver.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE

#define MOTOR_DRIVER_ARMED_DUTY         CONFIG_MOTOR_DRIVER_ARMED_DUTY_CYCLE // Duty Cycle when Armed but idling
#define MOTOR_DRIVER_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits TODO
#define MOTOR_DRIVER_MAX_DUTY           CONFIG_MOTOR_DRIVER_MAX_DUTY_CYCLE // = 2^13 TODO
#define MOTOR_DRIVER_FREQUENCY          CONFIG_MOTOR_DRIVER_PWM_FREQUENCY // Frequency in Hertz. Set frequency at 4 kHz

struct motor {
    int motor_id;
    ledc_channel_config_t channel_config;
};

const struct motor Motors[4]= {{
        .motor_id = 1,
        .channel_config = {
            .channel    = LEDC_CHANNEL_0,
            .duty       = MOTOR_DRIVER_ARMED_DUTY,
            .gpio_num   = 5, // Motor1 ist Pin 5
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER,
            .flags.output_invert = 0
        },
    }, {
        .motor_id = 2,
        .channel_config = {
            .channel    = LEDC_CHANNEL_1,
            .duty       = MOTOR_DRIVER_ARMED_DUTY,
            .gpio_num   = 6, // Motor2 ist Pin 6
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER,
            .flags.output_invert = 0
        },
    }, {
        .motor_id = 3,
        .channel_config = {
            .channel    = LEDC_CHANNEL_2,
            .duty       = MOTOR_DRIVER_ARMED_DUTY,
            .gpio_num   = 3, // Motor3 ist Pin 3
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER,
            .flags.output_invert = 0
        },
    }, {
        .motor_id = 4,
        .channel_config = {
            .channel    = LEDC_CHANNEL_3,
            .duty       = MOTOR_DRIVER_ARMED_DUTY,
            .gpio_num   = 4, // Motor4 ist Pin 4
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER,
            .flags.output_invert = 0
        },
}};

extern esp_err_t init_motor_driver() {
    // TODO: ledc_find_suitable_duty_resolution(esp_clk_tree_src_get_freq_hz)
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = MOTOR_DRIVER_DUTY_RES,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = MOTOR_DRIVER_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    for (int i = 0; i < 4; i++) {
        err = ledc_channel_config(&(Motors[i].channel_config));
        if (err != ESP_OK) {
            ESP_LOGE("Motor driver", "Failed to configure channel");
            return err;
        }
    }
    return ESP_OK;
}

extern esp_err_t set_motor(int motor_id, float value) {
    if (motor_id < 1 || motor_id > 4) {
        ESP_LOGE("Motor driver", "Invalid motor ID");
        return ESP_ERR_INVALID_ARG;
    }
    if (value < 0 || value > 1.0) {
        ESP_LOGE("Motor driver", "Invalid motor value");
        return ESP_ERR_INVALID_ARG;
    }
    // get pwm_value from float value
    static const int pwm_min = MOTOR_DRIVER_ARMED_DUTY;
    static const int pwm_max = MOTOR_DRIVER_MAX_DUTY;// pow(2, MOTOR_DRIVER_DUTY_RES); // TODO optimize
    const int pwm_value = pwm_min + (int)(value * (pwm_max-pwm_min));
    ESP_LOGI("Motor driver", "Setting motor %d to %d", motor_id, pwm_value);

    esp_err_t err = ledc_set_duty(LEDC_MODE, Motors[motor_id-1].channel_config.channel, pwm_value);
    if (err != ESP_OK) {
        ESP_LOGE("Motor driver", "Failed to set duty");
        return err;
    }
    err = ledc_update_duty(LEDC_MODE, Motors[motor_id-1].channel_config.channel);
    if (err != ESP_OK) {
        ESP_LOGE("Motor driver", "Failed to update duty");
        return err;
    }
    return ESP_OK;
}
