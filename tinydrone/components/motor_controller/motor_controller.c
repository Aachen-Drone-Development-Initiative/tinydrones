#include "motor_controller.h"
#include <stdio.h>
#include <math.h>
#include "driver/ledc.h"
#include "esp_err.h"


#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_00       (5) // Define the output GPIO
#define LEDC_CHANNEL_00         LEDC_CHANNEL_0
#define LEDC_OUTPUT_IO_01       (6) // Define the output GPIO
#define LEDC_CHANNEL_01         LEDC_CHANNEL_1
#define LEDC_OUTPUT_IO_02       (3) // Define the output GPIO
#define LEDC_CHANNEL_02         LEDC_CHANNEL_2
#define LEDC_OUTPUT_IO_03       (4) // Define the output GPIO
#define LEDC_CHANNEL_03         LEDC_CHANNEL_3

/*
Motor1 ist Pin 5
Motor2 ist Pin 6
Motor3 ist Pin 3
Motor4 ist Pin 4
*/

#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (400) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz
#define LEDC_TEST_CH_NUM        (4)


static struct motor
{   
    int motor_id;
    ledc_channel_config_t channel_config;
};


struct motor Motors[4]={
{
    .motor_id = 1
},
{
    .motor_id = 2
},
{
    .motor_id = 3
},
{
    .motor_id = 4
},
};


void init_motors(void)
{
    int ch;
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {

    {
        .channel    = LEDC_CHANNEL_00,
        .duty       = 0,
        .gpio_num   = LEDC_OUTPUT_IO_00,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER,
        .flags.output_invert = 0
    },
    {
        .channel    = LEDC_CHANNEL_01,
        .duty       = 0,
        .gpio_num   = LEDC_OUTPUT_IO_01,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER,
        .flags.output_invert = 0
    },
    {
        .channel    = LEDC_CHANNEL_02,
        .duty       = 0,
        .gpio_num   = LEDC_OUTPUT_IO_02,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER,
        .flags.output_invert = 0
    },
    {
        .channel    = LEDC_CHANNEL_03,
        .duty       = 0,
        .gpio_num   = LEDC_OUTPUT_IO_03,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER,
        .flags.output_invert = 0
    },  
    };
    // update motors
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        Motors[ch].channel_config = ledc_channel[ch];
    }

    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[ch]));
    }
    
}

void set_motor(int motor_id, float value)
{
    
    //get pwm_value from float value
    int pwm_min = 0;
    int pwm_max = pow(2,LEDC_DUTY_RES);
    int pwm_vaule = pwm_min + (int)(value * (pwm_max-pwm_min));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, Motors[motor_id-1].channel_config.channel, pwm_vaule));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, Motors[motor_id-1].channel_config.channel));
}
