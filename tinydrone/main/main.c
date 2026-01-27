#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>

// our components
#include "mpu6050.h"
#include "controller_communication.h"
#include "motor_driver.h"

#define DEFAULT_TASK_PRIORITY    (5)
#define DEFAULT_TASK_STACK_DEPTH (200)

#define TINYDRONE_TASK_PRIORITY    (DEFAULT_TASK_PRIORITY)
#define TINYDRONE_TASK_STACK_DEPTH (DEFAULT_TASK_STACK_DEPTH * 5)

static i2c_master_bus_handle_t  s_i2c_master_bus = NULL;
static MPU6050_Handle           s_mpu = {0};

static void tinydrone_task(void*)
{
    while (true) {

        // estimate orientation using IMU-component
    
        // determine new motor signals using ori-estimate and ctrl-comm-component
    
        // drive motors using
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL); // Delete this task
}

static esp_err_t tinydrone_init(gpio_num_t i2c_sda,
                                gpio_num_t i2c_scl,
                                bool enable_long_range_wifi)
{
    // init I2C
    {
        i2c_master_bus_config_t i2c_config = {
            .i2c_port = I2C_NUM_0,                 // use i2c controller 0
            .sda_io_num = i2c_sda,
            .scl_io_num = i2c_scl,
            .glitch_ignore_cnt = 7,                // recommended value from documentation
            .intr_priority = 2,                    // medium priority
            .clk_source = I2C_CLK_SRC_DEFAULT,     // default clock source
            .flags.enable_internal_pullup = true,
        };

        if (i2c_new_master_bus(&i2c_config, &s_i2c_master_bus) != ESP_OK) {
            // TODO: Do good error reporting
            return ESP_FAIL;
        }
    }

    // init NVS (non volatile storage) (required by ESPNow)
    {    
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            if (nvs_flash_erase() != ESP_OK) DGL_ABORT("init_nvs: nvs_flash_erase failed");
            ret = nvs_flash_init();
        }
        if(ret != ESP_OK) DGL_ABORT("init_nvs: nvs_flash_init failed");
    }

    // init MPU6050
    {
        // TODO: Make this also create a task with specifiable priority
        if (mpu6050_init(&s_mpu, s_i2c_master_bus, MPU6050_PLUS_MINUS_500_DEG_PER_SECOND, MPU6050_PLUS_MINUS_2G) != ESP_OK) {
            // TODO: Do good error reporting
            return ESP_FAIL;
        }
    }

    // Init wifi
    {
        if (esp_netif_init() != ESP_OK)                DGL_ABORT("init_wifi: esp_netif_init failed");
        if (esp_event_loop_create_default() != ESP_OK) DGL_ABORT("init_wifi: esp_event_loop_create_default failed");
    
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    
        if (esp_wifi_init(&cfg) != ESP_OK)                 DGL_ABORT("init_wifi: esp_wifi_init failed");
        if (esp_wifi_set_mode(ESPNOW_WIFI_MODE) != ESP_OK) DGL_ABORT("init_wifi: esp_wifi_set_mode failed");
        if (esp_wifi_start() != ESP_OK)                    DGL_ABORT("init_wifi: esp_wifi_start failed");

        if (enable_long_range_wifi) {
            if (esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) != ESP_OK) {
                DGL_ABORT("init_wifi: esp_wifi_set_protocol failed");
            }
        }
    }
    
    // Init espnow messaging
    espnow_init_and_start_task(DEFAULT_TASK_PRIORITY, DEFAULT_TASK_STACK_DEPTH);

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t ret_init = tinydrone_init(GPIO_NUM_11,  // i2c_sda
                                        GPIO_NUM_10); // i2c_scl

    if (ret_init != ESP_OK) {
        printf("ERROR: tinydrone_init() failed, halting...\n");
        while (true); // halts everything
    }

    // Not implemented yet
    /* while (!ctrl_comm_is_connected()) { */
    /*     vTaskDelay(100 / portTICK_PERIOD_MS); */
    /*     printf("Waiting for connection with controller...\n"); */
    /* } */

    TaskHandle_t* tinydrone_task_handle = NULL;

    BaseType_t ret_task_create = xTaskCreate(tinydrone_task,
                                             "tinydrone_task",
                                             TINYDRONE_TASK_STACK_DEPTH,
                                             NULL,
                                             TINYDRONE_TASK_PRIORITY,
                                             tinydrone_task_handle);
    
    if (ret_task_create != pdPASS) {
        printf("ERROR: Failed to create task tinydrone_task, halting...\n");
        while (true); // halts everything
    }

    // Returning from app_main, deletes the main task (which is now replaced by the 'tinydrone_task')
}
