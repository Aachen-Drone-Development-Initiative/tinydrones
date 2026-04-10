#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// our components
#include "mpu6050.h"
#include "espnow_messaging.h"
#include "logging.h"
#include "motor_driver.h"

#define DEFAULT_TASK_PRIORITY    5
#define DEFAULT_TASK_STACK_DEPTH 1000

static i2c_master_bus_handle_t  s_i2c_master_bus = NULL;
static MPU6050_Handle           s_mpu = {0};

static void tinydrone_task(void*)
{
    espnow_block_until_controller_connected();
    
    while (true)
    {
        LOG_DEBUG("main loop");

        LOG_ERROR("controller online=%s", espnow_controller_online() ? "true" : "false");

        Payload_Controll_Commands ctrl_cmds;
        espnow_get_latest_controll_commands(&ctrl_cmds);

        if (ctrl_cmds.flags & CTRL_CMD_FLAG_IS_ARMED) {
            CONTR_LOG_DEBUG("is armed!");
            //...
        }
        else {
            CONTR_LOG_DEBUG("not armed.");
        }

        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelete(NULL); // Delete this task
}

static esp_err_t tinydrone_init(gpio_num_t i2c_sda,
                                gpio_num_t i2c_scl)
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
            assert(nvs_flash_erase() == ESP_OK);
            ret = nvs_flash_init();
        }
        assert(ret == ESP_OK);
    }

    /* // init MPU6050 */
    /* // TODO: Make this also create a task with specifiable priority */
    /* if (mpu6050_init(&s_mpu, s_i2c_master_bus, MPU6050_PLUS_MINUS_500_DEG_PER_SECOND, MPU6050_PLUS_MINUS_2G) != ESP_OK) { */
    /*     // TODO: Do good error reporting */
    /*     return ESP_FAIL; */
    /* } */
    
    // Init espnow messaging
    espnow_init_and_start_task(false, DEFAULT_TASK_PRIORITY, DEFAULT_TASK_STACK_DEPTH * 10);

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t ret_init = tinydrone_init(GPIO_NUM_11,  // i2c_sda
                                        GPIO_NUM_10); // i2c_scl

    if (ret_init != ESP_OK) {
        LOG_ERROR("tinydrone_init() failed, halting...\n");
        abort();
    }

    TaskHandle_t* tinydrone_task_handle = NULL;

    BaseType_t ret_task_create = xTaskCreate(tinydrone_task,
                                             "tinydrone_task",
                                             DEFAULT_TASK_STACK_DEPTH * 5,
                                             NULL,
                                             DEFAULT_TASK_PRIORITY,
                                             tinydrone_task_handle);
    
    if (ret_task_create != pdPASS) {
        LOG_ERROR("Failed to create tinydrone_task, halting...\n");
        abort();
    }

    // Returning from app_main, deletes the main task (which is now replaced by the 'tinydrone_task')
}
