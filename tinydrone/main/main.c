#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// our components
#include "mpu6050.h"
#include "remote_comm.h"
#include "motor_driver.h"

#define DEFAULT_TASK_PRIORITY    5
#define DEFAULT_TASK_STACK_DEPTH 1000

static i2c_master_bus_handle_t  s_i2c_master_bus = NULL;
static MPU6050_Handle           s_mpu = {0};

void espnow_update_parameters_callback(const EPacket_C2D_Parameters* epkt_params)
{
    const Payload_Named_Value* params = epkt_params->parameters;
    int16_t param_count = epkt_params->parameter_count;

    remote_comm_send_log_values(params, param_count);
    
    /* for (int i = 0; i < param_count; ++i) */
    /* { */
    /*     CONTR_LOG_DEBUG("param_name=%.*s", NAMED_VALUE_NAME_LENGTH, params[i].name); */
        
    /*     switch (params[i].dtype) { */
    /*     case DTYPE_INT32: { */
    /*         int32_t value = 0; */
    /*         memcpy(&value, &params[i].data, sizeof(value)); */
    /*         CONTR_LOG_DEBUG("param_value=%d", value); */
    /*     } break; */
            
    /*     case DTYPE_INT64: { */
    /*         int64_t value = 0; */
    /*         memcpy(&value, &params[i].data, sizeof(value)); */
    /*         CONTR_LOG_DEBUG("param_value=%lld", (long long) value); */
    /*     } break; */
            
    /*     case DTYPE_FLOAT32: { */
    /*         float value = 0; */
    /*         memcpy(&value, &params[i].data, sizeof(value)); */
    /*         CONTR_LOG_DEBUG("param_value=%f", value); */
    /*     } break; */

    /*     default: */
    /*         CONTR_LOG_DEBUG("param_value=<unknown dtype %u>", params[i].dtype); */
    /*         break; */
            
    /*     } */
        
    /* } */
}

static void tinydrone_task(void*)
{
    remote_comm_set_update_parameters_callback(espnow_update_parameters_callback);
    remote_comm_block_until_connected();
    
    while (true)
    {
        Payload_Controll_Commands ctrl_cmds;
        remote_comm_get_latest_controll_commands(&ctrl_cmds);

        /* float acc_x = 1.3, acc_y = 432.3, acc_z = 34.2; */

        /* Payload_Named_Value log_values[] = { */
        /*     remote_comm_create_named_value("acc_x", DTYPE_FLOAT32, &acc_x), */
        /*     remote_comm_create_named_value("acc_y", DTYPE_FLOAT32, &acc_y), */
        /*     remote_comm_create_named_value("acc_z", DTYPE_FLOAT32, &acc_z), */
        /* }; */
        
        /* remote_comm_send_log_values(log_values, 3); */
        
        vTaskDelay(pdMS_TO_TICKS(100));
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
    
    Remote_Comm_Config remote_comm_config = {
        .long_range_wifi = false,
        .task_priority   = DEFAULT_TASK_PRIORITY,
        .task_stack_size = DEFAULT_TASK_STACK_DEPTH * 10,
    };
    
    remote_comm_init(&remote_comm_config);

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t ret_init = tinydrone_init(GPIO_NUM_11,  // i2c_sda
                                        GPIO_NUM_10); // i2c_scl

    if (ret_init != ESP_OK) {
        ESP_LOGE("APP_MAIN", "tinydrone_init() failed, halting...\n");
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
        ESP_LOGE("APP_MAIN", "Failed to create tinydrone_task, halting...\n");
        abort();
    }

    // Returning from app_main, deletes the main task (which is now replaced by the 'tinydrone_task')
}
