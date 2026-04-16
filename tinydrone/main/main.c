#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>

// our components
#include "controller_communication.h"
#include "motor_driver.h"
#include "control.h"
#include "mpu6050.h"
#include "navigation.h"
#include "mqtt_setup.h"

#define DEFAULT_TASK_PRIORITY (5)
#define DEFAULT_TASK_STACK_DEPTH (10000)

#define TINYDRONE_TASK_PRIORITY (DEFAULT_TASK_PRIORITY)
#define NAVIGATION_TASK_PRIORITY (DEFAULT_TASK_PRIORITY)
#define TINYDRONE_TASK_STACK_DEPTH (DEFAULT_TASK_STACK_DEPTH)

#define CORE0 0
// only define xCoreID CORE1 as 1 if this is a multiple core processor target,
// else define it as tskNO_AFFINITY
#define CORE1 ((CONFIG_FREERTOS_NUMBER_OF_CORES > 1) ? 1 : tskNO_AFFINITY)

static QueueHandle_t current_attitude_queue;

static void tinydrone_task(void *) {
    const float frequency_in_hz = 10.0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    int time = 0;
    while (true) {
        // estimate orientation using IMU-component
        attitude attitude;
        int ticks_to_wait = pdMS_TO_TICKS(1000 / frequency_in_hz);
        if (current_attitude_queue != NULL) {
            BaseType_t xReturn = xQueueReceive(current_attitude_queue,
                                                (void *)&attitude, ticks_to_wait);

            ESP_LOGI("tinydrone_task", "gyro roll %f", attitude.roll_deg);
            ESP_LOGI("tinydrone_task", "gyro pitch %f", attitude.pitch_deg);
        } else {
            ESP_LOGE("tinydrone_task", "Attitude queue not initialized");
        }

        // determine motor signals
        float u_pitch = pid_control_pitch(attitude.pitch_deg/360.0*2*M_PI, 0.0, 0.6, 0.5, 0.12);
        float u_roll = pid_control_roll(attitude.roll_deg/360.0*M_PI, 0.0, 0.6, 0.5, 0.12);
        ESP_LOGI("tinydrone_task", "pitch control %f", u_pitch);
        ESP_LOGI("tinydrone_task", "roll control %f", u_roll);
        Control_Inputs control_inputs = {0.5, u_roll, u_pitch, 0.0};
        Motor_Values motor_values = control_inputs_to_motor_values(control_inputs);
        motor_values = clip_motor_values(motor_values);
        ESP_ERROR_CHECK(set_motor(1, motor_values.motor_value_1));
        ESP_ERROR_CHECK(set_motor(2, motor_values.motor_value_2));
        ESP_ERROR_CHECK(set_motor(3, motor_values.motor_value_3));
        ESP_ERROR_CHECK(set_motor(4, motor_values.motor_value_4));

        // // drive motors using
        // // silly test do not merge this to another branch
        // float motor_signal = 0.5+0.5*sin(time *2*M_PI / 100.0);
        // float motor_signal1 = 0.5+0.5*cos(time *2*M_PI / 100.0);
        // ESP_ERROR_CHECK(set_motor(1, motor_signal/2));
        // ESP_ERROR_CHECK(set_motor(2, motor_signal/10));
        // ESP_ERROR_CHECK(set_motor(3, motor_signal1/2));
        // ESP_ERROR_CHECK(set_motor(4, motor_signal1/10));
        // ESP_LOGI("tinydrone", "Motor signals: %d, %f", 1, motor_signal);
        // time++;

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / frequency_in_hz));
    }
    vTaskDelete(NULL); // Delete this task
}


void app_main(void) {
    // Not implemented yet
    /* while (!ctrl_comm_is_connected()) { */
    /*     vTaskDelay(100 / portTICK_PERIOD_MS); */
    /*     printf("Waiting for connection with controller...\n"); */
    /* } */


    // current_attitude_queue = xQueueCreate(1, sizeof(attitude));
    // ESP_ERROR_CHECK(init_motor_driver());

    // // create tinydrone_task
    // TaskHandle_t *tinydrone_task_handle = NULL;
    // BaseType_t ret_td_task_create = xTaskCreatePinnedToCore(
    //     tinydrone_task, "tinydrone_task", TINYDRONE_TASK_STACK_DEPTH, NULL,
    //     TINYDRONE_TASK_PRIORITY, tinydrone_task_handle, CORE0);
    // if (ret_td_task_create != pdPASS) {
    //     printf("ERROR: Failed to create task tinydrone_task...\n");
    // }

    // // create navigation_task
    // TaskHandle_t *navigation_task_handle = NULL;
    // BaseType_t ret_nav_task_create = xTaskCreatePinnedToCore(
    //     sensors_task, "navigation_task", DEFAULT_TASK_STACK_DEPTH,
    //     &current_attitude_queue, NAVIGATION_TASK_PRIORITY, navigation_task_handle,
    //     CORE0);
    // if (ret_nav_task_create != pdPASS) {
    //     printf("ERROR: Failed to create task navigation_task...\n");
    // }


    // Returning from app_main, deletes the main task (which is now replaced by the 'tinydrone_task')


    // MQTT broker initialization
    // ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(wifi_setup());
}
