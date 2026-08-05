#include "include/navigation.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "mpu6050.h"

#include "sys/time.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

esp_err_t init_sensors(int i2c_sda, int i2c_scl, MPU6050_Handle* mpu_handle_pointer) {
    ESP_LOGI("init_sensors", "Initializing sensors");

    // init I2C
    i2c_master_bus_config_t i2c_config = {
        .i2c_port = I2C_NUM_0, // use i2c controller 0
        .sda_io_num = i2c_sda,
        .scl_io_num = i2c_scl,
        .glitch_ignore_cnt = 7,            // recommended value from documentation
        // .intr_priority = 2,                // medium priority
        .clk_source = I2C_CLK_SRC_DEFAULT, // default clock source
        .flags.enable_internal_pullup = true,
    };
    static i2c_master_bus_handle_t s_i2c_master_bus = NULL;

    if (i2c_new_master_bus(&i2c_config, &s_i2c_master_bus) != ESP_OK) {
      ESP_LOGE("init_sensors", "Failed to initialize I2C bus");
      return ESP_FAIL;
    }

    // init MPU6050
    if (mpu6050_init(mpu_handle_pointer, s_i2c_master_bus,
                     MPU6050_PLUS_MINUS_500_DEG_PER_SECOND,
                     MPU6050_PLUS_MINUS_2G) != ESP_OK) {
      ESP_LOGE("init_sensors", "Failed to initialize MPU6050");
      return ESP_FAIL;
    }
    return ESP_OK;
}

void sensors_task(void *param) {
  QueueHandle_t current_attitude_queue = *(QueueHandle_t*)param;
  static MPU6050_Handle mpu_handle = {0};

  if (init_sensors(I2C_SDA_PIN, I2C_SCL_PIN, &mpu_handle) == ESP_FAIL) {
    ESP_LOGE(pcTaskGetName(NULL), "Failed to initialize sensors. Deleting task.");
    vTaskDelete(NULL);
  }

  // Initialise the xLastWakeTime variable with the current time.
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static float pitch_deg = 0;
  static float roll_deg = 0;

  while (1) {
    // Wait for the next cycle.
    // Perform action here.
    MPU6050_Data mpu_data = mpu6050_get_data(&mpu_handle);

    float accel_roll_deg = atan((mpu_data.acc_x+mpu_data.acc_y)/sqrt(2) / mpu_data.acc_z) * 180 / M_PI;
    float accel_pitch_deg = atan((-mpu_data.acc_y+mpu_data.acc_x)/sqrt(2) / mpu_data.acc_z) * 180 / M_PI;
    ESP_LOGI(pcTaskGetName(NULL), "accel roll %f", accel_roll_deg);
    ESP_LOGI(pcTaskGetName(NULL), "accel pitch %f", accel_pitch_deg);

    float ratio = 0.99;
    float gyro_pitch_deg =
        pitch_deg - (mpu_data.omega_x+mpu_data.omega_y) / FREQUENCY_IN_HZ * 180 / M_PI;
    float gyro_roll_deg =
        roll_deg - (mpu_data.omega_x-mpu_data.omega_y) / FREQUENCY_IN_HZ * 180 / M_PI;

    ESP_LOGI(pcTaskGetName(NULL), "gyro roll %f", gyro_roll_deg);
    ESP_LOGI(pcTaskGetName(NULL), "gyro pitch %f", gyro_pitch_deg);

    pitch_deg = ratio * gyro_pitch_deg + (1 - ratio) * accel_pitch_deg;
    roll_deg = ratio * gyro_roll_deg + (1 - ratio) * accel_roll_deg;

    ESP_LOGI(pcTaskGetName(NULL), "roll %f", roll_deg);
    ESP_LOGI(pcTaskGetName(NULL), "pitch %f", pitch_deg);

    attitude attitude = {roll_deg, pitch_deg};
    BaseType_t xReturn = xQueueOverwrite(current_attitude_queue, &attitude);
    if (xReturn != pdPASS) {
        ESP_LOGE(pcTaskGetName(NULL), "Failed to overwrite attitude queue");
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / FREQUENCY_IN_HZ));
  }
  vTaskDelete(NULL);
};
