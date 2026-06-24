#include "navigation.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "mpu6050.h"
#include "fusion_math.h"

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

//! WTF
// void get_euler_from_q (attitude *attitude, float q0, float q1, float q2, float q3) const { 

void get_euler_from_q (attitude *attitude, float q0, float q1, float q2, float q3) {
    attitude->roll_deg = asinf(-2.0f * (q1 * q3 - q0 * q2)) * 180.0f / (float)M_PI;
    attitude->pitch_deg = atan2f(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 180.0f / (float)M_PI;
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

  float beta = 0.1f;

  float q0 = 1.0f; // w
  float q1 = 0.0f; // x
  float q2 = 0.0f; // y
  float q3 = 0.0f; // z


  while (1) {
    // Wait for the next cycle.
    // Perform action here.
    MPU6050_Data mpu_data = mpu6050_get_data(&mpu_handle);

    float ax = mpu_data.acc_x;
    float ay = mpu_data.acc_y;
    float az = mpu_data.acc_z;
    // float gx = mpu_data.gyr_x;
    // float gy = mpu_data.gyr_y;
    // float gz = mpu_data.gyr_z;
    float gx = mpu_data.omega_x;
    float gy = mpu_data.omega_y;
    float gz = mpu_data.omega_z;



    float recipNorm;
    
    float qDot1, qDot2, qDot3, qDot4;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer
    // normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      float s0, s1, s2, s3;
      float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

      // Normalise accelerometer measurement
      // recipNorm = fast_inv_sqrt(ax * ax + ay * ay + az * az);
      //! v !
      // recipNorm = 1.0f / FusionVectorNorm(v);
      recipNorm = 1.0f / (ax * ax + ay * ay + az * az);

      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Auxiliary variables to avoid repeated arithmetic
      _2q0 = 2.0f * q0;
      _2q1 = 2.0f * q1;
      _2q2 = 2.0f * q2;
      _2q3 = 2.0f * q3;
      _4q0 = 4.0f * q0;
      _4q1 = 4.0f * q1;
      _4q2 = 4.0f * q2;
      _8q1 = 8.0f * q1;
      _8q2 = 8.0f * q2;
      q0q0 = q0 * q0;
      q1q1 = q1 * q1;
      q2q2 = q2 * q2;
      q3q3 = q3 * q3;

      // Gradient decent algorithm corrective step
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 +
           _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 +
           _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
      // recipNorm = fast_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
      //! correct v 
      recipNorm = 1.0f / (s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;

      // Apply feedback step
      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    float dt = 1/FREQUENCY_IN_HZ; //? this ?
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    // recipNorm = fast_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    //! correct v 
    recipNorm = 1.0f / (q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
  



    //* Gravitation
   // float accel_roll_deg = atan(mpu_data.acc_x / mpu_data.acc_z) * 180 / M_PI;
    //float accel_pitch_deg = atan(-mpu_data.acc_y / mpu_data.acc_z) * 180 / M_PI;

    // TODO: implement Madgwick Filter
    // TODO: 1) parameter : accel_roll_deg, accel_pitch_deg, mpu_data.omega_x,  mpu_data.omega_y
    // TODO: 2) output : roll_deg, pitch_deg passing ( yaw_deg ?) into attitude
    //? ONLY 2-DIM ?

    attitude attitude = {roll_deg, pitch_deg};
    get_euler_from_q(&attitude, q0, q1, q2, q3);  

    BaseType_t xReturn = xQueueOverwrite(current_attitude_queue, &attitude);
    if (xReturn != pdPASS) {
        ESP_LOGE(pcTaskGetName(NULL), "Failed to overwrite attitude queue");
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / FREQUENCY_IN_HZ));
  }
  vTaskDelete(NULL);
};
