#pragma once

#include "esp_task.h"
#include "mpu6050.h"
#include <stdio.h>

#define FREQUENCY_IN_HZ 1000

#define I2C_SDA_PIN GPIO_NUM_11
#define I2C_SCL_PIN GPIO_NUM_10

void sensors_task(void *tinydrone_task_handle);

typedef struct {
  float roll_deg;
  float pitch_deg;
} attitude;
