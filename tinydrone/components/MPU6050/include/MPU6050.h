#pragma once

#include "driver/i2c_master.h"

// The enum corresponds with the required flag for the GYRO_CONFIG Register
typedef enum MPU6050_Gyro_Range_Type {
    MPU6050_PLUS_MINUS_250_DEG_PER_SECOND  = 0b00000000,
    MPU6050_PLUS_MINUS_500_DEG_PER_SECOND  = 0b00001000,
    MPU6050_PLUS_MINUS_1000_DEG_PER_SECOND = 0b00010000,
    MPU6050_PLUS_MINUS_2000_DEG_PER_SECOND = 0b00011000,
} MPU6050_Gyro_Range_Type;

// The enum corresponds with the required flag for the ACCEL_CONFIG Register
typedef enum MPU6050_Accel_Range_Type {
    MPU6050_PLUS_MINUS_2G  = 0b00000000,
    MPU6050_PLUS_MINUS_4G  = 0b00001000,
    MPU6050_PLUS_MINUS_8G  = 0b00010000,
    MPU6050_PLUS_MINUS_10G = 0b00011000,
} MPU6050_Accel_Range_Type;

typedef struct MPU6050_Handle {
    i2c_master_dev_handle_t i2c_handle;
    MPU6050_Gyro_Range_Type gyro_range;
    MPU6050_Accel_Range_Type accel_range;
} MPU6050_Handle;

MPU6050_Handle mpu6050_init(i2c_master_bus_handle_t master_bus, MPU6050_Gyro_Range_Type, MPU6050_Accel_Range_Type);

typedef struct MPU6050_Data {
    int64_t time_us;
    float omega_x, omega_y, omega_z; // [rad/s]
    float acc_x, acc_y, acc_z;       // [m/s^2]
} MPU6050_Data;

MPU6050_Data mpu6050_get_data(MPU6050_Handle mpu);
