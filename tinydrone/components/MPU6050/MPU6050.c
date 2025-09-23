#include "MPU6050.h"

#include "sys/time.h"
#include <math.h>

#define REG_PWR_MGMT_1        0x6B
#define REG_SMPLRT_DIV        0x19
#define REG_GYRO_CONFIG       0x1B
#define REG_ACCEL_CONFIG      0x1C
#define REG_GYRO_XOUT_H       0x43
#define REG_ACCEL_XOUT_H      0x3B

#define G_ACC 9.80665f

static bool mpu6050_set_register(MPU6050_Handle mpu, uint8_t register_address, uint8_t value)
{
    uint8_t buffer[] = {register_address, value};
    if (i2c_master_transmit(mpu.i2c_handle, buffer, sizeof(buffer), -1) != ESP_OK) return false;
    return true;
}

// Starts reading at 'begin_register_address' 
static bool mpu6050_read_registers(MPU6050_Handle mpu, uint8_t begin_register_address, uint8_t *buffer, size_t length)
{
    // First transmit the starting address, then receive the bytes
    if (i2c_master_transmit_receive(mpu.i2c_handle, &begin_register_address, 1, buffer, length, -1) != ESP_OK) return false;
    return true;
}

MPU6050_Handle mpu6050_init(i2c_master_bus_handle_t master_bus, MPU6050_Gyro_Range_Type gyro_range, MPU6050_Accel_Range_Type accel_range)
{
    i2c_device_config_t i2c_device_config = {
        .dev_addr_length = I2C_ADDR_BIT_7, // 7-bit address
        .device_address = 0x68, // I2C address of the device
        .scl_speed_hz = 100000, // I2C clock speed
    };

    // probe the device to check if it's connected
    if (i2c_master_probe(master_bus, 0x68, 100) != ESP_OK) {
        printf("Error: Didn't find an i2c device let alone mpu6050 at adress 0x68\n");
        return (MPU6050_Handle){0};
    }

    MPU6050_Handle mpu = {0};
    mpu.gyro_range = gyro_range;
    mpu.accel_range = accel_range;
    if (i2c_master_bus_add_device(master_bus, &i2c_device_config, &mpu.i2c_handle) != ESP_OK) {
        printf("Error: Failed to add mpu6050 as i2c device\n");
        return (MPU6050_Handle){0};
    }

    if (!mpu6050_set_register(mpu, REG_PWR_MGMT_1, 0x00)) return (MPU6050_Handle){0}; // wake up the MPU6050 by writing 0 to PWR_MGMT_1 register

    // specify sample rate
    // sample rate = (gyro output rate - 8 Khz) / (value at REG_SMPLRT_DIV + 1)
    // REG_SMPLRT_DIV == 0x00 => max sample rate
    // WARNING: The output rate of the accelerometer is just 1 Khz, so if we query at 8 Khz, accel values will repeat
    if (!mpu6050_set_register(mpu, REG_SMPLRT_DIV, 0x00)) return (MPU6050_Handle){0};

    // specify range
    if (!mpu6050_set_register(mpu, REG_GYRO_CONFIG, (uint8_t) gyro_range)) return (MPU6050_Handle){0};
    if (!mpu6050_set_register(mpu, REG_ACCEL_CONFIG, (uint8_t) accel_range)) return (MPU6050_Handle){0};

    return mpu;
}

MPU6050_Data mpu6050_get_data(MPU6050_Handle mpu)
{
    uint8_t raw_gyro_buffer[6];
    uint8_t raw_accel_buffer[6];

    // Read all accelerometer and gyro output registers.
    // They are contiguous in memory like this: ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L
    // And this: GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L
    if (!mpu6050_read_registers(mpu, REG_GYRO_XOUT_H, raw_gyro_buffer, sizeof(raw_gyro_buffer))) return (MPU6050_Data){0};
    if (!mpu6050_read_registers(mpu, REG_ACCEL_XOUT_H, raw_accel_buffer, sizeof(raw_accel_buffer))) return (MPU6050_Data){0};

    // get the current time
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);

    // Combine high/low bits: Converting big-endian value [high, low] to little-endian [low, high]
    int16_t raw_gyro_x = ((uint16_t) raw_gyro_buffer[0] << 8) | (uint16_t) raw_gyro_buffer[1];
    int16_t raw_gyro_y = ((uint16_t) raw_gyro_buffer[2] << 8) | (uint16_t) raw_gyro_buffer[3];
    int16_t raw_gyro_z = ((uint16_t) raw_gyro_buffer[4] << 8) | (uint16_t) raw_gyro_buffer[5];
    int16_t raw_acc_x = ((uint16_t) raw_accel_buffer[0] << 8) | (uint16_t) raw_accel_buffer[1];
    int16_t raw_acc_y = ((uint16_t) raw_accel_buffer[2] << 8) | (uint16_t) raw_accel_buffer[3];
    int16_t raw_acc_z = ((uint16_t) raw_accel_buffer[4] << 8) | (uint16_t) raw_accel_buffer[5];

    float gyro_range_radps = 0; // [rad/s]
    switch (mpu.gyro_range) {
    case MPU6050_PLUS_MINUS_250_DEG_PER_SECOND: gyro_range_radps = 250.f/360.0 * 2.0*M_PI; break;
    case MPU6050_PLUS_MINUS_500_DEG_PER_SECOND: gyro_range_radps = 500.f/360.0 * 2.0*M_PI; break;
    case MPU6050_PLUS_MINUS_1000_DEG_PER_SECOND: gyro_range_radps = 1000.f/360.0 * 2.0*M_PI; break;
    case MPU6050_PLUS_MINUS_2000_DEG_PER_SECOND: gyro_range_radps = 2000.f/360.0 * 2.0*M_PI; break;
    }
    
    float accel_range_mpss = 0; // [m/s^2]
    switch (mpu.accel_range) {
    case MPU6050_PLUS_MINUS_2G: accel_range_mpss = 2.f * G_ACC; break;
    case MPU6050_PLUS_MINUS_4G: accel_range_mpss = 4.f * G_ACC; break;
    case MPU6050_PLUS_MINUS_8G: accel_range_mpss = 8.f * G_ACC; break;
    case MPU6050_PLUS_MINUS_10G: accel_range_mpss = 10.f * G_ACC; break;
    }
    
    MPU6050_Data data = {
        .time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec,
        .omega_x = ((float) raw_gyro_x / 32768.f) * gyro_range_radps, // 32768 = 2^15
        .omega_y = ((float) raw_gyro_y / 32768.f) * gyro_range_radps,
        .omega_z = ((float) raw_gyro_z / 32768.f) * gyro_range_radps,
        .acc_x = ((float) raw_acc_x / 32768.f) * accel_range_mpss,
        .acc_y = ((float) raw_acc_y / 32768.f) * accel_range_mpss,
        .acc_z = ((float) raw_acc_z / 32768.f) * accel_range_mpss,
    };

    return data;
}
