#pragma once

#include "packet_config.h"

#include <stdint.h>

#define PACKED __attribute__((packed))

typedef struct {
    int16_t pitch;
    int16_t yaw;
    int16_t roll;
    int16_t throttle;
    uint8_t is_armed : 1;
} PACKED Payload_Controll_Commands;

typedef struct {
    int64_t time_us;
    float omega_x, omega_y, omega_z; // [rad/s]
    float acc_x, acc_y, acc_z;       // [m/s^2]
} PACKED Payload_Imu_Data;

typedef struct {
    int64_t time_us;
    float battery_voltage;
    // ...
} PACKED Payload_Telemetry;

typedef struct {
    uint8_t connected : 1;
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED Payload_Drone_Info;

#undef PACKED
