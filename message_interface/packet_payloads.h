#pragma once

#include "messaging_config.h"

#include <stdint.h>

#define PACKED __attribute__((packed))

enum {
    CTRL_CMD_FLAG_IS_ARMED = 1 << 0,
};

typedef struct {
    int16_t pitch;
    int16_t yaw;
    int16_t roll;
    int16_t throttle;
    uint8_t flags;
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

enum {
    DRONE_INFO_FLAG_IS_CONNETED      = 1 << 0,
    DRONE_INFO_FLAG_IS_RESPONDING    = 1 << 1,
};

typedef struct {
    uint8_t flags;
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED Payload_Drone_Info;

#undef PACKED
