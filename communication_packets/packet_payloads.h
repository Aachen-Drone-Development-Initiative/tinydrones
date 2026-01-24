#pragma once

#include <stdint.h>

#define PACKED __attribute__((packed))

#define MAX_DRONE_NAME_LENGTH (12)
#define MAX_PAYLOAD_SIZE () // TODO: this is not yet respected anywhere, probably put this in packet creation

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

enum {
    PKT_LOG_NONE = 0,
    PKT_LOG_DEBUG = 1,
    PKT_LOG_ERROR = 2,
};
typedef uint8_t Pkt_Log_Type;

typedef struct {
    Pkt_Log_Type type;
    int msg_length;
    char msg[0];
} PACKED Payload_Log;

typedef struct {
    uint8_t connected : 1;
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED Payload_Drone_Info;

#undef PACKED
