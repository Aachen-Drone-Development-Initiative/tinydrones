#pragma once

#include <stdint.h>

#define PACKED __attribute__((packed))

#define MAX_DRONE_NAME_LENGTH (16)

enum {
    CTRL_STATE_DRONE_ARMED = (uint16_t) (1 << 0),
};
typedef uint16_t Controll_State_Flags;

typedef struct {
    Controll_State_Flags ctrl_state_flags;
    int16_t pitch;
    int16_t yaw;
    int16_t roll;
    int16_t throttle;
} PACKED Payload_Controll_Commands;

typedef struct {
    int64_t time_us;
    float omega_x, omega_y, omega_z; // [rad/s]
    float acc_x, acc_y, acc_z;       // [m/s^2]
} PACKED Payload_Imu_Data;

enum {
    LOG_NONE = 0,
    LOG_DBG_INFO = 1,
    LOG_ERROR = 2,
};
typedef uint8_t Log_Type;

typedef struct {
    Log_Type type;
    int msg_length;
    char msg[0];
} PACKED Payload_Log;

#undef PACKED
