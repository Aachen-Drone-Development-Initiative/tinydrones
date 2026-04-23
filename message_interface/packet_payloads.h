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
    int16_t thrust;
    uint8_t flags;
} PACKED Payload_Controll_Commands;

enum {
    DRONE_INFO_FLAG_IS_CONNETED      = 1 << 0,
    DRONE_INFO_FLAG_IS_RESPONDING    = 1 << 1,
};

typedef struct {
    uint8_t flags;
    char drone_name[DRONE_NAME_LENGTH];
} PACKED Payload_Drone_Info;

enum {
    DTYPE_INT32     = 0,
    DTYPE_INT64     = 1,
    DTYPE_FLOAT32   = 2,
};

typedef struct {
    uint8_t dtype;
    char name[NAMED_VALUE_NAME_LENGTH];
    uint64_t data;
} PACKED Payload_Named_Value;


#undef PACKED
