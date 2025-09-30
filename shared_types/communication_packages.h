#pragma once

#include <stdint.h>

// NOTE (YUZENI): Julia does not support packed structs, so keeping padding enabled is easier for now
#define PKG_DISABLE_PADDING 0

#if PKG_DISABLE_PADDING
    #define PKG_PADDING __attribute__((packed))
#else
    #define PKG_PADDING
#endif

enum {
    PKG_INVALID = 0,

    // D2C - Drone to Controller
    PKG_D2C_ADVERTISE = 1,
    PKG_D2C_ACCEPT_CONNECTION = 2,
    PKG_D2C_ACCEPT_DISCONNECT = 3,

    // 2U - (Drone/Controller) to User
    PKG_2U_IMU_DATA = 4,

    // 2D - (User/Controller) to Drone
    PKG_2D_CONNECT = 5,
    PKG_2D_DISCONNECT = 6,
    PKG_2D_CONTROLL_COMMANDS = 7,

    // U2C - User to Controller
    PKG_U2C_GET_INDEXED_DRONES = 8,

    // C2U - Controller to User
    PKG_C2U_DRONE_CONNECTED = 9,
    PKG_C2U_DRONE_DISCONNECTED = 10,
    PKG_C2U_INDEXED_DRONE = 11,
    PKG_C2U_INDEXED_DRONES_END = 12,
    
    PKG_TYPE_SIZE, // NOTE (YUZENI): Make sure this has the largest value
};
typedef uint8_t Package_Type;

typedef struct {
    Package_Type type;
    uint8_t payload[0];
} PKG_PADDING Package;

typedef uint32_t Serial_Delimiter;
#define SERIAL_DELIMITER ((Serial_Delimiter) 0x796E6974) // "tiny" in ascii-codes

typedef int ESP_IDX;
#define INVALID_ESP_IDX (-1)

typedef struct {
    Serial_Delimiter delimiter;
    Package pkg;
    uint8_t payload[0];
} PKG_PADDING Serial_Package;

/////////////////////////////////////////////
// Packages D2C (Drone to Controller)
/////////////////////////////////////////////

// type == PKG_D2C_ADVERTISE
typedef struct { Package pkg; } PKG_PADDING Package_D2C_Advertise;

// type == PKG_D2C_ACCEPT_CONNECTION
typedef struct { Package pkg; } PKG_PADDING Package_D2C_Accept_Connection;

// type == PKG_D2C_ACCEPT_DISCONNECT
typedef struct { Package pkg; } PKG_PADDING Package_D2C_Accept_Disconnect;

/////////////////////////////////////////////
// Packages 2D ((User/Controller) to Drone)
/////////////////////////////////////////////

// type == PKG_2D_CONNECT
typedef struct { Package pkg; } PKG_PADDING Package_2D_Connect;

typedef struct {
    Serial_Package spkg;
    ESP_IDX esp_idx;
} PKG_PADDING Serial_Package_2D_Connect;

// type == PKG_2D_DISCONNECT
typedef struct { Package pkg; } PKG_PADDING Package_2D_Disconnect;

typedef struct {
    Serial_Package spkg;
    ESP_IDX esp_idx;
} PKG_PADDING Serial_Package_2D_Disconnect;

enum {
    DRONE_ARMED = (uint16_t) (1 << 0),
};
typedef uint16_t Controll_State_Flags;

typedef struct {
    Controll_State_Flags ctrl_state_flags;
    int16_t pitch;
    int16_t yaw;
    int16_t roll;
    int16_t throttle;
} Payload_Controll_Commands;

// type == PKG_2D_CONTROLL_COMMANDS
typedef struct {
    Package pkg;
    Payload_Controll_Commands ctrl_cmds;
} PKG_PADDING Package_2D_Controll_Commands;

typedef struct {
    Serial_Package spkg;
    Payload_Controll_Commands ctrl_cmds;
    ESP_IDX esp_idx;
} PKG_PADDING Serial_Package_2D_Controll_Commands;

/////////////////////////////////////////////
// Packages U2C (User to Controller)
/////////////////////////////////////////////

// type == PKG_U2C_GET_INDEXED_DRONES
typedef struct {
    Serial_Package spkg;
} PKG_PADDING Serial_Package_U2C_Get_Indexed_Drones;

/////////////////////////////////////////////
// Packages C2U (Controller to User)
/////////////////////////////////////////////

// type == PKG_C2U_DRONE_CONNECTED
typedef struct {
    Serial_Package spkg;
    ESP_IDX esp_idx;
} PKG_PADDING Serial_Package_C2U_Drone_Connected;

// type == PKG_C2U_DRONE_DISCONNECTED
typedef struct {
    Serial_Package spkg;
    ESP_IDX esp_idx;
} PKG_PADDING Serial_Package_C2U_Drone_Disconnected;

// type == PKG_C2U_INDEXED_DRONE
typedef struct {
    Serial_Package spkg;
    ESP_IDX idxed_drone;
} PKG_PADDING Serial_Package_C2U_Indexed_Drone;

// type == PKG_C2U_INDEXED_DRONES_END
typedef struct {
    Serial_Package spkg;
} PKG_PADDING Serial_Package_C2U_Indexed_Drones_End;

/////////////////////////////////////////////
// Packages 2U ((Drone/Controller) to User)
/////////////////////////////////////////////

typedef struct {
    int64_t time_us;
    float omega_x, omega_y, omega_z;
    float acc_x, acc_y, acc_z;
} PKG_PADDING Payload_Imu_Data;

// type == PKG_2U_IMU_DATA
typedef struct {
    Package pkg;
    Payload_Imu_Data imu_data;
} PKG_PADDING Package_2U_Imu_Data;

typedef struct {
    Serial_Package spkg;
    Payload_Imu_Data imu_data;
    ESP_IDX esp_idx;
} PKG_PADDING Serial_Package_2U_Imu_Data;

#undef PKG_PADDING
