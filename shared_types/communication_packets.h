#pragma once

#include <stdint.h>

#define PACKETS_REMOVE_PADDING (1)

#if PACKETS_REMOVE_PADDING
    #define PKT_PADDING __attribute__((packed))
#else
    #define PKT_PADDING
#endif

// The delimiter sent between packets in a serial-data-stream
typedef uint32_t Serial_Delimiter;
#define SERIAL_DELIMITER ((Serial_Delimiter) 0x796E6974) // "tiny" in ascii-codes

// An esp-idx refers to one of the esp-devices visible to the controller.
typedef int Esp_Idx;
#define INVALID_ESP_IDX (-1)

enum {
    PKT_INVALID = 0,

    // U2C - User to Controller
    PKT_U2C_CONNECT = 1, // TODO: change name
    PKT_U2C_DISCONNECT = 2, // TODO: change name
    PKT_U2C_CONTROLL_COMMANDS = 3,
    PKT_U2C_GET_DETECTED_DRONES = 4,

    // C2U - Controller to User
    PKT_C2U_DRONE_CONNECTED = 5,
    PKT_C2U_DRONE_DISCONNECTED = 6,
    PKT_C2U_DETECTED_DRONES = 7,
    PKT_C2U_IMU_DATA = 8,
    PKT_C2U_LOG_MSG = 9,
    
    // C2D - Controller to Drone
    PKT_D2C_ADVERTISE = 10,
    PKT_D2C_ACCEPT_CONNECTION = 11,
    PKT_D2C_ACCEPT_DISCONNECT = 12,
    PKT_D2C_IMU_DATA = 13,
    // TODO: Implement PKT_D2C_LOG_MSG

    // D2C - Drone to Controller
    PKT_C2D_CONNECT = 14,
    PKT_C2D_DISCONNECT = 15,
    PKT_C2D_CONTROLL_COMMANDS = 16,
    
    PKT_TYPE_UPPER_BOUND = 32,
};
typedef uint8_t Packet_Type;

typedef struct {
    Packet_Type type;
    int16_t payload_size; // The size of the payload can be dynamic: payload_size = sizeof(My_Serial_Packet) - sizeof(Serial_Packet)
    uint8_t payload[0];
} PKT_PADDING Packet;

// U2C - User to Controller

// type == PKT_U2C_CONNECT
typedef struct {
    Packet pkt;
    Esp_Idx esp_idx;
} PKT_PADDING Packet_U2C_Connect;

// type == PKT_U2C_DISCONNECT
typedef struct {
    Packet pkt;
    Esp_Idx esp_idx;
} PKT_PADDING Packet_U2C_Disconnect;

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
} PKT_PADDING Payload_Controll_Commands;

// type == PKT_U2C_CONTROLL_COMMANDS
typedef struct {
    Packet pkt;
    Payload_Controll_Commands ctrl_cmds;
    Esp_Idx esp_idx;
} PKT_PADDING Packet_U2C_Controll_Commands;

// type == PKT_U2C_GET_DETECTED_DRONES
typedef struct {
    Packet pkt;
} PKT_PADDING Packet_U2C_Get_Detected_Drones;

// C2U - Controller to User

// type == PKT_C2U_DRONE_CONNECTED
typedef struct {
    Packet pkt;
    Esp_Idx esp_idx;
} PKT_PADDING Packet_C2U_Drone_Connected;

// type == PKT_C2U_DRONE_DISCONNECTED
typedef struct {
    Packet pkt;
    Esp_Idx esp_idx;
} PKT_PADDING Packet_C2U_Drone_Disconnected;

// type == PKT_C2U_DETECTED_DRONES
typedef struct {
    Packet pkt;
    int detected_drones_length;
    Esp_Idx detected_drones[0];
} PKT_PADDING Packet_C2U_Detected_Drones;

typedef struct {
    int64_t time_us;
    float omega_x, omega_y, omega_z; // [rad/s]
    float acc_x, acc_y, acc_z;       // [m/s^2]
} PKT_PADDING Payload_Imu_Data;

// type == PKT_C2U_IMU_DATA
typedef struct {
    Packet pkt;
    Payload_Imu_Data imu_data;
    Esp_Idx esp_idx;
} PKT_PADDING Packet_C2U_Imu_Data;

enum {
    LOG_MSG_NONE = 0,
    LOG_MSG_DBG_INFO = 1,
    LOG_MSG_ERROR = 2,
};
typedef uint8_t Log_Msg_Type;

// type == PKT_C2U_LOG_MSG
typedef struct {
    Packet pkt;
    Log_Msg_Type log_msg_type;
    int msg_length;
    char msg[0];
} PKT_PADDING Packet_C2U_Log_Msg;

// D2C - Drone to Controller

// type == PKT_D2C_ADVERTISE
typedef struct { Packet pkt; } PKT_PADDING Packet_D2C_Advertise;

// type == PKT_D2C_ACCEPT_CONNECTION
typedef struct { Packet pkt; } PKT_PADDING Packet_D2C_Accept_Connection;

// type == PKT_D2C_ACCEPT_DISCONNECT
typedef struct { Packet pkt; } PKT_PADDING Packet_D2C_Accept_Disconnect;

// type == PKT_D2C_IMU_DATA
typedef struct {
    Packet pkt;
    Payload_Imu_Data imu_data;
} PKT_PADDING Packet_D2C_Imu_Data;

// C2D - Controller to Drone

// type == PKT_C2D_CONNECT
typedef struct { Packet pkt; } PKT_PADDING Packet_C2D_Connect;

// type == PKT_C2D_DISCONNECT
typedef struct { Packet pkt; } PKT_PADDING Packet_C2D_Disconnect;

// type == PKT_C2D_CONTROLL_COMMANDS
typedef struct {
    Packet pkt;
    Payload_Controll_Commands ctrl_cmds;
} PKT_PADDING Packet_C2D_Controll_Commands;

#undef PKT_PADDING
