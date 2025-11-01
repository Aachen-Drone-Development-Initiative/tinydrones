#pragma once

#include <stdint.h>

#define PACKETS_DISABLE_PADDING (1)

#if PACKETS_DISABLE_PADDING
    #define PKT_PADDING __attribute__((packed))
#else
    #define PKT_PADDING
#endif

// The delimiter sent between packets in a serial-data-stream
typedef uint32_t Serial_Delimiter;
#define SERIAL_DELIMITER ((Serial_Delimiter) 0x796E6974) // "tiny" in ascii-codes

// An esp-idx refers to one of the esp-devices visible to the controller.
typedef int ESP_IDX;
#define INVALID_ESP_IDX (-1)

// U2C - User to Controller
// C2U - Controller to User
// C2D - Controller to Drone
// D2C - Drone to Controller

enum {
    PKT_INVALID = 0,

    PKT_D2C_ADVERTISE = 1,
    
    PKT_U2C_CONNECT = 2,
    PKT_C2D_CONNECT = 3,
    PKT_D2C_ACCEPT_CONNECTION = 4,
    PKT_C2U_DRONE_CONNECTED = 5,
    
    PKT_U2C_DISCONNECT = 6,
    PKT_C2D_DISCONNECT = 7,
    PKT_D2C_ACCEPT_DISCONNECT = 8,
    PKT_C2U_DRONE_DISCONNECTED = 9,
    
    PKT_U2C_CONTROLL_COMMANDS = 10,
    PKT_C2D_CONTROLL_COMMANDS = 11,

    PKT_U2C_GET_DETECTED_DRONES = 12,
    PKT_C2U_DETECTED_DRONES = 13,

    PKT_D2C_IMU_DATA = 14,
    PKT_C2U_IMU_DATA = 15,
    
    PKT_C2U_LOG_MSG = 16,
    // TODO: Implement PKT_D2C_LOG_MSG
    
    PKT_TYPE_SIZE, // NOTE (YUZENI): Make sure this has the largest value
};
typedef uint8_t Packet_Type;

typedef struct {
    Packet_Type type;
    int16_t payload_size; // The size of the payload can be dynamic: payload_size = sizeof(My_Serial_Packet) - sizeof(Serial_Packet)
    uint8_t payload[0];
} PKT_PADDING Packet;

// type == PKT_D2C_ADVERTISE
typedef struct { Packet pkt; } PKT_PADDING Packet_D2C_Advertise;

// type == PKT_U2C_CONNECT
typedef struct {
    Packet pkt;
    ESP_IDX esp_idx;
} PKT_PADDING Packet_U2C_Connect;

// type == PKT_C2D_CONNECT
typedef struct { Packet pkt; } PKT_PADDING Packet_C2D_Connect;

// type == PKT_D2C_ACCEPT_CONNECTION
typedef struct { Packet pkt; } PKT_PADDING Packet_D2C_Accept_Connection;

// type == PKT_C2U_DRONE_CONNECTED
typedef struct {
    Packet pkt;
    ESP_IDX esp_idx;
} PKT_PADDING Packet_C2U_Drone_Connected;
    
// type == PKT_U2C_DISCONNECT
typedef struct {
    Packet pkt;
    ESP_IDX esp_idx;
} PKT_PADDING Packet_U2C_Disconnect;

// type == PKT_C2D_DISCONNECT
typedef struct { Packet pkt; } PKT_PADDING Packet_C2D_Disconnect;

// type == PKT_D2C_ACCEPT_DISCONNECT
typedef struct { Packet pkt; } PKT_PADDING Packet_D2C_Accept_Disconnect;

// type == PKT_C2U_DRONE_DISCONNECTED
typedef struct {
    Packet pkt;
    ESP_IDX esp_idx;
} PKT_PADDING Packet_C2U_Drone_Disconnected;
    
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
} PKT_PADDING Payload_Controll_Commands;

// type == PKT_U2C_CONTROLL_COMMANDS
typedef struct {
    Packet pkt;
    Payload_Controll_Commands ctrl_cmds;
    ESP_IDX esp_idx;
} PKT_PADDING Packet_U2C_Controll_Commands;

// type == PKT_C2D_CONTROLL_COMMANDS
typedef struct {
    Packet pkt;
    Payload_Controll_Commands ctrl_cmds;
} PKT_PADDING Packet_C2D_Controll_Commands;

// type == PKT_U2C_GET_DETECTED_DRONES
typedef struct {
    Packet pkt;
} PKT_PADDING Packet_U2C_Get_Detected_Drones;

// type == PKT_C2U_DETECTED_DRONES
typedef struct {
    Packet pkt;
    int detected_drones_length;
    ESP_IDX detected_drones[0];
} PKT_PADDING Packet_C2U_Detected_Drones;

typedef struct {
    int64_t time_us;
    float omega_x, omega_y, omega_z; // [rad/s]
    float acc_x, acc_y, acc_z;       // [m/s^2]
} PKT_PADDING Payload_Imu_Data;

// type == PKT_D2C_IMU_DATA
typedef struct {
    Packet pkt;
    Payload_Imu_Data imu_data;
} PKT_PADDING Packet_D2C_Imu_Data;

// type == PKT_C2U_IMU_DATA
typedef struct {
    Packet pkt;
    Payload_Imu_Data imu_data;
    ESP_IDX esp_idx;
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

#undef PKT_PADDING
