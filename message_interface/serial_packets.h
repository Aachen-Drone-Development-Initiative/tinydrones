#pragma once

#include "packet_config.h"
#include "packet_payloads.h"

#include <stdint.h>

#define SPKT_MAX_SIZE (1024 * 4)

enum {
    SPKT_INVALID = 0,
    
    SPKT_M2C_CONNECT_TO_DRONE = 64,       // Master asks the Controller to connect to the Drone
    SPKT_M2C_DISCONNECT_FROM_DRONE = 65,  // Master asks the Controller to disconnect from the Drone
    SPKT_M2C_CONTROLL_COMMANDS = 66,      // Master sends controll commands for a specific Drone to the Controller
    SPKT_M2C_GET_DRONE_INFOS = 67,        // Master requests basic infos about all available Drones, such as their names
    SPKT_M2C_PING_DRONE = 68,             // Master requests a Ping-Pong response test for a specific Drone

    SPKT_C2M_DRONE_CONNECTION_UPDATE = 96, // Controller notifies the Master about a successful connection to the Drone
    SPKT_C2M_DRONE_INFOS = 97,             // Controller sends the requested basic infos about available Drones to the Master
    SPKT_C2M_DRONE_IMU_DATA = 98,          // Controller sends IMU data of a Drone to the Master
    SPKT_C2M_DRONE_TELEMETRY = 99,         // Controller sends telemetry data of a Drone to the Master
    SPKT_C2M_LOG = 100,
    SPKT_C2M_PING_DRONE_RESULT = 101,      // Controller notifies Master if the Ping-Pong response test was successful or not
};
typedef uint8_t SPacket_Type;

// This delimiter makes up the first bytes of the serial packet header

typedef uint32_t Serial_Delimiter;
#define SERIAL_DELIMITER ((Serial_Delimiter) 0xB62A429F) // randomly generated

#define PACKED __attribute__((packed))

typedef struct {
    Serial_Delimiter delim;     // Delimiter is in front of the Serial Packet
    SPacket_Type type;          // The packet type
    int16_t payload_size;       // payload_size = packet_size - sizeof(SHeader)
} PACKED SHeader;

// "Base Class" Serial Packet
// You can pointer cast between SPacket and any SPacket_* (of course after verifying the type)

typedef struct {
    SHeader header;
    uint8_t payload[0];
} PACKED SPacket;


// M2C - Master to Controller Packets


typedef struct {
    SHeader header;             // type == SPKT_M2C_CONNECT_TO_DRONE
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED SPacket_M2C_Connect_To_Drone;

typedef struct {
    SHeader header;             // type == SPKT_M2C_DISCONNECT_FROM_DRONE
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED SPacket_M2C_Disconnect_From_Drone;

typedef struct {
    SHeader header;             // type == SPKT_M2C_CONTROLL_COMMANDS
    Payload_Controll_Commands ctrl_cmds;
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED SPacket_M2C_Controll_Commands;

typedef struct {
    SHeader header;             // type == SPKT_M2C_GET_DRONE_INFOS
} PACKED SPacket_M2C_Get_Drone_Infos;

typedef struct {
    SHeader header;             // type == SPKT_M2C_PING_DRONE
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED SPacket_M2C_Ping_Drone;


// C2M - Controller to Master Packets


enum {
    DRONE_CONN_UPDATE_SUCCESSFULLY_CONNECTED      = 0,
    DRONE_CONN_UPDATE_FAILED_TO_CONNECT           = 1,
    DRONE_CONN_UPDATE_SUCCESSFULLY_DISCONNECTED   = 2,
    DRONE_CONN_UPDATE_FAILED_TO_DISCONNECT        = 3,
};

typedef struct {
    SHeader header;             // type == SPKT_C2M_DRONE_CONNECTION_UPDATE
    uint8_t update;
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED SPacket_C2M_Drone_Connection_Update;

typedef struct {
    SHeader header;             // type == SPKT_C2M_DRONE_INFOS
    int16_t drone_infos_count;
    Payload_Drone_Info drone_infos[0];
} PACKED SPacket_C2M_Drone_Infos;

typedef struct {
    SHeader header;             // type == SPKT_C2M_IMU_DATA
    Payload_Imu_Data imu_data;
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED SPacket_C2M_Drone_Imu_Data;

typedef struct {
    SHeader header;             // type == SPKT_C2M_DRONE_TELEMETRY
    Payload_Telemetry telemetry;
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED SPacket_C2M_Drone_Telemetry;

typedef struct {
    SHeader header;             // type == SPKT_C2M_LOG
    char msg[0];                // msg_length = header.payload_size
} PACKED SPacket_C2M_Log;

typedef struct {
    SHeader header;             // type == SPKT_C2M_PING_DRONE_RESULT
    uint8_t drone_responded;    // 0 - false, 1 - true
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED SPacket_C2M_Ping_Drone_Result;


#undef PACKED
