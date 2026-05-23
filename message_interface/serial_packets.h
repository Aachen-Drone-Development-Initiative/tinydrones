#pragma once

#include "messaging_config.h"
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
    SPKT_M2C_RESTART_DRONE = 69,          // Master requests to restart a specific Drone
    SPKT_M2C_SET_DRONE_PARAMETERS = 70,
    SPKT_M2C_DRONE_POSE_ESTIMATE = 71,

    SPKT_C2M_SIMPLE_RESPONSE = 96,    // Controller sends a success/fail response for a M2C packet like SPKT_M2C_CONNECT_TO_DRONE
    SPKT_C2M_DRONE_INFOS = 97,        // Controller sends the requested basic infos about available Drones to the Master
    SPKT_C2M_DRONE_LOG_VALUES = 98,
    SPKT_C2M_MSG = 100,
};
typedef uint8_t SPacket_Type;

// This delimiter makes up the first bytes of the serial packet (header)
typedef uint32_t Serial_Delimiter;
#define SERIAL_DELIMITER ((Serial_Delimiter) 0xB62A429F) // randomly generated

typedef uint32_t SPacket_ID;

#define PACKED __attribute__((packed))

typedef struct {
    Serial_Delimiter delim;     // Delimiter is in front of the Serial Packet
    SPacket_Type type;          // The packet type
    int16_t payload_size;       // payload_size = packet_size - sizeof(SHeader)
    SPacket_ID packet_id;
} PACKED SHeader;

// "Base Class" Serial Packet
// You can pointer cast between SPacket and any SPacket_* (after verifying the type)

typedef struct {
    SHeader header;
    uint8_t payload[0];
} PACKED SPacket;


// M2C - Master to Controller Packets


typedef struct {
    SHeader header;             // type == SPKT_M2C_CONNECT_TO_DRONE
    char drone_name[DRONE_NAME_LENGTH];
} PACKED SPacket_M2C_Connect_To_Drone;

typedef struct {
    SHeader header;             // type == SPKT_M2C_DISCONNECT_FROM_DRONE
    char drone_name[DRONE_NAME_LENGTH];
} PACKED SPacket_M2C_Disconnect_From_Drone;

typedef struct {
    SHeader header;             // type == SPKT_M2C_CONTROLL_COMMANDS
    Payload_Controll_Commands ctrl_cmds;
    char drone_name[DRONE_NAME_LENGTH];
} PACKED SPacket_M2C_Controll_Commands;

typedef struct {
    SHeader header;             // type == SPKT_M2C_GET_DRONE_INFOS
} PACKED SPacket_M2C_Get_Drone_Infos;

typedef struct {
    SHeader header;             // type == SPKT_M2C_PING_DRONE
    char drone_name[DRONE_NAME_LENGTH];
} PACKED SPacket_M2C_Ping_Drone;

typedef struct {
    SHeader header;             // type == SPKT_M2C_RESTART_DRONE
    char drone_name[DRONE_NAME_LENGTH];
} PACKED SPacket_M2C_Restart_Drone;

typedef struct {
    SHeader header;             // type == SPKT_M2C_SET_DRONE_PARAMETERS
    char drone_name[DRONE_NAME_LENGTH];
    int16_t parameter_count;
    Payload_Named_Value parameters[0];
} PACKED SPacket_M2C_Set_Drone_Parameters;

typedef struct {
    SHeader header;             // type == SPKT_M2C_DRONE_POSE_ESTIMATE
    Payload_Pose_Estimate pose;
    char drone_name[DRONE_NAME_LENGTH];
} PACKED SPacket_M2C_Drone_Pose_Estimate;


// C2M - Controller to Master Packets


enum {
    RESPONSE_SUCCESS = 0,
    RESPONSE_FAIL = 1,
};

typedef struct {
    SHeader header;             // type == SPKT_C2M_SIMPLE_RESPONSE
    uint8_t response;
    SPacket_ID request_pkt_id;      // id of the packet which this packet responds to
    SPacket_Type request_pkt_type;  // type of the packet which this packet responds to
} PACKED SPacket_C2M_Simple_Response;

typedef struct {
    SHeader header;             // type == SPKT_C2M_DRONE_INFOS
    int16_t drone_infos_count;
    Payload_Drone_Info drone_infos[0];
} PACKED SPacket_C2M_Drone_Infos;

typedef struct {
    SHeader header;             // type == SPKT_C2M_DRONE_LOG_VALUES
    char drone_name[DRONE_NAME_LENGTH];
    int16_t log_value_count;
    Payload_Named_Value log_values[0];
} PACKED SPacket_C2M_Drone_Log_Values;

typedef struct {
    SHeader header;             // type == SPKT_C2M_MSG
    char msg[0];                // msg_length = header.payload_size
} PACKED SPacket_C2M_Msg;


#undef PACKED
