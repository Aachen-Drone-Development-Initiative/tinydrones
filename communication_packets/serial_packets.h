#pragma once

#include "packet_payloads.h"

#include <stdint.h>

#define PACKED __attribute__((packed))

enum {
    SPKT_INVALID = 0,
    
    SPKT_M2C_CONNECT_TO_DRONE = 64,       /* Master asks the Controller to connect to the Drone */
    SPKT_M2C_DISCONNECT_FROM_DRONE = 65,  /* Master asks the Controller to disconnect from the Drone */
    SPKT_M2C_CONTROLL_COMMANDS = 66,      /* Master sends controll commands for a specific Drone to the Controller */
    SPKT_M2C_GET_DRONE_INFOS = 67,        /* Master requests basic infos about all available Drones, such as their names */

    SPKT_C2M_DRONE_CONNECTED = 96,        /* Controller notifies the Master about a successful connection to the Drone */
    SPKT_C2M_DRONE_DISCONNECTED = 97,     /* Controller notifies the Master about a successful disconnection from the Drone */
    SPKT_C2M_DRONE_INFOS = 98,            /* Controller sends the requested basic infos about available Drones to the Master */
    SPKT_C2M_DRONE_IMU_DATA = 99,         /* Controller sends IMU data of a Drone to the Master */
    SPKT_C2M_DRONE_TELEMETRY = 100,       /* Controller sends telemetry data of a Drone to the Master */
};
typedef uint8_t SPacket_Type;

/* This delimiter will be in front of all serial packets.
   For example: ...0x796E6974 packet1 0x796E6974 packet2... */

typedef uint32_t Serial_Delimiter;
#define SERIAL_DELIMITER ((Serial_Delimiter) 0x796E6974)
#define SERIAL_DELIMITER_STRING "tiny" /* 0x796E6974 in ascii */

/* This delimiter plus the normal SERIAL_DELIMITER will be in front of log messages.
   For example: ...0x67736D6C796E6974 "some log message" 0x796E6974 packet1... */

typedef uint64_t Serial_Log_Msg_Delimiter;
#define SERIAL_LOG_MSG_DELIMITER ((Serial_Log_Msg_Delimiter) 0x67736D6C796E6974)
#define SERIAL_LOG_MSG_DELIMITER_STRING "tinylsmg" /* 0x67736D6C796E6974 in ascii */

typedef struct {
    Serial_Delimiter delim;     /* Delimiter is in fron of the Serial Packet */
    SPacket_Type type;          /* The packet type */
    int16_t payload_size;       /* payload_size = packet_size - sizeof(SHeader) */
} PACKED SHeader;

/* Abstract Serial Packet */
/* You can pointer cast between SPacket and any SPacket_* (of course after verifying the type)*/

typedef struct {
    SHeader header;
    uint8_t payload[0];
} PACKED SPacket;

/* M2C - Master to Controller Packets */

typedef struct {
    SHeader header;             /* type == SPKT_M2C_CONNECT_TO_DRONE */
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED SPacket_M2C_Connect_To_Drone;

typedef struct {
    SHeader header;             /* type == SPKT_M2C_DISCONNECT_FROM_DRONE */
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED SPacket_M2C_Disconnect_From_Drone;

typedef struct {
    SHeader header;             /* type == SPKT_M2C_CONTROLL_COMMANDS */
    Payload_Controll_Commands ctrl_cmds;
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED SPacket_M2C_Controll_Commands;

typedef struct {
    SHeader header;             /* type == SPKT_M2C_GET_DRONE_INFOS */
} PACKED SPacket_M2C_Get_Drone_Infos;

/* C2M - Controller to Master Packets */

typedef struct {
    SHeader header;             /* type == SPKT_C2M_DRONE_CONNECTED */
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED SPacket_C2M_Drone_Connected;

typedef struct {
    SHeader header;             /* type == SPKT_C2M_DRONE_DISCONNECTED */
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED SPacket_C2M_Drone_Disconnected;

typedef struct {
    SHeader header;             /* type == SPKT_C2M_DRONE_INFOS */
    int16_t drone_infos_count;
    Payload_Drone_Info drone_infos[0];
} PACKED SPacket_C2M_Drone_Infos;

typedef struct {
    SHeader header;             /* type == SPKT_C2M_IMU_DATA */
    Payload_Imu_Data imu_data;
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED SPacket_C2M_Drone_Imu_Data;

typedef struct {
    SHeader header;             /* type == SPKT_C2M_DRONE_TELEMETRY */
    Payload_Imu_Data imu_data;
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED SPacket_C2M_Drone_Telemetry;

#undef PACKED
