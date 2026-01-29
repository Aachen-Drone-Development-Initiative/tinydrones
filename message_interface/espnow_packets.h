
// TODO: EPKT_D2C_ACCEPT_CONNECT is actually necessary, because the drone can decline in case it is already connected
// to a different controller.

#pragma once

#include "packet_config.h"
#include "packet_payloads.h"

#include <stdint.h>

#define EPKT_MAX_SIZE 1470   /* max packet length for espnow v2, should be equal to ESP_NOW_MAX_DATA_LEN_V2 */

enum {
    EPKT_INVALID = 0,
    
    EPKT_D2C_ADVERTISE = 1,           /* Drone notifies the Controller that it is available */
    EPKT_D2C_ACK_RECEIVE = 4,         /* Drone acknowledges that a specific packet was received */
    EPKT_D2C_LOG = 5,                 /* Drone sends log message to Controller */
    EPKT_D2C_IMU_DATA = 6,            /* Drone sends IMU data to Controller */
    /* EPKT_D2C_TELEMETRY = 7,           /\* Drone sends telemetry data to Controller *\/ */

    EPKT_C2D_CONNECT = 32,            /* Controller asks Drone to connect */
    EPKT_C2D_DISCONNECT = 33,         /* Controller asks Drone to disconnect */
    EPKT_C2D_CONTROLL_COMMANDS = 34,  /* Controller sends controll commands to the Drone */
    EPKT_C2D_ACK_RECEIVE = 35,        /* Controller acknowledges that a specific packet was received */
};
typedef uint8_t EPacket_Type;

enum {
    EPKT_FLAG_REQUIRE_ACK = 1 << 0,
};
typedef uint8_t EPacket_Flags;

typedef uint32_t EPacket_ID;

#define PACKED __attribute__((packed))

typedef struct {
    EPacket_Type type;          /* The packet type */
    EPacket_Flags flags;        /* Flags to enable or disable things (currently just acknowledge) */
    int16_t payload_size;       /* payload_size = packet_size - sizeof(EHeader) */
    EPacket_ID packet_id;       /* A 'unique' randomly generated id */
} PACKED EHeader;

/* Abstract ESP-Now Packet */
/* You can pointer cast between EPacket and any EPacket_* (of course after verifying the type)*/

typedef struct {
    EHeader header;
    uint8_t payload[0];
} PACKED EPacket;

/* D2C - Drone to Controller Packets */

typedef struct {
    EHeader header;             /* type == PKT_D2C_ADVERTISE */
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED EPacket_D2C_Advertise;

typedef struct {
    EHeader header;             /* type == PKT_D2C_ACCEPT_CONNECTION */
} PACKED EPacket_D2C_Accept_Connection;

typedef struct {
    EHeader header;             /* type == PKT_D2C_ACCEPT_DISCONNECT */
} PACKED EPacket_D2C_Accept_Disconnect;

typedef struct {
    EHeader header;             /* type == EPKT_D2C_ACK_RECEIVE */
    EPacket_ID acknowledged_packet_id;
} PACKED EPacket_D2C_Ack_Receive;

typedef struct {
    EHeader header;             /* type == PKT_D2C_LOG */
    char msg[0];                /* msg_length = header.payload_size */
} PACKED EPacket_D2C_Log;

typedef struct {
    EHeader header;             /* type == PKT_D2C_IMU_DATA */
    Payload_Imu_Data imu_data;
} PACKED EPacket_D2C_Imu_Data;

typedef struct {
    EHeader header;             /* type == EPKT_D2C_TELEMETRY */
    Payload_Telemetry telemetry;
} PACKED EPacket_D2C_Telemetry;

/* C2D - Controller to Drone Packets */

typedef struct {
    EHeader header;             /* type == PKT_C2D_CONNECT */
} PACKED EPacket_C2D_Connect;

typedef struct {
    EHeader header;             /* type == PKT_C2D_DISCONNECT */
} PACKED EPacket_C2D_Disconnect;

typedef struct {
    EHeader header;             /* type == PKT_C2D_CONTROLL_COMMANDS */
    Payload_Controll_Commands ctrl_cmds;
} PACKED EPacket_C2D_Controll_Commands;

typedef struct {
    EHeader header;             /* type == EPKT_C2D_ACK_RECEIVE */
    EPacket_ID acknowledged_packet_id;
} PACKED EPacket_C2D_Ack_Receive;

#undef PACKED
