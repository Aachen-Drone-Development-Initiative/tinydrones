#pragma once

#include "common_packet_payloads.h"
#include <stdint.h>

#define PACKED __attribute__((packed))

enum {
    EPKT_INVALID = 0,
    EPKT_LOWER_BOUND = 0,
    
    // C2D - Controller to Drone
    EPKT_D2C_ADVERTISE = 1,
    EPKT_D2C_ACCEPT_CONNECTION = 2,
    EPKT_D2C_ACCEPT_DISCONNECT = 3,
    EPKT_D2C_IMU_DATA = 4,
    EPKT_D2C_LOG = 5,

    // D2C - Drone to Controller
    EPKT_C2D_CONNECT = 6,
    EPKT_C2D_DISCONNECT = 7,
    EPKT_C2D_CONTROLL_COMMANDS = 8,

    // Special Packets
    EPKT_ACKNOWLEDGE_RECEIVE = 9,

    EPKT_UPPER_BOUND = 63,
};
typedef uint8_t EPacket_Type;

#define EPKT_COUNT (EPKT_UPPER_BOUND - EPKT_LOWER_BOUND)
#define EPKT_IDX(x) ((x) - EPKT_LOWER_BOUND)

/* The Packet_ID is a randomly generated identifier, that associates
   any copy of the original packet with the original packet.
   This is usefull, when the same packet is received twice,
   or when sending an ack (acknowledge) packet. */

typedef uint16_t EPacket_ID;

/* The espnow header contains general information about the packet.
   It's type the size of it's payload and a packet-id.
   Because 'Packet = Header + Payload', you can compute the size of
   the packet like this: 'pkt_size = sizeof(Header) + header.payload_size'. */

typedef struct {
    EPacket_Type type;
    EPacket_ID packet_id;
    int16_t payload_size;
} PACKED EHeader;

/* Packet is the 'base class' for all the packets below.
   You can use it, when you want to abstract over all packets. */

typedef struct {
    EHeader header;
    uint8_t payload[0];
} PACKED EPacket;


/* D2C - Drone to Controller Packets... */

// type == PKT_D2C_ADVERTISE
typedef struct {
    EHeader header;
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED EPacket_D2C_Advertise;

// type == PKT_D2C_ACCEPT_CONNECTION
typedef struct { EHeader header; } PACKED EPacket_D2C_Accept_Connection;

// type == PKT_D2C_ACCEPT_DISCONNECT
typedef struct { EHeader header; } PACKED EPacket_D2C_Accept_Disconnect;

// type == PKT_D2C_IMU_DATA
typedef struct {
    EHeader header;
    Payload_Imu_Data imu_data;
} PACKED EPacket_D2C_Imu_Data;

// type == PKT_D2C_LOG
typedef struct {
    EHeader header;
    Payload_Log log;
} PACKED EPacket_D2C_Log;


/* C2D - Controller to Drone Packets... */

// type == PKT_C2D_CONNECT
typedef struct { EHeader header; } PACKED EPacket_C2D_Connect;

// type == PKT_C2D_DISCONNECT
typedef struct { EHeader header; } PACKED EPacket_C2D_Disconnect;

// type == PKT_C2D_CONTROLL_COMMANDS
typedef struct {
    EHeader header;
    Payload_Controll_Commands ctrl_cmds;
} PACKED EPacket_C2D_Controll_Commands;


/* Special Packets... */

// type == EPKT_ACKNOWLEDGE_RECEIVE
typedef struct {
    EHeader header;
} PACKED EPacket_Acknowledge_Receive;

#undef PACKED
