#pragma once

#include "common_packet_payloads.h"
#include <stdint.h>

#define PACKED __attribute__((packed))

enum {
    SPKT_INVALID = 0,
    SPKT_LOWER_BOUND = 63,

    // M2C - Master to Controller
    SPKT_M2C_CONNECT_TO_DRONE = 64,
    SPKT_M2C_DISCONNECT_FROM_DRONE = 65,
    SPKT_M2C_CONTROLL_COMMANDS = 66,
    SPKT_M2C_GET_DETECTED_DRONES = 67,

    // C2M - Controller to Master
    SPKT_C2M_DRONE_CONNECTED = 68,
    SPKT_C2M_DRONE_DISCONNECTED = 69,
    SPKT_C2M_DETECTED_DRONES = 70,
    SPKT_C2M_IMU_DATA = 71,
    SPKT_C2M_LOG = 72,

    SPKT_UPPER_BOUND = 127,
};
typedef uint8_t SPacket_Type;

#define SPKT_COUNT (SPKT_UPPER_BOUND - SPKT_LOWER_BOUND)
#define SPKT_IDX(x) ((x) - SPKT_LOWER_BOUND)

/* The delimiter sent between packets in a serial-data-stream.
   It's always sent before the packet. */

typedef uint32_t Serial_Delimiter;
#define SERIAL_DELIMITER ((Serial_Delimiter) 0x796E6974) // "tiny" in ascii-codes

/* The serial header contains general information about the packet.
   It's type and the size of it's payload. It is also prefixed with a delimiter.
   The delimiter is required, because the the first bytes in the serial buffer
   are not necessarily the beginning of a packet, although it's very likely.
   Because 'Packet = Header + Payload', you can compute the size of
   the packet like this: 'pkt_size = sizeof(Header) + header.payload_size'. */

typedef struct {
    Serial_Delimiter delim;
    SPacket_Type type;
    int16_t payload_size;
} PACKED SHeader;

/* SPacket is the 'base class' for all the packets below.
   You can use it, when you want to abstract over all packets. */

typedef struct {
    SHeader header;
    uint8_t payload[0];
} PACKED SPacket;

/* The Esp_Idx is used for refering to a specific drone, that is
   visible to the controller. */

typedef int Esp_Idx;
#define INVALID_ESP_IDX (-1)


/* M2C - Master to Controller Packets... */

// type == PKT_M2C_CONNECT_TO_DRONE
typedef struct {
    SHeader header;
    Esp_Idx esp_idx;
} PACKED SPacket_M2C_Connect_To_Drone;

// type == PKT_M2C_DISCONNECT_FROM_DRONE
typedef struct {
    SHeader header;
    Esp_Idx esp_idx;
} PACKED SPacket_M2C_Disconnect_From_Drone;

// type == PKT_M2C_CONTROLL_COMMANDS
typedef struct {
    SHeader header;
    Payload_Controll_Commands ctrl_cmds;
    Esp_Idx esp_idx;
} PACKED SPacket_M2C_Controll_Commands;

// type == PKT_M2C_GET_DETECTED_DRONES
typedef struct {
    SHeader header;
} PACKED SPacket_M2C_Get_Detected_Drones;


/* C2M - Controller to Master Packets... */

// type == PKT_C2M_DRONE_CONNECTED
typedef struct {
    SHeader header;
    Esp_Idx esp_idx;
} PACKED SPacket_C2M_Drone_Connected;

// type == PKT_C2M_DRONE_DISCONNECTED
typedef struct {
    SHeader header;
    Esp_Idx esp_idx;
} PACKED SPacket_C2M_Drone_Disconnected;

typedef struct {
    Esp_Idx esp_idx;
    char drone_name[MAX_DRONE_NAME_LENGTH];
} PACKED Payload_Detected_Drone;

// type == PKT_C2M_DETECTED_DRONES
typedef struct {
    SHeader header;
    int detected_drones_length;
    Payload_Detected_Drone detected_drones[0];
} PACKED SPacket_C2M_Detected_Drones;

// type == PKT_C2M_IMU_DATA
typedef struct {
    SHeader header;
    Payload_Imu_Data imu_data;
    Esp_Idx esp_idx;
} PACKED SPacket_C2M_Imu_Data;

// type == PKT_C2M_LOG
typedef struct {
    SHeader header;
    Esp_Idx esp_idx;
    Payload_Log log;
} PACKED SPacket_C2M_Log;

#undef PACKED
