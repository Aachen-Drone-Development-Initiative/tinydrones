#pragma once

#include "espnow_packets.h"
#include "freertos/FreeRTOS.h"
#include "stdbool.h"

typedef enum {
    ESPNOW_MSG_OK = 0,
    ESPNOW_MSG_ERR_CONTROLLER_DISCONNECTED,
    ESPNOW_MSG_ERR_CONTROLLER_OFFLINE,
    ESPNOW_MSG_ERR_ESP_NOW_SEND_FAIL,
} Espnow_Messaging_Ret;

// Call this before any other function in this header.
// Initializes ESPNow and starts the task that handles receiving/sending packets
// to and from the controller.
void espnow_init_and_start_task(bool long_range_wifi, uint32_t task_priority, uint32_t task_stack_size);

// Terminates the task that handles receiving/sending packets, frees
// any allocated memory, resets all values and deinitializes ESPNow.
void espnow_deinit();

// Blocks until a controller is connected.
void espnow_block_until_controller_connected();

// Get the most recent controll commands. Always works, even if controller is disconnected or offline.
void espnow_get_latest_controll_commands(Payload_Controll_Commands* out);

// Returns true if the drone is connected to a controller.
bool espnow_controller_connected();

// Returns true if the controller is 'online', meaning at least one packet from the
// controller was received within the last 'MIN_C2D_SEND_INTERVAL_MS', if this is
// not the case we can assume that the controller is out of reach or powered off.
bool espnow_controller_online();

// Use this to initialize the header of your EPacket. See below for an example.
EHeader espnow_header_create(EPacket_Type type, size_t packet_size, EPacket_Flags flags);

// Use this to send espnow packets to the controller
//
// example use:
//     EPacket_D2C_Telemetry epkt;
//     epkt.header = espnow_header_create(EPKT_D2C_TELEMETRY, sizeof(EPacket_D2C_Telemetry), 0);
//     epkt.telemetry = ...;
//     espnow_send_packet_to_controller((EPacket*) &epkt);
//
Espnow_Messaging_Ret espnow_send_packet_to_controller(EPacket* epkt);

// Works just like printf, sends the resulting string to the controller.
Espnow_Messaging_Ret espnow_send_msg_to_controller(const char* format, ...);

