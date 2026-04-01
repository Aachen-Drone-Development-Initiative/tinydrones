#pragma once

#include "espnow_packets.h"
#include "freertos/FreeRTOS.h"
#include "stdbool.h"

// Call this before any other function in this header.
// Initializes ESPNow and starts the task that handles receiving/sending packets
// to and from the controller.
void espnow_init_and_start_task(bool long_range_wifi, uint32_t task_priority, uint32_t task_stack_size);

// Terminates the task that handles receiving/sending packets, frees
// any allocated memory, resets all values and deinitializes ESPNow.
void espnow_deinit();

// Blocks until a controller is connected
void espnow_block_until_controller_connected();

// Get the most recent controll commands.
Payload_Controll_Commands espnow_get_latest_controll_commands();

// Use this to initialize the header of your EPacket
EHeader espnow_header_create(EPacket_Type type, size_t packet_size, EPacket_Flags flags);

// Use this to send espnow packets to the controller
//
// example use:
//     EPacket_D2C_Telemetry epkt;
//     epkt.header = espnow_header_create(EPKT_D2C_TELEMETRY, sizeof(EPacket_D2C_Telemetry), 0);
//     epkt.telemetry = ...;
//     espnow_packet_send_to_controller((EPacket*) &epkt);
//
bool espnow_send_packet_to_controller(EPacket* epkt);

// Works just like printf
bool espnow_send_msg_to_controller(const char* format, ...);

