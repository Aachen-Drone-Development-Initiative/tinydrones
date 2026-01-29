#pragma once

#include "espnow_packets.h"

// Call this before any other function in this header.
// Initializes ESPNow and starts the task that handles receiving/sending packets
// to and from the controller.
void espnow_init_and_start_task(uint32_t task_priority, uint32_t task_stack_depth);

// Terminates the task that handles receiving/sending packets, frees
// any allocated memory, resets all values and deinitializes ESPNow.
// TODO: void espnow_deinit();

// Blocks until a controller is connected
void espnow_block_until_controller_connected();

// Works just like printf
bool espnow_send_log_to_controller(const char* format, ...);

// Get the most recent controll commands.
//
// args:
//     ctrl_cmds     - gets written into
//     ticks_to_wait - how long to wait, to get access to the controll commands.
//
bool espnow_get_latest_controll_commands(Payload_Controll_Commands* ctrl_cmds, TickType_t ticks_to_wait);

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
bool espnow_packet_send_to_controller(EPacket* epkt);
