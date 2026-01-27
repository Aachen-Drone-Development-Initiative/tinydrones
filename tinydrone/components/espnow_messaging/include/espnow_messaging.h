#pragma once

#include "espnow_packets.h"

/* Call this before any other function in this header.
   Initializes ESPNow and starts the task that handles receiving/sending packets
   to and from the controller. */

void espnow_init_and_start_task(uint32_t task_priority,
                                uint32_t task_stack_depth,
                                TickType_t task_loop_delay);

/* Terminates the task that handles receiving/sending packets, frees
   any allocated memory, resets all values and deinitializes ESPNow. */

void espnow_deinit();

/* Returns true when a controller connected successfully to the drone.
   The drone should wait for this event, by calling this function in a loop
   with some delay. */

bool espnow_controller_connected();

bool espnow_send_log_to_controller(const char* format, ...); /* works just like printf */

EHeader espnow_header_create(EPacket_Type type, size_t packet_size, EPacket_Flags flags); /* use this to initialize the header of your EPacket */

bool espnow_get_latest_controll_commands(Payload_Controll_Commands* ctrl_cmds);

bool espnow_packet_send_to_controller(EPacket* epkt);
