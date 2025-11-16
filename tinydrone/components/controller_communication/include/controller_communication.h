#pragma once

#include "communication_packets.h"

/* Call this before any other function in this header.
   Initializes ESPNow and starts the task that handles receiving/sending packets
   to and from the controller. */

void ctrl_comm_init_and_start_task(uint32_t task_priority,
                                   uint32_t task_stack_depth,
                                   TickType_t task_loop_delay);

/* Terminates the task that handles receiving/sending packets, frees
   any allocated memory, resets all values and deinitializes ESPNow. */

void ctrl_comm_deinit();

/* Returns true when some controller connected successfully to the drone.
   The drone should wait for this event, by calling this function in a loop
   with some delay. */

bool ctrl_comm_is_connected();

typedef struct {
    Controll_State_Flags ctrl_state_flags;
    int16_t pitch;
    int16_t yaw;
    int16_t roll;
    int16_t throttle;
} Ctrl_Comm_Commands;

// TODO: Implement this
/* Ctrl_Comm_Commands ctrl_comm_get_latest_ctrl_cmds(); */

/* Send a log (message) to the controller. This function works
   exaclty like 'printf()'. So you can use it like this:
   'ctrl_comm_send_log(LOG_ERROR, "Expected %d but got %d, 42, value);' */

void ctrl_comm_send_log(Log_Msg_Type type, const char* format, ...);
