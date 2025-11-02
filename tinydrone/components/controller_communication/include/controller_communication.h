#pragma once

#include "../../../../shared_types/communication_packets.h"

// TODO: Implement this
// Call this before any other function in this header.
// Initializes NVS, WIFI, ESPNow and starts the Task that handles receiving/sending packets
/* void ctrl_comm_init_and_start_task(); */

// TODO: Implement this
// Returns true when some controller connected successfully to the drone.
// The drone should wait for this event, by calling this function in a loop with some delay.
/* bool ctrl_comm_is_connected(int task_priority); */

typedef struct {
    Controll_State_Flags ctrl_state_flags;
    int16_t pitch;
    int16_t yaw;
    int16_t roll;
    int16_t throttle;
} Ctrl_Comm_Commands;

// TODO: Implement this
/* Ctrl_Comm_Commands ctrl_comm_get_latest_ctrl_cmds(); */

// TODO: Implement this
/* bool ctrl_comm_send_log(Log_Msg_Type type, const char* msg, ...); */
