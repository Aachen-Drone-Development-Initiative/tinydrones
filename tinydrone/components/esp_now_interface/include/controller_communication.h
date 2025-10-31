#pragma once

// Call this before any other function in this header.
// Initializes NVS, WIFI, ESPNow and starts the Task that handles receiving/sending packets
void ctrl_comm_init_and_start_task();

// Returns true when some controller connected successfully to the drone.
// The drone should wait for this event, by calling this in a loop with some delay.
bool is_connected_to_controller();

typedef struct {
    
} Controll_Commands;

Controll_Commands
