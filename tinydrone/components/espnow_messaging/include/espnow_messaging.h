#pragma once

#include "espnow_packets.h"
#include "freertos/FreeRTOS.h"
#include "stdint.h"
#include "stdbool.h"

typedef enum {
    ESPNOW_MSG_OK = 0,
    ESPNOW_MSG_ERR_ARG,
    ESPNOW_MSG_ERR_CONTROLLER_DISCONNECTED,
    ESPNOW_MSG_ERR_CONTROLLER_OFFLINE,
    ESPNOW_MSG_ERR_ESP_NOW_SEND_FAIL,
    ESPNOW_MSG_ERR_PACKET_TOO_LARGE,
} Espnow_Messaging_Ret;

typedef struct {
    bool long_range_wifi;
    uint32_t task_priority;
    uint32_t task_stack_size;
} Espnow_Messaging_Config;

// Call this before any other function in this header.
// Initializes ESPNow and starts the task that handles receiving/sending packets
// to and from the controller.
void espnow_init(const Espnow_Messaging_Config* config);

// Terminates the task that handles receiving/sending packets, frees
// any allocated memory, resets all values and deinitializes ESPNow.
void espnow_deinit();

// Blocks until a controller is connected.
void espnow_block_until_controller_connected();

// Returns true if the drone is connected to a controller.
bool espnow_controller_connected();

// Returns true if the controller is 'online', meaning at least one packet from the
// controller was received within the last 'MIN_C2D_SEND_INTERVAL_MS', if this is
// not the case we can assume that the controller is out of reach or powered off.
bool espnow_controller_online();

// Get the most recent controll commands. Always works, even if controller is disconnected or offline.
void espnow_get_latest_controll_commands(Payload_Controll_Commands* out);

typedef void (*espnow_update_parameters_callback_t)(const EPacket_C2D_Parameters* epkt_params);

// Try to make the callback fast, because since its called from the main espnow_messaging task,
// it will pause espnow communication.
void espnow_set_espnow_update_parameters_callback(espnow_update_parameters_callback_t callback);

// Helper function, inteded to be used in the 'update_parameters_callback'.
// It is the callers responsibility that the types of the parameter and of 'value_out' match.
bool espnow_expect_parameter(const EPacket_C2D_Parameters* epkt_params, const char* name, void* value_out);

// Use this to initialize the header of your EPacket. See below for an example.
EHeader espnow_header_create(EPacket_Type type, size_t packet_size, EPacket_Flags flags);

// Use this to send espnow packets to the controller
Espnow_Messaging_Ret espnow_send_packet_to_controller(EPacket* epkt);

// Works just like printf, sends the resulting string to the controller.
Espnow_Messaging_Ret espnow_send_msg_to_controller(const char* format, ...);

// Helper function, intended to be used to fill an array of named values for 'espnow_send_log_values_to_controller'
Payload_Named_Value espnow_create_named_value(const char* name, uint8_t dtype, void* value_in);

// Send the log values as a single packet to the controller.
// 'log_value_count' must be <= EPKT_MAX_LOG_VALUES_PER_PACKET.
Espnow_Messaging_Ret espnow_send_log_values_to_controller(const Payload_Named_Value* log_values, int log_value_count);
