#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

typedef struct {
    uint8_t src_mac_addr[6];
    uint8_t des_mac_addr[6];
    uint8_t payload[ESP_NOW_MAX_DATA_LEN_V2];
    size_t len;
} ESPnow_Packet_t;

typedef struct {
    uint8_t mac_addr[6];
    bool connected;
} Dongle_Connection_t;

/// @brief Enable drone advertisement
/// Blocks and sends out drone advertisement until dongle accepts.
/// Adds a peer with the broadcast MAC address if not already added.
void advertise_drone(Dongle_Connection_t* connection, int64_t interval_ms);

/// @brief Accept a dongle connection
/// @param connection 
void accept_dongle_connection(Dongle_Connection_t* connection);

/// @brief initialize ESP-NOW
//void esp_now_init();

