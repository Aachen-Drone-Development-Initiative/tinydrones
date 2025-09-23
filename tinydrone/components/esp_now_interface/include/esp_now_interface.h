#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

static QueueHandle_t esp_now_recv_queue; // Queue for incoming packets

typedef struct {
    uint8_t src_mac_addr[6];
    uint8_t des_mac_addr[6];
    uint8_t payload[ESP_NOW_MAX_DATA_LEN_V2];
    size_t len;
} ESPnow_Packet;

typedef struct {
    uint8_t mac_addr[6];
    bool connected;
} Dongle_Connection;

/// @brief initialize ESP-NOW
/// Must be called after esp_wifi_init()
/// Creates the receive queue
void esp_now_setup();

/// @brief Enable drone advertisement
/// Blocks and sends out drone advertisement until dongle accepts.
/// Adds a peer with the broadcast MAC address if not already added.
/// @param interval_ms Interval in milliseconds between advertisements
Dongle_Connection advertise_drone(int64_t interval_ms);

/// @brief Checks if addr is broadcast address
bool is_broadcast_addr(uint8_t *addr);

