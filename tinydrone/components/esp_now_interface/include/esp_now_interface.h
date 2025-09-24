#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "../../../../shared_types/drone_and_controller_communication_types.h"

typedef struct {
    uint8_t src_mac_addr[6];
    uint8_t des_mac_addr[6];
    uint8_t len;
    uint8_t* data;
} ESPnow_Packet;

typedef struct {
    uint8_t mac_addr[6];
    bool connected;
} ESPnow_Controller_Connection;

/// @brief initialize ESP-NOW
/// Must be called after esp_wifi_init()
/// Creates the receive queue
void esp_now_setup();

/// @brief Advertise Drone to Controller
/// Blocks and sends out drone advertisement until espnow controller accepts.
/// Adds a peer with the broadcast MAC address if not already added.
/// @param connection Struct for new connection
/// @param interval_ms Interval in milliseconds between advertisements
void advertise_drone_espnow(ESPnow_Controller_Connection* connection, int64_t interval_ms);

/// @brief Checks if addr is broadcast address
bool is_broadcast_addr(uint8_t *addr);

/// @brief Abstraction of esp_now_send to packages
/// @param mac_addr 
/// @param package 
void send_package_espnow(uint8_t *mac_addr, Package *package);

/// @brief Parses ESPNOW packets into Packages
/// Mainly for internal component use
/// @param packet Pointer to ESPNOW packet
/// @param package Pointer to Package
void espnow_packet_parse(ESPnow_Packet *packet, Package *package);

/// @brief Converts Packages into ESPNOW packets
/// Mainly for internal component use
/// @param packet Pointer to ESPNOW packet
/// @param package Pointer to Package
// void package_to_espnow_packet(ESPnow_Packet *packet, Package *package);

