#include "controller_communication.h"
#include "communication_packets.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#define RECEIVE_CALLBACK_QUEUE_SIZE (10)
#define SEND_CALLBACK_QUEUE_SIZE (10)

// Special mac address for broadcasting to all devices in range, not just a single device
static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint8_t s_zero_mac[ESP_NOW_ETH_ALEN]      = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

void controller_communication_task(void*)
{
    while (1)
    {
        process_incoming_espnow_packet();

        send_all_queued_espnow_packets();
    }
}
