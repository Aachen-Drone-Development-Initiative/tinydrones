#include "esp_now_interface.h"

#include "../../shared_types/communication_packets.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#define BROADCAST_MAC_ADDR (uint8_t[]){0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
#define ESP_NOW_QUEUE_LENGTH 10
#define DEFAULT_ESPNOW_CHANNEL 0

static QueueHandle_t s_esp_now_recv_queue; // Queue for espnow packets

typedef struct {
    uint8_t src_mac_addr[6];
    uint8_t des_mac_addr[6];
    uint8_t len;
    uint8_t* data;
} ESPnow_Packet;

void espnow_recv_cb(const esp_now_recv_info_t *recieved_packet, const uint8_t *data, int data_len) {
    ESPnow_Packet espnow_packet;
    ESP_LOGI("espnow", "Received espnow_packet from %02X:%02X:%02X:%02X:%02X:%02X, length: %d", 
        recieved_packet->src_addr[0], recieved_packet->src_addr[1], recieved_packet->src_addr[2], 
        recieved_packet->src_addr[3], recieved_packet->src_addr[4], recieved_packet->src_addr[5],
        data_len);

    espnow_packet.data = malloc(data_len);

    espnow_packet.len = data_len;
    memcpy(espnow_packet.src_mac_addr, recieved_packet->src_addr, 6);
    memcpy(espnow_packet.des_mac_addr, recieved_packet->des_addr, 6);
    memcpy(espnow_packet.data, data, data_len);

    xQueueSend(s_esp_now_recv_queue, &espnow_packet, 0);
}

bool is_broadcast_addr(uint8_t* addr) {
    return (memcmp(addr, BROADCAST_MAC_ADDR, ESP_NOW_ETH_ALEN) == 0);
}

void esp_now_setup() {
    ESP_ERROR_CHECK(esp_now_init());
    s_esp_now_recv_queue = xQueueCreate(ESP_NOW_QUEUE_LENGTH, sizeof(ESPnow_Packet));
}

void send_package_espnow(uint8_t *mac_addr, Package *package) {
    // convert package to espnow data
    int16_t size;
    switch (package->type) {
        case PKG_D2C_ADVERTISE:
            size = sizeof(Package_Request_Connection);
            break;
        case PKG_D2C_IMU_DATA:
            size = sizeof(Package_Imu_Data);
            break;
        case PKG_C2D_ACCEPT_ADVERTISEMENT:
            size = sizeof(Package_Accept_Connection);
            break;
        case PKG_C2D_CONTROLL_COMMANDS:
            size = sizeof(Package_Controller_Commands);
            break;
        default:
            ESP_LOGE("espnow-send", "Unknown package type: %d", package->type);
            return;
    }
    ESP_LOGD("espnow-send", "Sending package of type %d, size %d", package->type, size);
    ESP_ERROR_CHECK(esp_now_send(mac_addr, (uint8_t *)package, size));
}

void advertise_drone_espnow(ESPnow_Controller_Connection *connection, int64_t interval_ms) {
    // Register Broadcast MAC Address
    esp_now_peer_info_t peerInfo = {};
    if (esp_now_get_peer(BROADCAST_MAC_ADDR, &peerInfo) == ESP_ERR_ESPNOW_NOT_FOUND) {
        peerInfo.channel = DEFAULT_ESPNOW_CHANNEL;
        peerInfo.encrypt = false;
        memcpy(peerInfo.peer_addr, BROADCAST_MAC_ADDR, 6); 
        peerInfo.ifidx = WIFI_IF_STA;
        ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
    } 

    connection->connected = false;
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    
    ESPnow_Packet* buf = malloc(sizeof(ESPnow_Packet));
    
    Package advertisement_package = {
        .type = PKG_D2C_ADVERTISE
    };

    while (!connection->connected) {
        // Send broadcast message
        ESP_LOGI("advertisement", "Advertising Drone");
        send_package_espnow(BROADCAST_MAC_ADDR, (Package *) &advertisement_package);

        if (xQueueReceive(s_esp_now_recv_queue, buf, interval_ms/portTICK_PERIOD_MS) == pdTRUE) {
            ESP_LOGI("advertisement", "Advertisement answered");
            free(buf->data);
            Package package;
            espnow_packet_parse(buf, &package);

            if (!is_broadcast_addr(buf->des_mac_addr)) {
                // Maybe check here for a specific message from the espnow controller
                if (package.type != PKG_C2D_ACCEPT_ADVERTISEMENT) {
                    ESP_LOGI("advertisement", "Advertisement answer not accepted, wrong package type: %d", package.type);
                } else {
                    ESP_LOGI("advertisement", "Advertisement answer accepted!");
                    break;
                }
            }
        }
    }

    connection->connected = true;
    memcpy(connection->mac_addr, buf->src_mac_addr, 6);
    memcpy(peerInfo.peer_addr, connection->mac_addr, 6);
    esp_now_add_peer(&peerInfo);


    // cleanup
    free(buf);
    xQueueReset(s_esp_now_recv_queue);
}

void espnow_packet_parse(ESPnow_Packet *packet, Package *package) {
    package->type = packet->data[0];
    memcpy(package, packet->data + sizeof(packet->data[0]), packet->len-1);
}

