#include "esp_now_interface.h"

#define BROADCAST_MAC_ADDR (uint8_t[]){0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
#define ESP_NOW_QUEUE_LENGTH 10

void esp_now_cb(const esp_now_recv_info_t *recieved_packet, const uint8_t *data, int data_len) {
    ESPnow_Packet packet;
    ESP_LOGI("advertisement", "Received packet from %02X:%02X:%02X:%02X:%02X:%02X, length: %d", 
        recieved_packet->src_addr[0], recieved_packet->src_addr[1], recieved_packet->src_addr[2], 
        recieved_packet->src_addr[3], recieved_packet->src_addr[4], recieved_packet->src_addr[5],
        data_len);
    memcpy(packet.src_mac_addr, recieved_packet->src_addr, 6);
    memcpy(packet.des_mac_addr, recieved_packet->des_addr, 6);
    memcpy(packet.payload, data, data_len);
    packet.len = data_len;

    xQueueSend(s_esp_now_recv_queue, &packet, 0);
}

bool is_broadcast_addr(uint8_t* addr) {
    return (memcmp(addr, BROADCAST_MAC_ADDR, ESP_NOW_ETH_ALEN) == 0);
}

void esp_now_setup() {
    ESP_ERROR_CHECK(esp_now_init());
    s_esp_now_recv_queue = xQueueCreate(ESP_NOW_QUEUE_LENGTH, sizeof(ESPnow_Packet));
}

ESPnow_Controller_Connection advertise_drone(int64_t interval_ms) {
    ESPnow_Controller_Connection connection = {};
    // Register Broadcast MAC Address
    esp_now_peer_info_t peerInfo = {};
    if (esp_now_get_peer(BROADCAST_MAC_ADDR, &peerInfo) == ESP_ERR_ESPNOW_NOT_FOUND) {
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        memcpy(peerInfo.peer_addr, BROADCAST_MAC_ADDR, 6); 
        peerInfo.ifidx = WIFI_IF_STA;
        ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
    } 

    connection.connected = false;
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_cb));
    
    ESPnow_Packet* buf = malloc(sizeof(ESPnow_Packet));
    while (!connection.connected) {
        if (xQueueReceive(s_esp_now_recv_queue, buf, interval_ms/portTICK_PERIOD_MS) == pdTRUE) {
            ESP_LOGI("advertisement", "Advertisement answered: %d", buf->payload[0]);
            if (!is_broadcast_addr(buf->des_mac_addr)) {
                // Maybe check here for a specific message from the espnow controller
                ESP_LOGI("advertisement", "Advertisement answer accepted!");
                break;
            }

        }
        // Send broadcast message
        ESP_LOGI("advertisement", "Advertising Drone");
        uint8_t advertisement_msg[] = "TinyDrone Advertisement";
        esp_now_send(BROADCAST_MAC_ADDR, advertisement_msg, sizeof(advertisement_msg));
    }

    connection.connected = true;
    memcpy(connection.mac_addr, buf->src_mac_addr, 6);

    free(buf);
    xQueueReset(s_esp_now_recv_queue);
    return connection;
}

