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

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} ESPNow_Send_Callback_Info;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *packet_data;
    uint16_t packet_data_length;
} ESPNow_Receive_Callback_Info;

static void destroy_espnow_receive_callback_info(ESPNow_Receive_Callback_Info recv_info)
{
    free(recv_info.packet_data);
}

typedef void (Receive_Pkt_Handler*)(Packet*, uint8_t* mac_addr);

static void recieve_c2d_connect(Packet*, uint8_t* mac_addr);
static void recieve_c2d_disconnect(Packet*, uint8_t* mac_addr);
static void recieve_c2d_controll_commands(Packet*, uint8_t* mac_addr);

static const Receive_Pkt_Handler s_receive_pkt_handler_table[PKT_TYPE_UPPER_BOUND] =
{
    [PKT_C2D_CONNECT]           = { recieve_c2d_connect },
    [PKT_C2D_DISCONNECT]        = { recieve_c2d_disconnect },
    [PKT_C2D_CONTROLL_COMMANDS] = { recieve_c2d_controll_commands },
};

// Begin static variables

static QueueHandle_t s_espnow_receive_callback_queue = NULL; // Queue for ESPNow_Receive_Callback_Info
static QueueHandle_t s_espnow_send_callback_queue = NULL; // Queue for ESPNow_Send_Callback_Info
static TaskHandle_t* s_ctrl_comm_task_handle = NULL;
static TickType_t s_task_loop_delay = 0;
static bool s_is_running = false;

// End static variables

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status);
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int length);

void ctrl_comm_task(void*);

void ctrl_comm_init_and_start_task(uint32_t task_priority,
                                   uint32_t task_stack_depth,
                                   TickType_t task_loop_delay)
{
    // Initalizing ESPNow
    
    if (esp_now_init() != ESP_OK)                           TINY_ABORT("init_espnow: esp_now_init failed");
    if (esp_now_register_send_cb(espnow_send_cb) != ESP_OK) TINY_ABORT("init_espnow: esp_now_register_send_cb failed");
    if (esp_now_register_recv_cb(espnow_recv_cb) != ESP_OK) TINY_ABORT("init_espnow: esp_now_register_recv_cb failed");
    
    // Adding broadcast peer to peer list
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) TINY_ABORT("init_espnow: malloc failed");
    
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = 0;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_broadcast_mac, sizeof(s_broadcast_mac));
    
    if (esp_now_add_peer(peer) != ESP_OK) TINY_ABORT("init_espnow: esp_now_add_peer failed");
    
    free(peer);

    // Creating the task
    
    BaseType_t ret_task_create = xTaskCreate(ctrl_comm_task,
                                             "ctrl_comm_task",
                                             task_stack_depth,
                                             NULL,
                                             task_priority,
                                             s_ctrl_comm_task_handle);

    if (ret_task_create != pdPASS) TINY_ABORT("Failed to create task ctrl_comm_task");

    s_task_loop_delay = task_loop_delay;
    s_is_running = true;

    // Initialize queues for receive and send events
    s_espnow_receive_callback_queue = xQueueCreate(RECEIVE_CALLBACK_QUEUE_SIZE, sizeof(ESPNow_Receive_Callback_Info));
    s_espnow_send_callback_queue    = xQueueCreate(SEND_CALLBACK_QUEUE_SIZE, sizeof(ESPNow_Send_Callback_Info));
}

void ctrl_comm_shutdown()
{
    s_is_running = false;
    
    vQueueDelete(s_espnow_receive_callback_queue);
    vQueueDelete(s_espnow_send_callback_queue);

    esp_now_deinit();
}

static Packet* create_packet(int packet_size, Packet_Type type)
{
    assert(packet_size >= sizeof(Packet));
    Packet* pkt = malloc(packet_size);
    if (pkt == NULL) TINY_ABORT("create_packet: malloc failed");

    pkt->type = type;
    pkt->payload_size = packet_size - sizeof(Packet);
    
    return pkt;
}

static void destroy_packet(Packet* pkt)
{
    free(pkt);
}

static bool send_packet_to_controller_with_espnow(uint8_t* mac_addr, Packet* pkt)
{
    if (esp_now_send(mac_addr, (uint8_t*) pkt, packet_size(pkt)) != ESP_OK) {
        TINY_ERROR_NEVER_SEND("send_packet_to_controller_with_espnow: send packet error");
        return false;
    }
    
    return true;
}

bool ctrl_comm_is_connected()
{
    
}

void ctrl_comm_send_c2u_log(Log_Type type, const char* format, ...)
{
    va_list args;
    
    va_start(args, format);
    int msg_len = vsnprintf(NULL, 0, format, args);
    va_end(args);
    
    int pkt_log_size = sizeof(Packet_D2C_Log) + (msg_len + 1) * sizeof(char);

    Packet_D2C_Log* pkt_log = (Packet_D2C_Log*) create_packet(pkt_log_size, PKT_D2C_LOG);
    pkt_log->log_type = type;
    pkt_log->msg_length = msg_len + 1;
    
    va_start(args, format);
    vsnprintf(pkt_log->msg, msg_len + 1, format, args);
    va_end(args);

    send_packet_to_controller_with_espnow((Packet*) pkt_log);
    destroy_packet((Packet*) pkt_log);
}

void ctrl_comm_task(void*)
{
    while (s_is_running)
    {
        // Handle incoming packets from the drone via ESP-Now
        
        ESPNow_Receive_Callback_Info recv_info = { 0 };
        if (xQueueReceive(s_espnow_receive_callback_queue, &recv_info, 0))
        {
            Packet* pkt = (Packet*) recv_info.packet_data;
            Receive_Pkt_Handler recv_pkt_handler = s_receive_pkt_handler_table[pkt->type];

            if (recv_info.packet_data_length == packet_size(pkt))
            {
                if (recv_pkt_handler != NULL) {
                    recv_pkt_handler(pkt, recv_info.mac_addr);
                }
                else TINY_ERROR("Received unexpected or unknown packet from the controller");
            }
            else TINY_ERROR("recv_info.packet_data_length != packet_size(pkt)");
        }
        destroy_espnow_receive_callback_info(recv_info);

        // Handle ESP-Now send Callback

        ESPNow_Send_Callback_Info send_info = { 0 };
        if (xQueueReceive(s_espnow_send_callback_queue, &send_info, 0))
        {
            if (send_info.status == ESP_NOW_SEND_FAIL) {
                TINY_DBG_INFO("Failed to packets to controller with MAC:"MACSTR, MAC2STR(send_info.mac_addr));
            }
        }

        vTaskDelay(s_task_loop_delay);
    }
    
    vTaskDelete(NULL); // Delete this task
}

/* void espnow_recv_cb(const esp_now_recv_info_t *recieved_packet, const uint8_t *data, int data_len) { */
/*     ESPnow_Packet espnow_packet; */
/*     ESP_LOGI("espnow", "Received espnow_packet from %02X:%02X:%02X:%02X:%02X:%02X, length: %d",  */
/*         recieved_packet->src_addr[0], recieved_packet->src_addr[1], recieved_packet->src_addr[2],  */
/*         recieved_packet->src_addr[3], recieved_packet->src_addr[4], recieved_packet->src_addr[5], */
/*         data_len); */

/*     espnow_packet.data = malloc(data_len); */

/*     espnow_packet.len = data_len; */
/*     memcpy(espnow_packet.src_mac_addr, recieved_packet->src_addr, 6); */
/*     memcpy(espnow_packet.des_mac_addr, recieved_packet->des_addr, 6); */
/*     memcpy(espnow_packet.data, data, data_len); */

/*     xQueueSend(s_esp_now_recv_queue, &espnow_packet, 0); */
/* } */

/* bool is_broadcast_addr(uint8_t* addr) { */
/*     return (memcmp(addr, BROADCAST_MAC_ADDR, ESP_NOW_ETH_ALEN) == 0); */
/* } */

/* void esp_now_setup() { */
/*     ESP_ERROR_CHECK(esp_now_init()); */
/*     s_esp_now_recv_queue = xQueueCreate(ESP_NOW_QUEUE_LENGTH, sizeof(ESPnow_Packet)); */
/* } */

/* void send_package_espnow(uint8_t *mac_addr, Package *package) { */
/*     // convert package to espnow data */
/*     int16_t size; */
/*     switch (package->type) { */
/*         case PKG_D2C_ADVERTISE: */
/*             size = sizeof(Package_Request_Connection); */
/*             break; */
/*         case PKG_D2C_IMU_DATA: */
/*             size = sizeof(Package_Imu_Data); */
/*             break; */
/*         case PKG_C2D_ACCEPT_ADVERTISEMENT: */
/*             size = sizeof(Package_Accept_Connection); */
/*             break; */
/*         case PKG_C2D_CONTROLL_COMMANDS: */
/*             size = sizeof(Package_Controller_Commands); */
/*             break; */
/*         default: */
/*             ESP_LOGE("espnow-send", "Unknown package type: %d", package->type); */
/*             return; */
/*     } */
/*     ESP_LOGD("espnow-send", "Sending package of type %d, size %d", package->type, size); */
/*     ESP_ERROR_CHECK(esp_now_send(mac_addr, (uint8_t *)package, size)); */
/* } */

/* void advertise_drone_espnow(ESPnow_Controller_Connection *connection, int64_t interval_ms) { */
/*     // Register Broadcast MAC Address */
/*     esp_now_peer_info_t peerInfo = {}; */
/*     if (esp_now_get_peer(BROADCAST_MAC_ADDR, &peerInfo) == ESP_ERR_ESPNOW_NOT_FOUND) { */
/*         peerInfo.channel = DEFAULT_ESPNOW_CHANNEL; */
/*         peerInfo.encrypt = false; */
/*         memcpy(peerInfo.peer_addr, BROADCAST_MAC_ADDR, 6);  */
/*         peerInfo.ifidx = WIFI_IF_STA; */
/*         ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo)); */
/*     }  */

/*     connection->connected = false; */
/*     ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb)); */
    
/*     ESPnow_Packet* buf = malloc(sizeof(ESPnow_Packet)); */
    
/*     Package advertisement_package = { */
/*         .type = PKG_D2C_ADVERTISE */
/*     }; */

/*     while (!connection->connected) { */
/*         // Send broadcast message */
/*         ESP_LOGI("advertisement", "Advertising Drone"); */
/*         send_package_espnow(BROADCAST_MAC_ADDR, (Package *) &advertisement_package); */

/*         if (xQueueReceive(s_esp_now_recv_queue, buf, interval_ms/portTICK_PERIOD_MS) == pdTRUE) { */
/*             ESP_LOGI("advertisement", "Advertisement answered"); */
/*             free(buf->data); */
/*             Package package; */
/*             espnow_packet_parse(buf, &package); */

/*             if (!is_broadcast_addr(buf->des_mac_addr)) { */
/*                 // Maybe check here for a specific message from the espnow controller */
/*                 if (package.type != PKG_C2D_ACCEPT_ADVERTISEMENT) { */
/*                     ESP_LOGI("advertisement", "Advertisement answer not accepted, wrong package type: %d", package.type); */
/*                 } else { */
/*                     ESP_LOGI("advertisement", "Advertisement answer accepted!"); */
/*                     break; */
/*                 } */
/*             } */
/*         } */
/*     } */

/*     connection->connected = true; */
/*     memcpy(connection->mac_addr, buf->src_mac_addr, 6); */
/*     memcpy(peerInfo.peer_addr, connection->mac_addr, 6); */
/*     esp_now_add_peer(&peerInfo); */


/*     // cleanup */
/*     free(buf); */
/*     xQueueReset(s_esp_now_recv_queue); */
/* } */

/* void espnow_packet_parse(ESPnow_Packet *packet, Package *package) { */
/*     package->type = packet->data[0]; */
/*     memcpy(package, packet->data + sizeof(packet->data[0]), packet->len-1); */
/* } */

