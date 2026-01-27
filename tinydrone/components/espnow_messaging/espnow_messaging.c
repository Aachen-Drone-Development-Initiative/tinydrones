#include "espnow_messaging.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_now.h"

#define RECEIVE_CALLBACK_QUEUE_SIZE (10)
#define SEND_CALLBACK_QUEUE_SIZE (10)

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *packet_data;
    uint16_t packet_data_length;
} ESPNow_Receive_Info;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    EPacket* epkt;
    int64_t last_send_time_us;
} ESPNow_Send_Info;

static QueueHandle_t s_espnow_send_info_queue    = NULL;
static QueueHandle_t s_serial_send_info_queue    = NULL;

void controller_communication_task(void*);

static void espnow_send_callback(const esp_now_send_info_t *tx_info, esp_now_send_status_t status);
static void espnow_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int length);

void espnow_init_and_start_task(uint32_t task_priority, uint32_t task_stack_depth, TickType_t task_loop_delay)
{
    if (esp_now_init() != ESP_OK)                                    TINY_ABORT("espnow_init_and_start_task: esp_now_init failed");
    if (esp_now_register_send_cb(espnow_send_callback) != ESP_OK)    TINY_ABORT("espnow_init_and_start_task: esp_now_register_send_cb failed");
    if (esp_now_register_recv_cb(espnow_receive_callback) != ESP_OK) TINY_ABORT("espnow_init_and_start_task: esp_now_register_recv_cb failed");
    
    // Adding boradcast mac address to peer list.
    esp_now_peer_info_t peer = {0};
    static const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // TODO: test if this can just go in the {.x=y} initializer
    memcpy(&peer.peer_addr, &broadcast_mac, ESP_NOW_ETH_ALEN);
    peer.channel = 0;
    peer.ifidx = ESPNOW_WIFI_IF;
    
    if (esp_now_add_peer(&peer) != ESP_OK) {
        TINY_ABORT("espnow_init_and_start_task: esp_now_add_peer failed");
    }

    s_espnow_receive_info_queue = xQueueCreate(ESPNOW_RECEIVE_INFO_QUEUE_SIZE, sizeof(ESPNow_Receive_Info));
    s_espnow_send_info_queue    = xQueueCreate(ESPNOW_SEND_INFO_QUEUE_SIZE, sizeof(ESPNow_Send_Info));

    // ALSO CONTINUE HERE !!!!
}

static void espnow_receive_info_destroy(ESPNow_Receive_Info* e_recv_info)
{
    DGL_FREE(e_recv_info->packet_data);
    e_recv_info->packet_data = NULL;
    e_recv_info->packet_data_length = 0;
}

static void espnow_send_callback(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    /* This callback is not really utilized, because even if 'status' is ESP_NOW_SEND_FAIL
       we won't send the packet again. Mainly because it's difficult to know
       which packet 'tx_info' refers to (trust me its difficult). */

    if (status != ESP_NOW_SEND_SUCCESS) {
        TINY_LOG_DEBUG("ESP-Now send callback reported that sending a packet failed.");
    }
}

static void espnow_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int length)
{
    if (recv_info == NULL || recv_info->src_addr == NULL || data == NULL || length <= 0) {
        TINY_LOG_ERROR("espnow_receive_callback: arg error");
        return;
    }
    
    ESPNow_Receive_Info e_recv_info = {0};
    memcpy(e_recv_info.mac_addr, recv_info->src_addr, ESP_NOW_ETH_ALEN);

    // CONTINUE HERE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    /* e_recv_info.packet_data = TINY_MALLOC(length); */
    
    memcpy(e_recv_info.packet_data, data, length);
    e_recv_info.packet_data_length = length;
    
    if (!xQueueSend(s_espnow_receive_info_queue, &e_recv_info, ESPNOW_RECEIVE_INFO_QUEUE_SEND_DELAY)) {
        TINY_LOG_DEBUG("espnow_receive_callback: Sending espnow receive info to queue failed. The queue might be full.");
        espnow_receive_info_destroy(&e_recv_info);
    }
}

/* Terminates the task that handles receiving/sending packets, frees
   any allocated memory, resets all values and deinitializes ESPNow. */

void espnow_deinit();

/* Returns true when a controller connected successfully to the drone.
   The drone should wait for this event, by calling this function in a loop
   with some delay. */

bool espnow_controller_connected();

bool espnow_send_log_to_controller(const char* format, ...); /* works just like printf */

EHeader espnow_header_create(EPacket_Type type, size_t packet_size, EPacket_Flags flags); /* use this to initialize the header of your EPacket */

bool espnow_get_latest_controll_commands(Payload_Controll_Commands* ctrl_cmds);

bool espnow_packet_send_to_controller(EPacket* epkt);


void controller_communication_task(void*)
{
    while (1)
    {
        process_incoming_espnow_packet();

        send_all_queued_espnow_packets();
    }
}
