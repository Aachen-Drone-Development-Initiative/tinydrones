#include "espnow_messaging.h"

// our components
#include "logging.h"

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_random.h"
#include "sys/time.h"

// Buffer sizes (the packets life in these buffers, so they should be rather big)
#define ESPNOW_RECEIVE_INFO_RINGBUF_SIZE (1024 * 10)
#define ESPNOW_SEND_INFO_RINGBUF_SIZE (1024 * 10)

// Packet Acknowledge system
#define MAX_ESPNOW_PACKETS_WAITING_FOR_ACK 10
#define ESPNOW_WAIT_FOR_ACK_RESEND_TIMEOUT_MS 100

// Max Latency on blocking ops
#define ESPNOW_RECEIVE_INFO_RINGBUF_MAX_TICKS_TO_WAIT pdMS_TO_TICKS(10)
#define ESPNOW_SEND_INFO_RINGBUF_MAX_TICKS_TO_WAIT pdMS_TO_TICKS(10)
#define ESPNOW_BLOCK_UNTIL_CONTROLLER_CONNECTED_LOOP_DELAY pdMS_TO_TICKS(100)
#define SERIAL_SEND_INFO_RINGBUF_MAX_TICKS_TO_WAIT pdMS_TO_TICKS(10)

// ESP-Now settings
#define ESPNOW_WIFI_MODE WIFI_MODE_STA // The Drone should be the Station mode, because it connects to the Controllers Access Point (AP)
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA // We want to use the Station interface (IF)

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    EPacket epkt;
} ESPNow_Receive_Info;

typedef struct {
    bool occupied;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    int64_t last_send_time_us;
    EPacket epkt;
} ESPNow_Send_Info;

#define MAX_ESPNOW_SEND_INFO_SIZE (sizeof(ESPNow_Send_Info) - sizeof(EPacket) + EPKT_MAX_SIZE)

typedef struct {
    bool in_peer_list;
    bool connected;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
} Controller_Info;

typedef struct {
    bool controller_connected;
    Payload_Controll_Commands ctrl_cmds; // Most recent controll commands
} Interface_State;

// Info about the controller
static Controller_Info s_controller = {0};

Interface_State s_interface_state = {0};
SemaphoreHandle_t s_interface_state_mutex = NULL;

// incoming packets
static RingbufHandle_t s_espnow_receive_info_ringbuf = NULL;

// outgoing packets
static RingbufHandle_t s_espnow_send_info_ringbuf = NULL;

// waiting packets
static uint8_t s_espnow_send_infos_waiting_for_ack[MAX_ESPNOW_PACKETS_WAITING_FOR_ACK][MAX_ESPNOW_SEND_INFO_SIZE] = {0};

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

    ESPNow_Receive_Info* e_recv_info = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_espnow_receive_info_ringbuf,
                                             (void**) &e_recv_info,
                                             sizeof(ESPNow_Receive_Info) - sizeof(EPacket) + length,
                                             ESPNOW_RECEIVE_INFO_RINGBUF_MAX_TICKS_TO_WAIT);

    if (ret != pdTRUE) {
        TINY_LOG_DEBUG("espnow_receive_callback: Creating espnow receive info in ringbuffer failed. The ringbuffer might be full.");
        return;
    }

    memcpy(e_recv_info->mac_addr, recv_info->src_addr, ESP_NOW_ETH_ALEN);
    memcpy(&e_recv_info->epkt, data, length);
    // We should probably just remove the packet from the ringbuffer if this fails, but not sure if it ever would
    // do that in case this assert ever triggers.
    TINY_ASSERT(e_recv_info->epkt.header.payload_size == length - sizeof(EPacket));

    ret = xRingbufferSendComplete(s_espnow_receive_info_ringbuf, (void*) e_recv_info);
    if (ret != pdTRUE) {
        TINY_LOG_DEBUG("espnow_receive_callback: Completing espnow receive info in ringbuffer failed.");
    }
}

static size_t espnow_packet_size(EHeader e_header) { return sizeof(EHeader) + e_header.payload_size; }

static ESPNow_Send_Info* espnow_send_info_acquire_raw(size_t packet_size)
{
    size_t e_send_info_size = sizeof(ESPNow_Send_Info) - sizeof(EPacket) + packet_size;
    ESPNow_Send_Info* e_send_info = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_espnow_send_info_ringbuf,
                                             (void**) &e_send_info,
                                             e_send_info_size,
                                             SERIAL_SEND_INFO_RINGBUF_MAX_TICKS_TO_WAIT);

    if (ret != pdTRUE) {
        TINY_LOG_DEBUG("espnow_send_info_acquire: Creating espnow send info in ringbuffer failed. The ringbuffer might be full.");
        return NULL;
    }

    memset(e_send_info, 0, e_send_info_size);
    memcpy(e_send_info->mac_addr, s_controller.mac_addr, ESP_NOW_ETH_ALEN);

    return e_send_info;
}

static ESPNow_Send_Info* espnow_send_info_acquire(EPacket_Type type, size_t packet_size, EPacket_Flags flags)
{
    TINY_ASSERT(s_controller.connected == true);

    ESPNow_Send_Info* e_send_info = espnow_send_info_acquire_raw(packet_size);
    if (e_send_info) {
        e_send_info->epkt.header = espnow_header_create(type, packet_size, flags);
    }

    return e_send_info;
}

static void espnow_send_info_complete(const ESPNow_Send_Info* e_send_info)
{
    UBaseType_t ret = xRingbufferSendComplete(s_espnow_send_info_ringbuf, (void*) e_send_info);
    
    if (ret != pdTRUE) {
        TINY_LOG_DEBUG("espnow_send_info_complete: Completing espnow send info in ringbuffer failed.");
    }
}

static void process_espnow_packet_acknowledge(const ESPNow_Send_Info* e_send_info)
{
    // Currently we dont need to do anything special for any packet after receiving their ack.
    return;
}

static Interface_State take_interface_state()
{
    // portMAX_DELAY to block indefinitely
    if (xSemaphoreTake(s_interface_state_mutex, portMAX_DELAY)) {
        return s_interface_state;
    }
    
    TINY_ABORT("unreachable");
}

static void give_interaface_state(Interface_State new_interface_state)
{
    s_interface_state = new_interface_state;
    xSemaphoreGive(s_interface_state_mutex);
}

static void process_incoming_espnow_packet(const ESPNow_Receive_Info* e_recv_info)
{
    const EPacket* epkt = &e_recv_info->epkt;
    
    bool unknown_sender = (memcmp(e_recv_info->mac_addr, s_controller.mac_addr, ESP_NOW_ETH_ALEN) != 0);

    if (unknown_sender)
    {
        // If the sender is not our controller, then they either want to connect. Or its a mistake and we should ignore the packet.
        if (epkt->header.type == EPKT_C2D_CONNECT)
        {
            // Already connected to a controller, do nothing.
            if (s_controller.connected) return;

            // Now remove the previous controller from the peer list (if there was a previous controller).
            // We do this here instead of after receiving the disconnect packet from the previous controller,
            // because we still need to acknowledge the disconnect packet.
            if (s_controller.in_peer_list) {
                esp_err_t ret = esp_now_del_peer(s_controller.mac_addr);
                if (ret != ESP_OK) {
                    TINY_LOG_DEBUG("process_incoming_espnow_packet: Failed to delete Controller from peer list, ESP_ERR: %s", esp_err_to_name(ret));
                }
            }
        
            // Try to add the controller to the espnow peer list
            esp_now_peer_info_t peer = {0};
            memcpy(peer.peer_addr, e_recv_info->mac_addr, ESP_NOW_ETH_ALEN);
            peer.channel = 0;
            peer.ifidx = ESPNOW_WIFI_IF;

            esp_err_t ret = esp_now_add_peer(&peer);
            if (ret != ESP_OK) {
                TINY_LOG_ERROR("process_incoming_espnow_packet: Failed to add Controller to peer list, ESP_ERR: %s", esp_err_to_name(ret));
                return;
            }

            s_controller.in_peer_list = true;
            s_controller.connected = true;
            memcpy(s_controller.mac_addr, e_recv_info->mac_addr, ESP_NOW_ETH_ALEN);

            // Update the interface state
            Interface_State if_state = take_interface_state();
            if_state.controller_connected = true;
            give_interaface_state(if_state);
        }
        else {
            TINY_LOG_ERROR("process_incoming_espnow_packet: Received packet of type '%d' from unknow sender, which is not allowed.", epkt->header.type);
        }
        
        return;
    }

    // If we need to send an ack, we should do that before sending any other espnow packets
    if (epkt->header.flags & EPKT_FLAG_REQUIRE_ACK)
    {
        ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(EPKT_C2D_ACK_RECEIVE, sizeof(EPacket_C2D_Ack_Receive), 0);
        if (e_send_info) {
            EPacket_C2D_Ack_Receive* epkt_c2d_ack = (EPacket_C2D_Ack_Receive*) &e_send_info->epkt;
            epkt_c2d_ack->acknowledged_packet_id = epkt->header.packet_id;
            espnow_send_info_complete(e_send_info);
        }
    }

    switch (epkt->header.type) {
    
    case EPKT_C2D_DISCONNECT:
    {
        // Already not connected, do nothing.
        if (!s_controller.connected) break;

        s_controller.connected = false;

        // Update the interface state
        Interface_State if_state = take_interface_state();
        if_state.controller_connected = false;
        give_interaface_state(if_state);
        
    } break;
    
    case EPKT_C2D_CONTROLL_COMMANDS:
    {
        EPacket_C2D_Controll_Commands* epkt_c2d_ctrl_cmds = (EPacket_C2D_Controll_Commands*) epkt;
            
        // Update the interface state
        Interface_State if_state = take_interface_state();
        if_state.ctrl_cmds = epkt_c2d_ctrl_cmds->ctrl_cmds;
        give_interaface_state(if_state);
        
    } break;
    
    case EPKT_C2D_ACK_RECEIVE:
    {
        EPacket_C2D_Ack_Receive* epkt_c2d_ack_recv = (EPacket_C2D_Ack_Receive*) epkt;
        EPacket_ID acknowledged_packet_id = epkt_c2d_ack_recv->acknowledged_packet_id;
        
        int i = 0;
        for (; i < MAX_ESPNOW_PACKETS_WAITING_FOR_ACK; ++i)
        {
            ESPNow_Send_Info* e_send_info = (ESPNow_Send_Info*) &s_espnow_send_infos_waiting_for_ack[i];
            if (e_send_info->occupied && e_send_info->epkt.header.packet_id == acknowledged_packet_id) {
                process_espnow_packet_acknowledge(e_send_info);
                memset(e_send_info, 0, MAX_ESPNOW_SEND_INFO_SIZE);
                break;
            }
        }

        if (i == MAX_ESPNOW_PACKETS_WAITING_FOR_ACK) {
            TINY_LOG_DEBUG("Received an EPKT_C2D_ACK_RECEIVE for packet '%d', but this packet was not in the 'wait for ack array'", acknowledged_packet_id);
        }

    } break;
        
    default:
        TINY_LOG_DEBUG("Received ESP-Now Packet with unexpected or unknown type: '%d'", epkt->header.type);
    }
}

static void send_all_queued_espnow_packets()
{
    while (1)
    {
        size_t e_send_info_size;
        ESPNow_Send_Info* e_send_info = (ESPNow_Send_Info*) xRingbufferReceive(s_espnow_send_info_ringbuf, &e_send_info_size, pdMS_TO_TICKS(0));
        if (!e_send_info) break;

        EPacket* epkt = &e_send_info->epkt;

        // TODO: We are trying to send the packet until it goes through, because its difficult to put
        // it back in the ringbuffer for some reason. This will be less problematic, when this
        // function would just be a task, because it wouldn't block everything.
        while (1) {
            esp_err_t ret = esp_now_send(e_send_info->mac_addr, (uint8_t*) epkt, espnow_packet_size(epkt->header));
            if (ret == ESP_OK) break;
            
            TINY_LOG_DEBUG("send_all_queued_espnow_packets: esp_now_send failed");
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Record the time this packet was sent
        struct timeval tv_now;
        gettimeofday(&tv_now, NULL);
        e_send_info->last_send_time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;

        if (epkt->header.flags & EPKT_FLAG_REQUIRE_ACK)
        {
            // Find empty slot in s_espnow_send_infos_waiting_for_ack array
            ESPNow_Send_Info* empty_slot = NULL;
            for (int i = 0; i < MAX_ESPNOW_PACKETS_WAITING_FOR_ACK; ++i) {
                if (((ESPNow_Send_Info*) &s_espnow_send_infos_waiting_for_ack[i])->occupied) {
                    empty_slot = (ESPNow_Send_Info*) &s_espnow_send_infos_waiting_for_ack[i];
                }
            }

            if (empty_slot) {
                memcpy(empty_slot, e_send_info, e_send_info_size);
            }
            else {
                TINY_LOG_DEBUG("send_all_queued_espnow_packets: Failed to add ESPNow Send Info to the 'wait for ack array', because all '%d' slots are full",
                               MAX_ESPNOW_PACKETS_WAITING_FOR_ACK);
            }
        }

        vRingbufferReturnItem(s_espnow_send_info_ringbuf, e_send_info);
    }

    // Resend all the packets that are waiting for an ack after their timeout period
    for (int i = 0; i < MAX_ESPNOW_PACKETS_WAITING_FOR_ACK; ++i)
    {
        ESPNow_Send_Info* e_send_info = (ESPNow_Send_Info*) &s_espnow_send_infos_waiting_for_ack[i];
        if (e_send_info->occupied)
        {
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL);
            int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;

            if (time_us - e_send_info->last_send_time_us > ESPNOW_WAIT_FOR_ACK_RESEND_TIMEOUT_MS * 1000L)
            {
                esp_err_t ret = esp_now_send(e_send_info->mac_addr, (uint8_t*) &e_send_info->epkt, espnow_packet_size(e_send_info->epkt.header));
                if (ret != ESP_OK) {
                    TINY_LOG_DEBUG("send_all_queued_espnow_packets: esp_now_send failed");
                    // When esp_now_send failed once, we shouldn't immeditely try to send more,
                    // instead we return to continue where we left of the next time.
                    return;
                }

                e_send_info->last_send_time_us = time_us;
            }
        }
    }
}

static void espnow_messaging_task(void*)
{
    while (1)
    {
        size_t e_recv_info_size;
        ESPNow_Receive_Info* e_recv_info = xRingbufferReceive(s_espnow_receive_info_ringbuf, &e_recv_info_size, pdMS_TO_TICKS(0));
        if (e_recv_info) {
            process_incoming_espnow_packet(e_recv_info);
            vRingbufferReturnItem(s_espnow_receive_info_ringbuf, &e_recv_info);
        }

        send_all_queued_espnow_packets();
        
        vTaskDelay(pdMS_TO_TICKS(10)); // for debugging
    }

    // Delete this task
    vTaskDelete(NULL);
}

// Public API

void espnow_init_and_start_task(bool long_range_wifi, uint32_t task_priority, uint32_t task_stack_size)
{
    // Init wifi
    if (esp_netif_init() != ESP_OK)                TINY_ABORT("espnow_init_and_start_task: esp_netif_init failed");
    if (esp_event_loop_create_default() != ESP_OK) TINY_ABORT("espnow_init_and_start_task: esp_event_loop_create_default failed");
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    
    if (esp_wifi_init(&cfg) != ESP_OK)                 TINY_ABORT("espnow_init_and_start_task: esp_wifi_init failed");
    if (esp_wifi_set_mode(ESPNOW_WIFI_MODE) != ESP_OK) TINY_ABORT("espnow_init_and_start_task: esp_wifi_set_mode failed");
    if (esp_wifi_start() != ESP_OK)                    TINY_ABORT("espnow_init_and_start_task: esp_wifi_start failed");

    if (long_range_wifi) {
        if (esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) != ESP_OK) {
            TINY_ABORT("init_wifi: esp_wifi_set_protocol failed");
        }
    }

    TINY_ASSERT(esp_now_init() == ESP_OK);
    TINY_ASSERT(esp_now_register_send_cb(espnow_send_callback) == ESP_OK);
    TINY_ASSERT(esp_now_register_recv_cb(espnow_receive_callback) == ESP_OK);
    
    // Adding boradcast mac address to peer list.
    esp_now_peer_info_t peer = {0};
    static const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; // TODO: test if this can just go in the {.x=y} initializer
    memcpy(peer.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    peer.channel = 0;
    peer.ifidx = ESPNOW_WIFI_IF;
    
    if (esp_now_add_peer(&peer) != ESP_OK) {
        TINY_ABORT("espnow_init_and_start_task: esp_now_add_peer failed");
    }

    // Initializing our own state
    s_interface_state_mutex = xSemaphoreCreateBinary();
    
    s_espnow_receive_info_ringbuf = xRingbufferCreate(ESPNOW_RECEIVE_INFO_RINGBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    s_espnow_send_info_ringbuf    = xRingbufferCreate(ESPNOW_SEND_INFO_RINGBUF_SIZE,    RINGBUF_TYPE_NOSPLIT);

    // Starting the task
    TaskHandle_t task_handle = NULL; // We don't keep the task handle, because we don't need it.
    xTaskCreate(espnow_messaging_task, "espnow_messaging_task", task_stack_size, NULL, task_priority, &task_handle);
    TINY_ASSERT(task_handle);
}

void espnow_block_until_controller_connected()
{
    while (1)
    {
        Interface_State if_state = take_interface_state();
        bool connected = if_state.controller_connected;
        give_interaface_state(if_state);
        
        if (connected) {
            return;
        }
        else {
            vTaskDelay(ESPNOW_BLOCK_UNTIL_CONTROLLER_CONNECTED_LOOP_DELAY);
        }
    }
}

bool espnow_send_log_to_controller(const char* format, ...)
{
    // Importantly we don't log any errors that might occur in this function!
    va_list args;

    static char pkt_buffer[EPKT_MAX_SIZE];
    EPacket_D2C_Log* epkt_log = (EPacket_D2C_Log*) &pkt_buffer;

    va_start(args, format);
    int log_msg_len = vsnprintf(epkt_log->msg, EPKT_MAX_SIZE - sizeof(EPacket_D2C_Log), format, args) + 1; // + 1 because '\0' is not counted
    va_end(args);

    size_t packet_size = sizeof(EPacket_D2C_Log) + log_msg_len;
    epkt_log->header = espnow_header_create(EPKT_D2C_LOG, packet_size, 0);
    
    return espnow_packet_send_to_controller((EPacket*) epkt_log);
}

bool espnow_get_latest_controll_commands(Payload_Controll_Commands* ctrl_cmds, TickType_t ticks_to_wait)
{
    if (xSemaphoreTake(s_interface_state_mutex, ticks_to_wait))
    {
        Payload_Controll_Commands latest_ctrl_cmds = s_interface_state.ctrl_cmds;
        xSemaphoreGive(s_interface_state_mutex);

        *ctrl_cmds = latest_ctrl_cmds;
        return true;
    }
    
    return false;
}

EHeader espnow_header_create(EPacket_Type type, size_t packet_size, EPacket_Flags flags)
{
    TINY_ASSERT(packet_size >= sizeof(EPacket));
    
    return (EHeader) {
        .type = type,
        .flags = flags,
        .packet_id = esp_random(),
        .payload_size = packet_size - sizeof(EPacket)
    };
}

bool espnow_packet_send_to_controller(EPacket* epkt)
{
    size_t epkt_size = espnow_packet_size(epkt->header);
    ESPNow_Send_Info* e_send_info = espnow_send_info_acquire_raw(epkt_size);
    if (e_send_info) {
        memcpy(&e_send_info->epkt, epkt, epkt_size);
        espnow_send_info_complete(e_send_info);
        return true;
    }
    return false;
}
