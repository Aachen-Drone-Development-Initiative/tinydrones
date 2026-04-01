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
#include <stdatomic.h>

// Buffer sizes (the packets life in these buffers, so they should be rather big)
#define ESPNOW_RECEIVE_INFO_RINGBUF_SIZE (1024 * 10)
#define ESPNOW_SEND_INFO_RINGBUF_SIZE    (1024 * 10)

// Packet acknowledging
#define MAX_ESPNOW_PACKETS_WAITING_FOR_ACK    10
#define ESPNOW_WAIT_FOR_ACK_RESEND_TIMEOUT_MS 100

// Max latency on blocking ops
#define ESPNOW_RECEIVE_INFO_RINGBUF_MAX_TICKS_TO_WAIT      pdMS_TO_TICKS(10)
#define ESPNOW_SEND_INFO_RINGBUF_MAX_TICKS_TO_WAIT         pdMS_TO_TICKS(10)
#define ESPNOW_BLOCK_UNTIL_CONTROLLER_CONNECTED_LOOP_DELAY pdMS_TO_TICKS(100)
#define SERIAL_SEND_INFO_RINGBUF_MAX_TICKS_TO_WAIT         pdMS_TO_TICKS(10)

// Repeating events cooldown
#define ADVERTISE_COOLDOWN_MS 1000

// ESP-Now settings
#define ESPNOW_WIFI_MODE WIFI_MODE_STA // The Drone should be the Station mode, because it connects to the Controllers Access Point (AP)
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA // We want to use the Station interface (IF)

static const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// TODO: This should not be hardcoded obviously
static const char drone_name[MAX_DRONE_NAME_LENGTH] = "tiny1";

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    EPacket epkt;
} ESPNow_Receive_Info;

typedef struct {
    bool    occupied;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    int64_t last_send_time_us;
    EPacket epkt;
} ESPNow_Send_Info;

#define MAX_ESPNOW_SEND_INFO_SIZE (sizeof(ESPNow_Send_Info) - sizeof(EPacket) + EPKT_MAX_SIZE)

typedef struct
{
    bool in_peer_list;
    atomic_bool connected;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    
    // Latest controll commands
    atomic_int pitch;
    atomic_int yaw;
    atomic_int roll;
    atomic_int throttle;
    atomic_int flags;
    
} Controller_Info;

// Info about the currently connected controller
static Controller_Info s_controller = {0};

// Incoming packets
static RingbufHandle_t s_espnow_receive_info_ringbuf = NULL;

// Outgoing packets
static RingbufHandle_t s_espnow_send_info_ringbuf = NULL;

// Waiting packets (for acknowledgement)
static uint8_t s_espnow_send_infos_waiting_for_ack[MAX_ESPNOW_PACKETS_WAITING_FOR_ACK][MAX_ESPNOW_SEND_INFO_SIZE] = {0};

static bool get_controller_connected()
{
    return atomic_load(&s_controller.connected);
}

static void set_controller_connected(bool connected)
{
    atomic_store(&s_controller.connected, connected);
}

static Payload_Controll_Commands get_control_commands()
{
    return (Payload_Controll_Commands) {
        .pitch    = atomic_load(&s_controller.pitch),
        .yaw      = atomic_load(&s_controller.yaw),
        .roll     = atomic_load(&s_controller.roll),
        .throttle = atomic_load(&s_controller.throttle),
        .flags    = atomic_load(&s_controller.flags),
    };
}

static void set_control_commands(Payload_Controll_Commands ctrl_cmds)
{
    atomic_store(&s_controller.pitch,    ctrl_cmds.pitch);
    atomic_store(&s_controller.yaw,      ctrl_cmds.yaw);
    atomic_store(&s_controller.roll,     ctrl_cmds.roll);
    atomic_store(&s_controller.throttle, ctrl_cmds.throttle);
    atomic_store(&s_controller.flags,    ctrl_cmds.flags);
}

static void espnow_send_callback(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    // This callback is not really utilized, because even if 'status' is ESP_NOW_SEND_FAIL
    // we won't send the packet again. Mainly because it's difficult to know
    // which packet 'tx_info' refers to (trust me its difficult).

    if (status != ESP_NOW_SEND_SUCCESS) {
        LOG_DEBUG("ESP-Now send callback reported that sending a packet failed.");
    }
}

static void espnow_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int length)
{
    if (recv_info == NULL || recv_info->src_addr == NULL || data == NULL || length <= 0) {
        CONTR_LOG_ERROR("espnow_receive_callback: arg error");
        return;
    }

    ESPNow_Receive_Info* e_recv_info = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_espnow_receive_info_ringbuf,
                                             (void**) &e_recv_info,
                                             sizeof(ESPNow_Receive_Info) - sizeof(EPacket) + length,
                                             ESPNOW_RECEIVE_INFO_RINGBUF_MAX_TICKS_TO_WAIT);

    if (ret != pdTRUE) {
        CONTR_LOG_DEBUG("espnow_receive_callback: Creating espnow receive info in ringbuffer failed. The ringbuffer might be full.");
        return;
    }

    memcpy(e_recv_info->mac_addr, recv_info->src_addr, ESP_NOW_ETH_ALEN);
    memcpy(&e_recv_info->epkt, data, length);
    // We should probably just remove the packet from the ringbuffer if this fails, but not sure if it ever would
    // do that in case this assert ever triggers.
    assert(e_recv_info->epkt.header.payload_size == length - sizeof(EPacket));

    ret = xRingbufferSendComplete(s_espnow_receive_info_ringbuf, (void*) e_recv_info);
    if (ret != pdTRUE) {
        CONTR_LOG_DEBUG("espnow_receive_callback: Completing espnow receive info in ringbuffer failed.");
    }
}

static size_t espnow_packet_size(EHeader e_header) { return sizeof(EHeader) + e_header.payload_size; }

static ESPNow_Send_Info* espnow_send_info_acquire_raw(const uint8_t* dest_mac_addr, size_t packet_size)
{
    size_t e_send_info_size = sizeof(ESPNow_Send_Info) - sizeof(EPacket) + packet_size;
    ESPNow_Send_Info* e_send_info = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_espnow_send_info_ringbuf,
                                             (void**) &e_send_info,
                                             e_send_info_size,
                                             SERIAL_SEND_INFO_RINGBUF_MAX_TICKS_TO_WAIT);

    if (ret != pdTRUE) {
        LOG_DEBUG("espnow_send_info_acquire_raw: Creating espnow send info in ringbuffer failed. The ringbuffer might be full.");
        return NULL;
    }

    memset(e_send_info, 0, e_send_info_size);
    memcpy(e_send_info->mac_addr, dest_mac_addr, ESP_NOW_ETH_ALEN);

    return e_send_info;
}

static ESPNow_Send_Info* espnow_send_info_acquire(const uint8_t* dest_mac_addr,
                                                  EPacket_Type type,
                                                  size_t packet_size,
                                                  EPacket_Flags flags)
{
    ESPNow_Send_Info* e_send_info = espnow_send_info_acquire_raw(dest_mac_addr, packet_size);
    if (e_send_info) {
        e_send_info->epkt.header = espnow_header_create(type, packet_size, flags);
    }

    return e_send_info;
}

static void espnow_send_info_complete(const ESPNow_Send_Info* e_send_info)
{
    UBaseType_t ret = xRingbufferSendComplete(s_espnow_send_info_ringbuf, (void*) e_send_info);
    
    if (ret != pdTRUE) {
        LOG_DEBUG("espnow_send_info_complete: Completing espnow send info in ringbuffer failed.");
    }
}

static void process_espnow_packet_acknowledge(const ESPNow_Send_Info* e_send_info)
{
    return; // Currently we dont need to do anything special for any packet after receiving their ack.
}

static void process_incoming_espnow_packet(const ESPNow_Receive_Info* e_recv_info)
{
    const EPacket* epkt = &e_recv_info->epkt;

    LOG_DEBUG("processing incommig packet");

    bool from_controller = (memcmp(e_recv_info->mac_addr, s_controller.mac_addr, ESP_NOW_ETH_ALEN) == 0);

    if (!from_controller && epkt->header.type != EPKT_C2D_CONNECT) {
        CONTR_LOG_ERROR("process_incoming_espnow_packet: Received packet of type '%d' from unknow sender, which is not allowed.", epkt->header.type);
        return;
    }
    
    // If the sender is not our controller, then they either want to connect. Or its a mistake and we should ignore the packet.
    if (epkt->header.type == EPKT_C2D_CONNECT)
    {
        // Already connected to a controller, do nothing.
        if (get_controller_connected()) return;

        // Remove the previous controller from the peer list (if there was a previous controller).
        // We do this here instead of after receiving the disconnect packet from the previous controller,
        // because we still need to communicate with the previous controller (at least for the disconnect ack).
        if (s_controller.in_peer_list) {
            esp_err_t ret = esp_now_del_peer(s_controller.mac_addr);
            if (ret != ESP_OK) {
                CONTR_LOG_DEBUG("process_incoming_espnow_packet: Failed to delete Controller from peer list, ESP_ERR: %s", esp_err_to_name(ret));
            }
            s_controller.in_peer_list = false;
        }
        
        // Try to add the controller to the espnow peer list
        esp_now_peer_info_t peer = {0};
        memcpy(peer.peer_addr, e_recv_info->mac_addr, ESP_NOW_ETH_ALEN);
        peer.channel = 0;
        peer.ifidx = ESPNOW_WIFI_IF;
        esp_err_t ret = esp_now_add_peer(&peer);
        if (ret != ESP_OK) {
            CONTR_LOG_ERROR("process_incoming_espnow_packet: Failed to add Controller to peer list, ESP_ERR: %s", esp_err_to_name(ret));
            return;
        }

        s_controller.in_peer_list = true;
        set_controller_connected(true);
        memcpy(s_controller.mac_addr, e_recv_info->mac_addr, ESP_NOW_ETH_ALEN);
    }
    
    // If we need to send an ack, we should do that after potentially receiving EPKT_C2D_CONNECT,
    // but before sending any other packets (as soon after receiving as possible)
    if (epkt->header.flags & EPKT_FLAG_REQUIRE_ACK)
    {
        LOG_DEBUG("sending an ack");
        ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(s_controller.mac_addr, EPKT_D2C_ACK_RECEIVE, sizeof(EPacket_D2C_Ack_Receive), 0);
        if (e_send_info) {
            EPacket_D2C_Ack_Receive* epkt_d2c_ack = (EPacket_D2C_Ack_Receive*) &e_send_info->epkt;
            epkt_d2c_ack->acknowledged_packet_id = epkt->header.packet_id;
            espnow_send_info_complete(e_send_info);
        }
    }
    
    switch (epkt->header.type) {

    case EPKT_C2D_CONNECT:
        break; // this was already handeled above
        
    case EPKT_C2D_DISCONNECT:
    {
        if (!get_controller_connected()) break;
        set_controller_connected(false);
        
    } break;
    
    case EPKT_C2D_CONTROLL_COMMANDS:
    {
        EPacket_C2D_Controll_Commands* epkt_c2d_ctrl_cmds = (EPacket_C2D_Controll_Commands*) epkt;
        set_control_commands(epkt_c2d_ctrl_cmds->ctrl_cmds);
        
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
            CONTR_LOG_DEBUG("Received an EPKT_C2D_ACK_RECEIVE for packet '%d', but this packet "
                            "was not in the 'wait for ack array'", acknowledged_packet_id);
        }

    } break;

    case EPKT_C2D_PING:
        break; // Nothing to do except sending an ack
        
    default:
        CONTR_LOG_DEBUG("Received ESP-Now Packet with unexpected type: '%d'", epkt->header.type);
    }
}

static void send_all_queued_espnow_packets()
{
    while (1)
    {
        /* LOG_DEBUG("try sending esp now packet") */
        size_t e_send_info_size;
        ESPNow_Send_Info* e_send_info = (ESPNow_Send_Info*) xRingbufferReceive(s_espnow_send_info_ringbuf, &e_send_info_size, pdMS_TO_TICKS(0));
        if (!e_send_info) break;

        LOG_DEBUG("sending esp now packet")

        EPacket* epkt = &e_send_info->epkt;

        esp_err_t ret = esp_now_send(e_send_info->mac_addr, (uint8_t*) epkt, espnow_packet_size(epkt->header));
        if (ret == ESP_OK)
        {
            // Record the time this packet was sent
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL);
            e_send_info->last_send_time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;

            if (epkt->header.flags & EPKT_FLAG_REQUIRE_ACK)
            {
                // Find empty slot in s_espnow_send_infos_waiting_for_ack array
                ESPNow_Send_Info* empty_slot = NULL;
                for (int i = 0; i < MAX_ESPNOW_PACKETS_WAITING_FOR_ACK; ++i) {
                    if (((ESPNow_Send_Info*) &s_espnow_send_infos_waiting_for_ack[i])->occupied == false) {
                        empty_slot = (ESPNow_Send_Info*) &s_espnow_send_infos_waiting_for_ack[i];
                        break;
                    }
                }

                if (empty_slot) {
                    memcpy(empty_slot, e_send_info, e_send_info_size);
                }
                else {
                    CONTR_LOG_DEBUG("send_all_queued_espnow_packets: Failed to add ESPNow Send Info to the 'wait for ack array', "
                                    "because all '%d' slots are full", MAX_ESPNOW_PACKETS_WAITING_FOR_ACK);
                }
            }
        }
        else {
            LOG_DEBUG("send_all_queued_espnow_packets: esp_now_send failed, ESP_ERR: %s", esp_err_to_name(ret));
        }

        vRingbufferReturnItem(s_espnow_send_info_ringbuf, e_send_info);
    }

    // Resend all the packets that are waiting for an ack, if their resend cooldown is over
    for (int i = 0; i < MAX_ESPNOW_PACKETS_WAITING_FOR_ACK; ++i)
    {
        ESPNow_Send_Info* e_send_info = (ESPNow_Send_Info*) &s_espnow_send_infos_waiting_for_ack[i];
        if (e_send_info->occupied == false) {
            continue;
        }

        struct timeval tv_now;
        gettimeofday(&tv_now, NULL);
        int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;

        if (time_us - e_send_info->last_send_time_us > ESPNOW_WAIT_FOR_ACK_RESEND_TIMEOUT_MS * 1000L)
        {
            esp_err_t ret = esp_now_send(e_send_info->mac_addr, (uint8_t*) &e_send_info->epkt, espnow_packet_size(e_send_info->epkt.header));
            if (ret == ESP_OK) {
                e_send_info->last_send_time_us = time_us;
            }
            else {
                LOG_DEBUG("send_all_queued_espnow_packets: esp_now_send failed: %s", esp_err_to_name(ret));
                if (ret == ESP_ERR_ESPNOW_NO_MEM) {
                    // We should delay a while before sending again, so lets just return and continue the next time.
                    return;
                }
            }
        }
    }
}

static void send_advertisement()
{
    ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(broadcast_mac, EPKT_D2C_ADVERTISE, sizeof(EPacket_D2C_Advertise), 0);
    if (e_send_info) {
        EPacket_D2C_Advertise* epkt_d2c_ad = (EPacket_D2C_Advertise*) &e_send_info->epkt;
        memcpy(epkt_d2c_ad->drone_name, drone_name, MAX_DRONE_NAME_LENGTH);
        espnow_send_info_complete(e_send_info);
    }
}

// Main task
static void espnow_messaging_task(void*)
{
    int64_t time_of_last_advertise_ms = 0;
    
    while (1)
    {
        size_t e_recv_info_size;
        ESPNow_Receive_Info* e_recv_info = xRingbufferReceive(s_espnow_receive_info_ringbuf, &e_recv_info_size, pdMS_TO_TICKS(0));
        if (e_recv_info) {
            process_incoming_espnow_packet(e_recv_info);
            vRingbufferReturnItem(s_espnow_receive_info_ringbuf, e_recv_info);
        }

        // Send advertisement to the broadcast address (reaches everyone nearby)
        // once in a while, when we are not connected to any controller.
        if (!get_controller_connected())
        {
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL);
            int64_t time_ms = ((int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec) / 1000;
            
            if (time_ms - time_of_last_advertise_ms >= ADVERTISE_COOLDOWN_MS) {
                time_of_last_advertise_ms = time_ms;
                send_advertisement();
            }
        }

        send_all_queued_espnow_packets();

        // TODO: this delay is necessary because we need to yield for other tasks
        // and also give some time to lower priority tasks. Actually we should block
        // on a queue in this task instead, but we cant because we have two queues, that we would
        // have to block on, so they should probably be unified or the task should be split.
        vTaskDelay(1);
    }

    // Delete this task
    vTaskDelete(NULL);
}


// Public API implementation


void espnow_init_and_start_task(bool long_range_wifi, uint32_t task_priority, uint32_t task_stack_size)
{
    // Init wifi
    
    assert(esp_netif_init() == ESP_OK);
    assert(esp_event_loop_create_default() == ESP_OK);
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    
    assert(esp_wifi_init(&cfg) == ESP_OK);
    assert(esp_wifi_set_mode(ESPNOW_WIFI_MODE) == ESP_OK);
    assert(esp_wifi_start() == ESP_OK);

    if (long_range_wifi) {
        assert(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR)
               == ESP_OK);
    }

    assert(esp_now_init() == ESP_OK);
    assert(esp_now_register_send_cb(espnow_send_callback) == ESP_OK);
    assert(esp_now_register_recv_cb(espnow_receive_callback) == ESP_OK);
    
    // Adding boradcast mac address to peer list.
    
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    peer.channel = 0;
    peer.ifidx = ESPNOW_WIFI_IF;

    assert(esp_now_add_peer(&peer) == ESP_OK);

    s_espnow_receive_info_ringbuf = xRingbufferCreate(ESPNOW_RECEIVE_INFO_RINGBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    s_espnow_send_info_ringbuf    = xRingbufferCreate(ESPNOW_SEND_INFO_RINGBUF_SIZE,    RINGBUF_TYPE_NOSPLIT);

    // Starting the task
    TaskHandle_t task_handle = NULL; // We don't keep the task handle, because we don't need it.
    xTaskCreate(espnow_messaging_task, "espnow_messaging_task", task_stack_size, NULL, task_priority, &task_handle);
    assert(task_handle);
}

void espnow_deinit()
{
    return; // TODO
}

void espnow_block_until_controller_connected()
{
    while (1)
    {
        if (get_controller_connected()) {
            return;
        }
        else {
            vTaskDelay(ESPNOW_BLOCK_UNTIL_CONTROLLER_CONNECTED_LOOP_DELAY);
        }
    }
}

Payload_Controll_Commands espnow_get_latest_controll_commands()
{
    return get_control_commands();
}

EHeader espnow_header_create(EPacket_Type type, size_t packet_size, EPacket_Flags flags)
{
    assert(packet_size >= sizeof(EPacket));
    
    return (EHeader) {
        .type = type,
        .flags = flags,
        .packet_id = esp_random(),
        .payload_size = packet_size - sizeof(EPacket)
    };
}

bool espnow_send_packet_to_controller(EPacket* epkt)
{
    if (!get_controller_connected()) {
        return false;
    }

    size_t epkt_size = espnow_packet_size(epkt->header);
    ESPNow_Send_Info* e_send_info = espnow_send_info_acquire_raw(s_controller.mac_addr, epkt_size);
    if (e_send_info) {
        memcpy(&e_send_info->epkt, epkt, epkt_size);
        espnow_send_info_complete(e_send_info);
        return true;
    }
    return false;
}

bool espnow_send_msg_to_controller(const char* format, ...)
{
    if (!get_controller_connected()) {
        return false;
    }
    
    // Importantly we don't log any errors that might occur in this function!
    va_list args;

    // We dont use 'espnow_send_info_acquire/complete', but instead use a
    // statically allocated buffer and send the packet immediately.
    static char pkt_buffer[EPKT_MAX_SIZE];
    EPacket_D2C_Log* epkt_log = (EPacket_D2C_Log*) &pkt_buffer;

    va_start(args, format);
    int log_msg_size = vsnprintf(epkt_log->msg, EPKT_MAX_SIZE - sizeof(EPacket_D2C_Log), format, args) + 1; // + 1 because '\0' is not counted
    va_end(args);

    size_t packet_size = sizeof(EPacket_D2C_Log) + log_msg_size;
    epkt_log->header = espnow_header_create(EPKT_D2C_LOG, packet_size, 0);

    esp_err_t ret = esp_now_send(s_controller.mac_addr, (uint8_t*) epkt_log, packet_size);
    if (ret != ESP_OK) {
        LOG_ERROR("espnow_send_msg_to_controller: Failed to send espnow packet, ESP_ERR: %s", esp_err_to_name(ret));
        return false;
    }
    return true;
}
