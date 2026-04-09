#include "espnow_messaging.h"

// our components
#include "logging.h"

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_timer.h"
#include <stdatomic.h>

// The packets life in this queue, so it should be rather big
#define ESPNOW_RECEIVE_AND_SEND_INFO_RINGBUF_SIZE (1024 * 20)

// Packet acknowledging
#define MAX_ESPNOW_PACKETS_WAITING_FOR_ACK 10
const int espnow_wait_for_ack_resend_timeouts_ms[] = {10, 20, 40, 80, 160, 320, 640};
#define ESPNOW_WAIT_FOR_ACK_RESEND_TIMEOUTS_MS_LEN (sizeof(espnow_wait_for_ack_resend_timeouts_ms) / sizeof(int))

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

typedef enum {
    ESPNOW_RECEIVE_INFO = 0,
    ESPNOW_SEND_INFO,
} ESPNow_Info_Type;

typedef struct {
    ESPNow_Info_Type espnow_info_type;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    EPacket epkt;
} ESPNow_Receive_Info;

typedef struct {
    ESPNow_Info_Type espnow_info_type;
    bool occupied; // The array slot is occupied, used for the ack array.
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    int resend_count;
    esp_timer_handle_t pkt_resend_timer;
    EPacket epkt;
} ESPNow_Send_Info;

#define MAX_ESPNOW_SEND_INFO_SIZE (sizeof(ESPNow_Send_Info) - sizeof(EPacket) + EPKT_MAX_SIZE)

typedef struct
{
    bool in_peer_list;
    atomic_bool connected;
    int64_t time_since_last_c2d_packet_us;
    atomic_bool offline; // true, if time since last c2d packet is not within MIN_C2D_SEND_INTERVAL_MS
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

// Contains incoming and to-be-sent packets.
// We use a unified send and recive queue such that a single task can handle both.
// If we were to have multiple queues for a single task, then 'only waking up when there is work'
// becomes difficult, because we can't just block on a queue.
static RingbufHandle_t s_espnow_receive_and_send_info_ringbuf = NULL;

// Waiting packets (for acknowledgement)
static uint8_t    s_espnow_send_infos_waiting_for_ack[MAX_ESPNOW_PACKETS_WAITING_FOR_ACK][MAX_ESPNOW_SEND_INFO_SIZE] = {0};
static EPacket_ID s_espnow_send_infos_waiting_for_ack_packet_id[MAX_ESPNOW_PACKETS_WAITING_FOR_ACK] = {0}; // packet_id == 0 means the slot is empty
static SemaphoreHandle_t s_espnow_send_infos_waiting_for_ack_packet_id_mutex = NULL;

// Advertisement timer, sends an advertisement every so often if we are not connected to a controller
static esp_timer_handle_t periodic_advertise_timer = NULL;

static ESPNow_Send_Info* find_send_info_from_packet_id_in_wait_for_ack_array(EPacket_ID target_packet_id)
{
    xSemaphoreTake(s_espnow_send_infos_waiting_for_ack_packet_id_mutex, portMAX_DELAY);

    ESPNow_Send_Info* ret = NULL;
    for (int i = 0; i < MAX_ESPNOW_PACKETS_WAITING_FOR_ACK; ++i)
    {
        EPacket_ID packet_id = s_espnow_send_infos_waiting_for_ack_packet_id[i];
        if (packet_id == target_packet_id) {
            ret = s_espnow_send_infos_waiting_for_ack[i];
            break;
        }
    }
    
    xSemaphoreGive(s_espnow_send_infos_waiting_for_ack_packet_id_mutex);
    return ret;
}

static ESPNow_Send_Info* reserve_empty_slot_in_wait_for_ack_array(EPacket_ID packet_id)
{
    xSemaphoreTake(s_espnow_send_infos_waiting_for_ack_packet_id_mutex, portMAX_DELAY);

    ESPNow_Send_Info* empty_slot = NULL;
    for (int i = 0; i < MAX_ESPNOW_PACKETS_WAITING_FOR_ACK; ++i)
    {
        EPacket_ID packet_id = s_espnow_send_infos_waiting_for_ack_packet_id[i];
        if (packet_id == 0) {
            s_espnow_send_infos_waiting_for_ack_packet_id[i] = packet_id;
            empty_slot = s_espnow_send_infos_waiting_for_ack[i];
            break;
        }
    }
    
    xSemaphoreGive(s_espnow_send_infos_waiting_for_ack_packet_id_mutex);
    
    return empty_slot;
}

static void free_send_info_in_wait_for_ack_array(ESPNow_Send_Info* e_send_info)
{
    // Calculate index into 's_espnow_send_infos_waiting_for_ack' from pointer
    int idx = (s_espnow_send_infos_waiting_for_ack - e_send_info_size) / MAX_ESPNOW_SEND_INFO_SIZE;
    assert(s_espnow_send_infos_waiting_for_ack - e_send_info_size >= 0);
    assert((s_espnow_send_infos_waiting_for_ack - e_send_info_size) % MAX_ESPNOW_SEND_INFO_SIZE == 0);
    assert(idx >= 0 && idx < MAX_ESPNOW_PACKETS_WAITING_FOR_ACK);

    memset(e_send_info, 0, MAX_ESPNOW_SEND_INFO_SIZE);

    xSemaphoreTake(s_espnow_send_infos_waiting_for_ack_packet_id_mutex, portMAX_DELAY);
    s_espnow_send_infos_waiting_for_ack_packet_id[idx] = 0;
    xSemaphoreGive(s_espnow_send_infos_waiting_for_ack_packet_id_mutex);
}

static bool get_controller_connected()
{
    return atomic_load(&s_controller.connected);
}

static void set_controller_connected(bool connected)
{
    atomic_store(&s_controller.connected, connected);
}

static bool get_controller_offline()
{
    return atomic_load(&s_controller.offline);
}

static void set_controller_offline(bool offline)
{
    atomic_store(&s_controller.offline, offline);
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
    UBaseType_t ret = xRingbufferSendAcquire(s_espnow_receive_and_send_info_ringbuf,
                                             (void**) &e_recv_info,
                                             sizeof(ESPNow_Receive_Info) - sizeof(EPacket) + length,
                                             ESPNOW_RECEIVE_INFO_RINGBUF_MAX_TICKS_TO_WAIT);

    if (ret != pdTRUE) {
        CONTR_LOG_DEBUG("espnow_receive_callback: Creating espnow receive info in ringbuffer failed. The ringbuffer might be full.");
        return;
    }

    e_recv_info->espnow_info_type = ESPNOW_RECEIVE_INFO;
    memcpy(e_recv_info->mac_addr, recv_info->src_addr, ESP_NOW_ETH_ALEN);
    memcpy(&e_recv_info->epkt, data, length);
    // We should probably just remove the packet from the ringbuffer if this fails, but not sure if it ever would
    // do that in case this assert ever triggers.
    assert(e_recv_info->epkt.header.payload_size == length - sizeof(EPacket));

    ret = xRingbufferSendComplete(s_espnow_receive_and_send_info_ringbuf, (void*) e_recv_info);
    if (ret != pdTRUE) {
        CONTR_LOG_DEBUG("espnow_receive_callback: Completing espnow receive info in ringbuffer failed.");
    }
}

static size_t espnow_packet_size(EHeader e_header) { return sizeof(EHeader) + e_header.payload_size; }

static ESPNow_Send_Info* espnow_send_info_acquire_raw(const uint8_t* dest_mac_addr, size_t packet_size)
{
    size_t e_send_info_size = sizeof(ESPNow_Send_Info) - sizeof(EPacket) + packet_size;
    ESPNow_Send_Info* e_send_info = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_espnow_receive_and_send_info_ringbuf,
                                             (void**) &e_send_info,
                                             e_send_info_size,
                                             SERIAL_SEND_INFO_RINGBUF_MAX_TICKS_TO_WAIT);

    if (ret != pdTRUE) {
        LOG_DEBUG("espnow_send_info_acquire_raw: Creating espnow send info in ringbuffer failed. The ringbuffer might be full.");
        return NULL;
    }

    memset(e_send_info, 0, e_send_info_size);
    e_send_info->espnow_info_type = ESPNOW_SEND_INFO;
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
    UBaseType_t ret = xRingbufferSendComplete(s_espnow_receive_and_send_info_ringbuf, (void*) e_send_info);
    
    if (ret != pdTRUE) {
        LOG_DEBUG("espnow_send_info_complete: Completing espnow send info in ringbuffer failed.");
    }
}

static void process_espnow_packet_acknowledge(const ESPNow_Send_Info* e_send_info)
{
    return; // Currently there are no D2C packets that require ack
}

static void process_espnow_packet_failed_acknowledge(const ESPNow_Send_Info* e_send_info)
{
    return; // Currently there are no D2C packets that require ack
}

static void send_advertisement(void* arg)
{
    ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(broadcast_mac, EPKT_D2C_ADVERTISE, sizeof(EPacket_D2C_Advertise), 0);
    if (e_send_info) {
        EPacket_D2C_Advertise* epkt_d2c_ad = (EPacket_D2C_Advertise*) &e_send_info->epkt;
        memcpy(epkt_d2c_ad->drone_name, drone_name, MAX_DRONE_NAME_LENGTH);
        espnow_send_info_complete(e_send_info);
    }
}

static void start_periodic_advertise_timer()
{
    const esp_timer_create_args_t periodic_advertise_timer_args = {
        .callback = send_advertisement,
        .arg = NULL,
        .name = "periodic_advertise_timer",
    };

    esp_err_t ret = esp_timer_create(&periodic_advertise_timer_args, &s_periodic_advertise_timer);
    if (ret == ESP_OK) {
        ret = esp_timer_start_periodic(s_periodic_advertise_timer, ADVERTISE_COOLDOWN_MS * 1000);
        if (ret != ESP_OK) {
            CONTR_LOG_ERROR("start_periodic_advertise_timer: esp_timer_start_periodic failed, ESP_ERR: %s", esp_err_to_name(ret));
        }
    }
    else {
        CONTR_LOG_ERROR("start_periodic_advertise_timer: esp_timer_create failed, ESP_ERR: %s", esp_err_to_name(ret));
    }
}

static void stop_periodic_advertise_timer()
{
    esp_err_t ret = esp_timer_stop(s_periodic_advertise_timer);
    if (ret != ESP_OK) {
        CONTR_LOG_DEBUG("stop_periodic_advertise_timer: esp_timer_stop failed, ESP_ERR: %s", esp_err_to_name(ret));
    }    
}

static void process_incoming_espnow_packet(const ESPNow_Receive_Info* e_recv_info)
{
    const EPacket* epkt = &e_recv_info->epkt;

    bool from_controller = (memcmp(e_recv_info->mac_addr, s_controller.mac_addr, ESP_NOW_ETH_ALEN) == 0);

    // Update 'time_since_last_c2d_packet_us'
    if (from_controller) {
        s_controller.time_since_last_c2d_packet_us = esp_timer_get_time();
    }

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

        stop_periodic_advertise_timer(); // Stop advertising when connected
    }
    
    // If we need to send an ack, we should do that after potentially receiving EPKT_C2D_CONNECT,
    // but before sending any other packets (as soon after receiving as possible)
    if (epkt->header.flags & EPKT_FLAG_REQUIRE_ACK)
    {
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

        start_periodic_advertise_timer(); // Start advertising again when disconnected
        
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

        esp_err_t ret = esp_timer_stop(e_send_info->pkt_resend_timer);
        if (ret != ESP_OK) {
            CONTR_LOG_DEBUG("process_incoming_espnow_packet: esp_timer_stop failed, ESP_ERR: %s", esp_err_to_name(ret));
        }
        
        esp_err_t ret = esp_timer_delete(e_send_info->pkt_resend_timer);
        if (ret != ESP_OK) {
            CONTR_LOG_DEBUG("process_incoming_espnow_packet: esp_timer_delete failed, ESP_ERR: %s", esp_err_to_name(ret));
        }
            
        ESPNow_Send_Info* e_send_info = find_send_info_from_packet_id_in_wait_for_ack_array(acknowledged_packet_id);
        if (e_send_info) {
            process_espnow_packet_acknowledge(e_send_info);
            free_send_info_in_wait_for_ack_array(e_send_info);
        }
        else {
            CONTR_LOG_DEBUG("Received an EPKT_C2D_ACK_RECEIVE for packet '%d', but this packet "
                            "was not in the 'wait for ack array'", acknowledged_packet_id);
        }

    } break;

    case EPKT_C2D_PING:
        break; // Do nothing except sending an ack

    case EPKT_C2D_RESTART: {
        esp_restart();
    } break;
        
    default:
        CONTR_LOG_DEBUG("Received ESP-Now Packet with unexpected type: '%d'", epkt->header.type);
    }
}

static void resend_packet_that_is_waiting_for_ack(void* arg);

static void start_resend_pkt_timer(ESPNow_Send_Info* e_send_info, int64_t timeout_us)
{
    const esp_timer_create_args_t pkt_resend_timer_args = {
        .callback = resend_packet_that_is_waiting_for_ack,
        .arg = e_send_info,
        .name = "pkt_resend_timer",
    };

    ret = esp_timer_create(&pkt_resend_timer_args, &e_send_info->pkt_resend_timer);
    if (ret == ESP_OK) {
        ret = esp_timer_start_once(e_send_info->pkt_resend_timer, timeout_us);
        if (ret != ESP_OK) {
            CONTR_LOG_DEBUG("start_resend_pkt_timer: 'esp_timer_start_once' failed, ESP_ERR: %s", esp_err_to_name(ret));
        }
    }
    else {
        CONTR_LOG_DEBUG("start_resend_pkt_timer: 'esp_timer_create' failed, ESP_ERR: %s", esp_err_to_name(ret));
    }
}

static void resend_packet_that_is_waiting_for_ack(void* arg)
{
    ESPNow_Send_Info* e_send_info = (ESPNow_Send_Info*) arg;

    esp_err_t ret = esp_now_send(e_send_info->mac_addr, (uint8_t*) &e_send_info->epkt, espnow_packet_size(e_send_info->epkt.header));
    if (ret == ESP_OK) {
        e_send_info->resend_count += 1;
    }
    else {
        LOG_DEBUG("resend_packet_that_is_waiting_for_ack: esp_now_send failed, ESP_ERR: %s", esp_err_to_name(ret));
    }

    if (e_send_info->resend_count >= ESPNOW_WAIT_FOR_ACK_RESEND_TIMEOUTS_MS_LEN)
    {
        process_espnow_packet_failed_acknowledge(e_send_info);
        free_send_info_in_wait_for_ack_array(e_send_info);
    }
    else {
        start_resend_pkt_timer(e_send_info, espnow_wait_for_ack_resend_timeouts_ms[e_send_info->resend_count] * 1000); // restart timer
    }
}

static void send_espnow_packet(ESPNow_Send_Info* e_send_info)
{
    EPacket* epkt = &e_send_info->epkt;

    esp_err_t ret = esp_now_send(e_send_info->mac_addr, (uint8_t*) epkt, espnow_packet_size(epkt->header));
    if (ret == ESP_OK)
    {
        if (epkt->header.flags & EPKT_FLAG_REQUIRE_ACK)
        {
            ESPNow_Send_Info* empty_slot = reserve_empty_slot_in_wait_for_ack_array(e_send_info->epkt.header.packet_id);
            if (empty_slot)
            {
                memcpy(empty_slot, e_send_info, e_send_info_size);
                start_resend_pkt_timer(empty_slot, espnow_wait_for_ack_resend_timeouts_ms[0] * 1000);
            }
            else {
                CONTR_LOG_DEBUG("espnow_messaging_task: Failed to add ESPNow Send Info to the 'wait for ack array', "
                                "because all '%d' slots are full", MAX_ESPNOW_PACKETS_WAITING_FOR_ACK);
            }
        }
    }
    else {
        LOG_DEBUG("send_all_queued_espnow_packets: esp_now_send failed, ESP_ERR: %s", esp_err_to_name(ret));
    }
}

// Main task
static void espnow_messaging_task(void*)
{    
    while (1)
    {
        int64_t time_us = esp_timer_get_time();

        bool controller_offline = time_us - s_controller.time_since_last_c2d_packet_us > MIN_C2D_SEND_INTERVAL_MS * 1000;
        set_controller_offline(controller_offline);

        size_t e_info_size;
        void* e_info = xRingbufferReceive(s_espnow_receive_and_send_info_ringbuf, &e_info_size, portMAX_DELAY);
        
        ESPNow_Info_Type e_info_type = *((ESPNow_Info_Type*) e_info);
        switch (e_info_type) {
            
        case ESPNOW_RECEIVE_INFO: {
            process_incoming_espnow_packet((ESPNow_Receive_Info*) e_info);
        } break;
            
        case ESPNOW_SEND_INFO: {
            send_espnow_packet((ESPNow_Send_Info*) e_info);
        } break;

        }

        vRingbufferReturnItem(s_espnow_receive_and_send_info_ringbuf, e_info);
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

    // Initialising state

    s_espnow_receive_and_send_info_ringbuf = xRingbufferCreate(ESPNOW_RECEIVE_INFO_RINGBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    
    s_espnow_send_infos_waiting_for_ack_occupied_mutex = xSemaphoreCreateMutex();

    // Start advertise timer
    start_periodic_advertise_timer();

    // Starting the espnow messaging task
    
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

void espnow_get_latest_controll_commands(Payload_Controll_Commands* out)
{
    *out = get_control_commands();
}

bool espnow_controller_connected()
{
    return get_controller_connected();
}

bool espnow_controller_online()
{
    return !get_controller_offline();
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

Espnow_Messaging_Ret espnow_send_packet_to_controller(EPacket* epkt)
{
    if (!get_controller_connected()) return ESPNOW_MSG_ERR_CONTROLLER_DISCONNECTED;
    if (get_controller_offline())    return ESPNOW_MSG_ERR_CONTROLLER_OFFLINE;

    size_t epkt_size = espnow_packet_size(epkt->header);
    ESPNow_Send_Info* e_send_info = espnow_send_info_acquire_raw(s_controller.mac_addr, epkt_size);
    if (e_send_info) {
        memcpy(&e_send_info->epkt, epkt, epkt_size);
        espnow_send_info_complete(e_send_info);
        return ESPNOW_MSG_ERR_ESP_NOW_SEND_FAIL;
    }
    return ESPNOW_MSG_OK;
}

Espnow_Messaging_Ret espnow_send_msg_to_controller(const char* format, ...)
{
    if (!get_controller_connected()) return ESPNOW_MSG_ERR_CONTROLLER_DISCONNECTED;
    if (get_controller_offline())    return ESPNOW_MSG_ERR_CONTROLLER_OFFLINE;
    
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
        return ESPNOW_MSG_ERR_ESP_NOW_SEND_FAIL;
    }
    return ESPNOW_MSG_OK;
}
