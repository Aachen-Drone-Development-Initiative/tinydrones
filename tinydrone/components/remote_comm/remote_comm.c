#include "remote_comm.h"

#include <string.h>
#include <stdarg.h>
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_log.h"

// The packets life in this queue, so it should be rather big
#define EVENT_RINGBUF_SIZE (1024 * 20)

// Packet acknowledging
#define MAX_ESPNOW_PACKETS_WAITING_FOR_ACK 10
const int espnow_wait_for_ack_resend_timeouts_ms[] = {20, 40, 80, 160, 320};
#define ESPNOW_WAIT_FOR_ACK_RESEND_TIMEOUTS_MS_LEN (sizeof(espnow_wait_for_ack_resend_timeouts_ms) / sizeof(int))

// Max ticks to wait when trying to acquire memory from the event ringbuffer.
// The event ringbuffer is the main ringbuffer of the espnow_messaing task.
// Keeping this at 0 lets us see (through error messages) if the ringbuffer is too small,
// otherwise we would just see weird latency issues.

// This needs to be 0 ticks, because the main task should never block on the ringbuffer,
// since it would block itself from freeing the ringbuffer.
#define EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_MAIN_TASK 0

// Any other task that wants to push something to the ringbuffer should just block
// until this task has freed something from the ringbuffer.
#define EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_OTHER_TASK portMAX_DELAY

#define ESPNOW_BLOCK_UNTIL_CONTROLLER_CONNECTED_LOOP_DELAY pdMS_TO_TICKS(100)

// Repeating events cooldown
#define ADVERTISE_COOLDOWN_MS 1000

// ESP-Now/Wifi settings
#define ESPNOW_WIFI_MODE WIFI_MODE_STA // The drone should be in station mode (STA), because it connects to the controllers access point (AP).
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA // We want to use the station interface (IF).
#define ESPNOW_SEND_COOLDOWN_WHEN_ESPNOW_ERR_OUT_OF_MEM pdMS_TO_TICKS(100)

#define REMOTE_COMM_TAG "REM_COMM"

#define LOG_ERROR(format, ...)                      \
    do {                                            \
        ESP_LOGE(REMOTE_COMM_TAG, format, ##__VA_ARGS__);  \
    } while (0);

// Also send the error message to the controller
#define CONTR_LOG_ERROR(format, ...)                                    \
    do {                                                                \
        remote_comm_printf("ERROR (TINYDRONE): " format "\n", ##__VA_ARGS__); \
        ESP_LOGE(REMOTE_COMM_TAG, format, ##__VA_ARGS__);                      \
    } while (0);


#define TINYDRONE_ENABLE_DEBUG_LOGS 1

#if TINYDRONE_ENABLE_DEBUG_LOGS

#define LOG_DEBUG(format, ...)                      \
    do {                                            \
        ESP_LOGI(REMOTE_COMM_TAG, format, ##__VA_ARGS__);  \
    } while (0);

// Also send the debug message to the controller
#define CONTR_LOG_DEBUG(format, ...)                                    \
    do {                                                                \
        remote_comm_printf("DEBUG (TINYDRONE): " format "\n", ##__VA_ARGS__); \
        ESP_LOGI(REMOTE_COMM_TAG, format, ##__VA_ARGS__);                      \
    } while (0);

#else

#define LOG_DEBUG(...)
#define CONTR_LOG_DEBUG(...)

#endif

// Sending to this 'mac address' means sending to all ESPs in range.
static const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// TODO: This should not be hardcoded obviously
static const char drone_name[DRONE_NAME_LENGTH] = "tiny1";

typedef enum {
    ESPNOW_RECEIVE_EVENT = 0,
    ESPNOW_SEND_EVENT,
    ESPNOW_RESEND_UNACKNOWLEDGED_EVENT,
} Event_Type;

typedef struct {
    Event_Type event_type;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    EPacket epkt;
} ESPNow_Receive_Event;

typedef struct {
    Event_Type event_type;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    int resend_count;
    esp_timer_handle_t pkt_resend_timer;
    EPacket epkt;
} ESPNow_Send_Event;

typedef struct {
    Event_Type event_type;
    ESPNow_Send_Event* e_send_event; // Pointer to static memory buffer (wait-for-ack array)
} ESPNow_Resend_Unacknowledged_Event;

#define MAX_ESPNOW_SEND_EVENT_SIZE (sizeof(ESPNow_Send_Event) - sizeof(EPacket) + EPKT_MAX_SIZE)

typedef struct {
    bool in_peer_list;
    atomic_bool connected;
    atomic_int_fast64_t time_since_last_c2d_packet_us;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    
    // Latest controll commands
    atomic_int pitch;
    atomic_int yaw;
    atomic_int roll;
    atomic_int thrust;
    atomic_int flags;
} Controller_Info;

// Info about the currently connected controller
static Controller_Info s_controller = {0};

// Contains incoming, to-be-sent, and to-be-resent packets.
// We use a unified queue, so that the main task can simply work through everything linearly.
// And stop/resume with minimal latency, because we don't have to poll multiple queues.
// Otherwise we would have to coordinate multiple tasks which is much more difficult.
static RingbufHandle_t s_event_ringbuf = NULL;

// Static buffer for outgoing packets which need to be saved somewhere (for resending) until we receive their acknowledgement.
static uint8_t s_unacknowledged_espnow_send_events[MAX_ESPNOW_PACKETS_WAITING_FOR_ACK][MAX_ESPNOW_SEND_EVENT_SIZE] = {0};

// Advertisement timer, sends an advertisement every so often if we are not connected to a controller.
static esp_timer_handle_t s_periodic_advertise_timer = NULL;

static int64_t s_time_of_last_espnow_err_out_of_memory_us = 0;

static remote_comm_update_parameters_callback_t s_update_parameters_callback = NULL;

static ESPNow_Send_Event* get_unacknowledged_espnow_send_event_from_packet_id(EPacket_ID target_packet_id)
{
    for (int i = 0; i < MAX_ESPNOW_PACKETS_WAITING_FOR_ACK; ++i) {
        ESPNow_Send_Event* e_send_event = (ESPNow_Send_Event*) &s_unacknowledged_espnow_send_events[i];
        if (e_send_event->epkt.header.packet_id == target_packet_id) {
            return e_send_event;
        }
    }
    
    return NULL;
}

static ESPNow_Send_Event* acquire_unacknowledged_espnow_send_event()
{
    return get_unacknowledged_espnow_send_event_from_packet_id(0); // 'packet_id == 0' means the slot is empty
}

static void release_unacknowledged_espnow_send_event(ESPNow_Send_Event* e_send_event)
{
    ptrdiff_t relative_ptr = (uint8_t*) e_send_event - &s_unacknowledged_espnow_send_events[0][0];
    int idx = (int) (relative_ptr / MAX_ESPNOW_SEND_EVENT_SIZE);

    bool e_send_event_from_wait_for_ack_buffer = (idx >= 0 && idx < MAX_ESPNOW_PACKETS_WAITING_FOR_ACK);

    if (e_send_event_from_wait_for_ack_buffer) {
        if (relative_ptr % MAX_ESPNOW_SEND_EVENT_SIZE != 0) {
            CONTR_LOG_ERROR("release_unacknowledged_espnow_send_event: relative_ptr % MAX_ESPNOW_SEND_EVENT_SIZE != 0");
            return;
        }
        memset(e_send_event, 0, MAX_ESPNOW_SEND_EVENT_SIZE);
    }
}

static bool get_controller_connected()
{
    return atomic_load(&s_controller.connected);
}

static void set_controller_connected(bool connected)
{
    atomic_store(&s_controller.connected, connected);
}

static int64_t get_time_since_last_c2d_packet_us()
{
    return atomic_load(&s_controller.time_since_last_c2d_packet_us);
}

static void set_time_since_last_c2d_packet_us(int64_t time_us)
{
    atomic_store(&s_controller.time_since_last_c2d_packet_us, time_us);
}

static bool is_controller_offline()
{
    int64_t time_us = esp_timer_get_time();
    bool controller_offline = time_us - get_time_since_last_c2d_packet_us() > MIN_C2D_SEND_INTERVAL_MS * 1000;
    return controller_offline;
}

static Payload_Controll_Commands get_control_commands()
{
    return (Payload_Controll_Commands) {
        .pitch  = atomic_load(&s_controller.pitch),
        .yaw    = atomic_load(&s_controller.yaw),
        .roll   = atomic_load(&s_controller.roll),
        .thrust = atomic_load(&s_controller.thrust),
        .flags  = atomic_load(&s_controller.flags),
    };
}

static void set_control_commands(Payload_Controll_Commands ctrl_cmds)
{
    atomic_store(&s_controller.pitch,  ctrl_cmds.pitch);
    atomic_store(&s_controller.yaw,    ctrl_cmds.yaw);
    atomic_store(&s_controller.roll,   ctrl_cmds.roll);
    atomic_store(&s_controller.thrust, ctrl_cmds.thrust);
    atomic_store(&s_controller.flags,  ctrl_cmds.flags);
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

    // Make sure the packet passes some sanit checks
    EPacket* epkt = (EPacket*) data;
    if ((epkt->header.payload_size != length - sizeof(EPacket)) ||
        (epkt->header.type >= 64) ||
        (epkt->header.packet_id == 0))
    {
        CONTR_LOG_DEBUG("espnow_receive_callback: Received a corrupted ESP-Now Packet");
        return;
    }

    ESPNow_Receive_Event* e_recv_event = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_event_ringbuf,
                                             (void**) &e_recv_event,
                                             sizeof(ESPNow_Receive_Event) - sizeof(EPacket) + length,
                                             EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_OTHER_TASK);

    if (ret != pdTRUE) {
        CONTR_LOG_ERROR("espnow_receive_callback: Creating receive event in ringbuffer failed. The ringbuffer might be full.");
        return;
    }

    e_recv_event->event_type = ESPNOW_RECEIVE_EVENT;
    memcpy(e_recv_event->mac_addr, recv_info->src_addr, ESP_NOW_ETH_ALEN);
    memcpy(&e_recv_event->epkt, data, length);

    ret = xRingbufferSendComplete(s_event_ringbuf, (void*) e_recv_event);
    if (ret != pdTRUE) {
        CONTR_LOG_DEBUG("espnow_receive_callback: Completing receive event in ringbuffer failed.");
    }
}

static size_t espnow_packet_size(EHeader e_header) { return sizeof(EHeader) + e_header.payload_size; }

static ESPNow_Send_Event* espnow_send_event_acquire_raw(const uint8_t* dest_mac_addr, size_t packet_size, TickType_t ticks_to_wait)
{
    size_t e_send_event_size = sizeof(ESPNow_Send_Event) - sizeof(EPacket) + packet_size;
    ESPNow_Send_Event* e_send_event = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_event_ringbuf,
                                             (void**) &e_send_event,
                                             e_send_event_size,
                                             ticks_to_wait);

    if (ret != pdTRUE) {
        CONTR_LOG_ERROR("espnow_send_event_acquire_raw: Creating send event in ringbuffer failed. The ringbuffer might be full.");
        return NULL;
    }

    memset(e_send_event, 0, e_send_event_size);
    e_send_event->event_type = ESPNOW_SEND_EVENT;
    memcpy(e_send_event->mac_addr, dest_mac_addr, ESP_NOW_ETH_ALEN);

    return e_send_event;
}

static ESPNow_Send_Event* espnow_send_event_acquire(const uint8_t* dest_mac_addr,
                                                    EPacket_Type type,
                                                    size_t packet_size,
                                                    EPacket_Flags flags)
{
    ESPNow_Send_Event* e_send_event = espnow_send_event_acquire_raw(dest_mac_addr, packet_size, EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_MAIN_TASK);
    if (e_send_event) {
        e_send_event->epkt.header = remote_comm_create_packet_header(type, packet_size, flags);
    }

    return e_send_event;
}

static void espnow_send_event_complete(const ESPNow_Send_Event* e_send_event)
{
    UBaseType_t ret = xRingbufferSendComplete(s_event_ringbuf, (void*) e_send_event);
    
    if (ret != pdTRUE) {
        CONTR_LOG_ERROR("espnow_send_event_complete: Completing send event in ringbuffer failed.");
    }
}

static void queue_advertisement_packet_for_send(void* arg)
{
    ESPNow_Send_Event* e_send_event = espnow_send_event_acquire(broadcast_mac, EPKT_D2C_ADVERTISE, sizeof(EPacket_D2C_Advertise), 0);
    if (e_send_event) {
        EPacket_D2C_Advertise* epkt_d2c_ad = (EPacket_D2C_Advertise*) &e_send_event->epkt;
        memcpy(epkt_d2c_ad->drone_name, drone_name, DRONE_NAME_LENGTH);
        espnow_send_event_complete(e_send_event);
    }
}

static void start_periodic_advertise_timer()
{
    esp_err_t ret = esp_timer_start_periodic(s_periodic_advertise_timer, ADVERTISE_COOLDOWN_MS * 1000);
    if (ret != ESP_OK) {
        CONTR_LOG_ERROR("start_periodic_advertise_timer: esp_timer_start_periodic failed, ESP_ERR: %s", esp_err_to_name(ret));
    }
}

static void stop_periodic_advertise_timer()
{
    esp_err_t ret = esp_timer_stop(s_periodic_advertise_timer);
    if (ret != ESP_OK) {
        CONTR_LOG_ERROR("stop_periodic_advertise_timer: esp_timer_stop failed, ESP_ERR: %s", esp_err_to_name(ret));
    }
}

static void queue_unacknowledged_espnow_packet_for_resend(void* arg);

static bool create_and_start_resend_pkt_timer(ESPNow_Send_Event* e_send_event, int64_t timeout_us)
{
    const esp_timer_create_args_t pkt_resend_timer_args = {
        .callback = queue_unacknowledged_espnow_packet_for_resend,
        .arg = e_send_event,
        .name = "pkt_resend_timer",
    };

    esp_err_t ret = esp_timer_create(&pkt_resend_timer_args, &e_send_event->pkt_resend_timer);
    if (ret == ESP_OK) {
        ret = esp_timer_start_once(e_send_event->pkt_resend_timer, timeout_us);
        if (ret != ESP_OK) {
            CONTR_LOG_ERROR("create_and_start_resend_pkt_timer: 'esp_timer_start_once' failed, ESP_ERR: %s", esp_err_to_name(ret));
            return false;
        }
        return true;
    }
    else {
        CONTR_LOG_ERROR("create_and_start_resend_pkt_timer: 'esp_timer_create' failed, ESP_ERR: %s", esp_err_to_name(ret));
        return false;
    }
}

static void delete_and_stop_resend_pkt_timer(ESPNow_Send_Event* e_send_event)
{
    if (e_send_event->pkt_resend_timer == NULL) {
        return;
    }
    
    esp_err_t ret = esp_timer_stop(e_send_event->pkt_resend_timer);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        CONTR_LOG_ERROR("delete_and_stop_resend_pkt_timer: esp_timer_stop failed, ESP_ERR: %s", esp_err_to_name(ret));
    }
        
    ret = esp_timer_delete(e_send_event->pkt_resend_timer);
    if (ret != ESP_OK) {
        CONTR_LOG_ERROR("delete_and_stop_resend_pkt_timer: esp_timer_delete failed, ESP_ERR: %s", esp_err_to_name(ret));
    }

    e_send_event->pkt_resend_timer = NULL;
}

static void handle_espnow_packet_acknowledge(ESPNow_Send_Event* e_send_event)
{
    delete_and_stop_resend_pkt_timer(e_send_event);

    const EPacket* epkt = &e_send_event->epkt;

    switch (epkt->header.type) {
        // Currently there are no D2C packets that require ack
    default:
        CONTR_LOG_DEBUG("Received acknowledge for ESP-Now Packet with unexpected type: '%d'", epkt->header.type);
    }

    release_unacknowledged_espnow_send_event(e_send_event);
}

static void handle_espnow_packet_failed_acknowledge(ESPNow_Send_Event* e_send_event)
{
    delete_and_stop_resend_pkt_timer(e_send_event);

    const EPacket* epkt = &e_send_event->epkt;

    switch (epkt->header.type) {
        // Currently there are no D2C packets that require ack
    default:
        CONTR_LOG_DEBUG("Acknowledge failed for ESP-Now Packet with unexpected type: '%d'", epkt->header.type);
    }

    release_unacknowledged_espnow_send_event(e_send_event);
}

static void queue_unacknowledged_espnow_packet_for_resend(void* arg)
{
    ESPNow_Send_Event* e_send_event = (ESPNow_Send_Event*) arg;

    ESPNow_Resend_Unacknowledged_Event* e_resend_unack_event = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_event_ringbuf,
                                             (void**) &e_resend_unack_event,
                                             sizeof(ESPNow_Resend_Unacknowledged_Event),
                                             EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_OTHER_TASK);

    if (ret != pdTRUE) {
        CONTR_LOG_ERROR("queue_unacknowledged_espnow_packet_for_resend: 'xRingbufferSendAcquire()' failed. The ringbuffer might be full.");
        return;
    }

    e_resend_unack_event->event_type = ESPNOW_RESEND_UNACKNOWLEDGED_EVENT;
    e_resend_unack_event->e_send_event = e_send_event;

    ret = xRingbufferSendComplete(s_event_ringbuf, (void*) e_resend_unack_event);
    if (ret != pdTRUE) {
        CONTR_LOG_ERROR("queue_unacknowledged_espnow_packet_for_resend: 'xRingbufferSendComplete()' failed.");
    }
}

static void handle_incoming_espnow_packet(const ESPNow_Receive_Event* e_recv_event)
{
    const EPacket* epkt = &e_recv_event->epkt;

    bool from_controller = (memcmp(e_recv_event->mac_addr, s_controller.mac_addr, ESP_NOW_ETH_ALEN) == 0);

    if (from_controller) {
        set_time_since_last_c2d_packet_us(esp_timer_get_time());
    }

    if (!from_controller && epkt->header.type != EPKT_C2D_CONNECT)
    {
        if (epkt->header.type != EPKT_D2C_ADVERTISE) {
            CONTR_LOG_ERROR("handle_incoming_espnow_packet: Received an unexpected packet of type '%d' from unknow sender.", epkt->header.type);
        }
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
                CONTR_LOG_ERROR("handle_incoming_espnow_packet: Failed to delete Controller from peer list, ESP_ERR: %s", esp_err_to_name(ret));
            }
            s_controller.in_peer_list = false;
        }
        
        // Try to add the controller to the espnow peer list
        esp_now_peer_info_t peer = {0};
        memcpy(peer.peer_addr, e_recv_event->mac_addr, ESP_NOW_ETH_ALEN);
        peer.channel = 0;
        peer.ifidx = ESPNOW_WIFI_IF;
        esp_err_t ret = esp_now_add_peer(&peer);
        if (ret != ESP_OK) {
            CONTR_LOG_ERROR("handle_incoming_espnow_packet: Failed to add Controller to peer list, ESP_ERR: %s", esp_err_to_name(ret));
            return;
        }

        s_controller.in_peer_list = true;
        set_controller_connected(true);
        memcpy(s_controller.mac_addr, e_recv_event->mac_addr, ESP_NOW_ETH_ALEN);

        stop_periodic_advertise_timer(); // Stop advertising when connected
    }
    
    // If we need to send an ack, we should do that after potentially receiving EPKT_C2D_CONNECT,
    // but before sending any other packets (as soon after receiving as possible)
    if (epkt->header.flags & EPKT_FLAG_REQUIRE_ACK)
    {
        ESPNow_Send_Event* e_send_event = espnow_send_event_acquire(s_controller.mac_addr, EPKT_D2C_ACK_RECEIVE, sizeof(EPacket_D2C_Ack_Receive), 0);
        if (e_send_event) {
            EPacket_D2C_Ack_Receive* epkt_d2c_ack = (EPacket_D2C_Ack_Receive*) &e_send_event->epkt;
            epkt_d2c_ack->acknowledged_packet_id = epkt->header.packet_id;
            espnow_send_event_complete(e_send_event);
        }
    }
    
    switch (epkt->header.type) {

    case EPKT_C2D_CONNECT:
        break; // this was already handeled above
        
    case EPKT_C2D_DISCONNECT:
    {
        if (!get_controller_connected()) break;
        
        set_control_commands((Payload_Controll_Commands) {0});
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

        ESPNow_Send_Event* e_send_event = get_unacknowledged_espnow_send_event_from_packet_id(acknowledged_packet_id);
            
        if (e_send_event) {
            handle_espnow_packet_acknowledge(e_send_event);
        }
        else {
            CONTR_LOG_DEBUG("Received an EPKT_C2D_ACK_RECEIVE for packet '%d', but this packet "
                            "was not in the 'wait for ack array'", acknowledged_packet_id);
        }

    } break;

    case EPKT_C2D_PING: break; // Do nothing except sending an ack

    case EPKT_C2D_RESTART: {
        esp_restart();
    } break;

    case EPKT_C2D_PARAMETERS:
    {
        if (s_update_parameters_callback) {
            s_update_parameters_callback((EPacket_C2D_Parameters*) epkt);
        }
        
    } break;
        
    default:
        CONTR_LOG_DEBUG("Received ESP-Now Packet with unexpected type: '%d'", epkt->header.type);
    }
}

static bool espnow_send_bytes(uint8_t* mac_addr, uint8_t* bytes, size_t size)
{
    int64_t time_us = esp_timer_get_time();
    if (time_us - s_time_of_last_espnow_err_out_of_memory_us < ESPNOW_SEND_COOLDOWN_WHEN_ESPNOW_ERR_OUT_OF_MEM) {
        return false;
    }
    
    esp_err_t ret = esp_now_send(mac_addr, bytes, size);
    if (ret != ESP_OK) {
        LOG_ERROR("espnow_send_bytes: esp_now_send failed, ESP_ERR: %s", esp_err_to_name(ret));
        if (ret == ESP_ERR_ESPNOW_NO_MEM) {
            s_time_of_last_espnow_err_out_of_memory_us = esp_timer_get_time();
        }
        return false;
    }

    return true;
}

static void send_espnow_packet(ESPNow_Send_Event* e_send_event)
{
    EPacket* epkt = &e_send_event->epkt;

    espnow_send_bytes(e_send_event->mac_addr, (uint8_t*) epkt, espnow_packet_size(epkt->header));
    
    if (epkt->header.flags & EPKT_FLAG_REQUIRE_ACK)
    {
        // We copy the e_send_event into some static buffer which we first need to acquire
        ESPNow_Send_Event* e_send_event_copy = acquire_unacknowledged_espnow_send_event();
        if (e_send_event_copy)
        {
            size_t e_send_event_size = sizeof(ESPNow_Send_Event) + e_send_event->epkt.header.payload_size;
            memcpy(e_send_event_copy, e_send_event, e_send_event_size);
            bool success = create_and_start_resend_pkt_timer(e_send_event_copy, espnow_wait_for_ack_resend_timeouts_ms[0] * 1000);
            if (!success) {
                handle_espnow_packet_failed_acknowledge(e_send_event_copy);
            }
        }
        else {
            CONTR_LOG_ERROR("send_espnow_packet: Failed to find an empty slot for an unacknowledged ESPNow send event, "
                            "all (%d) slots are occupied", MAX_ESPNOW_PACKETS_WAITING_FOR_ACK);
        }
    }
}

static void resend_unacknowledged_espnow_packet(ESPNow_Resend_Unacknowledged_Event* e_resend_unack_event)
{
    ESPNow_Send_Event* e_send_event = e_resend_unack_event->e_send_event;
    EPacket* epkt = &e_send_event->epkt;

    esp_err_t ret = esp_now_send(e_send_event->mac_addr, (uint8_t*) epkt, espnow_packet_size(epkt->header));
    if (ret != ESP_OK) {
        LOG_ERROR("resend_unacknowledged_espnow_packet: esp_now_send failed, ESP_ERR: %s", esp_err_to_name(ret));
    }

    e_send_event->resend_count += 1;

    if (e_send_event->resend_count >= ESPNOW_WAIT_FOR_ACK_RESEND_TIMEOUTS_MS_LEN) {
        handle_espnow_packet_failed_acknowledge(e_send_event);
    }
    else {
        esp_err_t ret = esp_timer_start_once(e_send_event->pkt_resend_timer,
                                             espnow_wait_for_ack_resend_timeouts_ms[e_send_event->resend_count] * 1000);
        if (ret != ESP_OK) {
            LOG_ERROR("resend_unacknowledged_espnow_packet: esp_timer_start_once failed, ESP_ERR: %s", esp_err_to_name(ret));
            handle_espnow_packet_failed_acknowledge(e_send_event);
        }
    }
}

// Main task
static void remote_comm_task(void*)
{    
    while (1)
    {
        size_t event_size;
        void* event = xRingbufferReceive(s_event_ringbuf, &event_size, portMAX_DELAY);
        
        Event_Type event_type = *((Event_Type*) event);
        switch (event_type) {
            
        case ESPNOW_RECEIVE_EVENT: {
            handle_incoming_espnow_packet((ESPNow_Receive_Event*) event);
        } break;
            
        case ESPNOW_SEND_EVENT: {
            send_espnow_packet((ESPNow_Send_Event*) event);
        } break;

        case ESPNOW_RESEND_UNACKNOWLEDGED_EVENT: {
            resend_unacknowledged_espnow_packet((ESPNow_Resend_Unacknowledged_Event*) event);
        } break;

        }

        vRingbufferReturnItem(s_event_ringbuf, event);
    }

    // Delete this task
    vTaskDelete(NULL);
}


// Public API implementation


void remote_comm_init(const Remote_Comm_Config* config)
{
    // Init wifi
    
    assert(esp_netif_init() == ESP_OK);
    assert(esp_event_loop_create_default() == ESP_OK);
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    
    assert(esp_wifi_init(&cfg) == ESP_OK);
    assert(esp_wifi_set_mode(ESPNOW_WIFI_MODE) == ESP_OK);
    assert(esp_wifi_start() == ESP_OK);

    if (config->long_range_wifi) {
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

    s_event_ringbuf = xRingbufferCreate(EVENT_RINGBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    
    // Create advertise timer

    const esp_timer_create_args_t periodic_advertise_timer_args = {
        .callback = queue_advertisement_packet_for_send,
        .arg = NULL,
        .name = "periodic_advertise_timer",
    };

    assert(esp_timer_create(&periodic_advertise_timer_args, &s_periodic_advertise_timer) == ESP_OK);
    
    // And start the advertise timer
    start_periodic_advertise_timer();

    // Starting the espnow messaging task
    
    TaskHandle_t task_handle = NULL; // We don't keep the task handle, because we don't need it.
    xTaskCreate(remote_comm_task, "remote_comm_task", config->task_stack_size, NULL, config->task_priority, &task_handle);
    assert(task_handle);
}

void remote_comm_deinit()
{
    return; // TODO
}

void remote_comm_block_until_connected()
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

bool remote_comm_connected()
{
    return get_controller_connected();
}

bool remote_comm_online()
{
    return !is_controller_offline();
}

void remote_comm_get_latest_controll_commands(Payload_Controll_Commands* out)
{
    *out = get_control_commands();
}

EHeader remote_comm_create_packet_header(EPacket_Type type, size_t packet_size, EPacket_Flags flags)
{
    assert(packet_size >= sizeof(EPacket));

    EPacket_ID packet_id = esp_random();
    while (packet_id == 0) { // 0 is an invalid id
        packet_id = esp_random();
    }
    
    return (EHeader) {
        .type = type,
        .flags = flags,
        .packet_id = packet_id,
        .payload_size = packet_size - sizeof(EPacket)
    };
}

Remote_Comm_Ret remote_comm_send_packet(EPacket* epkt)
{
    if (!get_controller_connected()) return REMOTE_COMM_ERR_CONTROLLER_DISCONNECTED;
    if (is_controller_offline())     return REMOTE_COMM_ERR_CONTROLLER_OFFLINE;

    size_t epkt_size = espnow_packet_size(epkt->header);
    ESPNow_Send_Event* e_send_event = espnow_send_event_acquire_raw(s_controller.mac_addr, epkt_size, EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_OTHER_TASK);
    if (e_send_event) {
        memcpy(&e_send_event->epkt, epkt, epkt_size);
        espnow_send_event_complete(e_send_event);
        return REMOTE_COMM_OK;
    }
    return REMOTE_COMM_ERR_ESP_NOW_SEND_FAIL;
}

Remote_Comm_Ret remote_comm_printf(const char* format, ...)
{
    if (!get_controller_connected()) return REMOTE_COMM_ERR_CONTROLLER_DISCONNECTED;
    if (is_controller_offline())     return REMOTE_COMM_ERR_CONTROLLER_OFFLINE;
    
    // Importantly we don't log any errors that might occur in this function!
    va_list args;

    // We dont use 'espnow_send_event_acquire/complete', but instead use a
    // statically allocated buffer and send the packet immediately.
    static char pkt_buffer[EPKT_MAX_SIZE];
    EPacket_D2C_Msg* epkt_log = (EPacket_D2C_Msg*) &pkt_buffer;

    int max_log_msg_size = EPKT_MAX_SIZE - sizeof(EPacket_D2C_Msg);

    va_start(args, format);
    int log_msg_size = vsnprintf(epkt_log->msg, max_log_msg_size, format, args) + 1; // + 1 because '\0' is not counted
    va_end(args);

    if (log_msg_size > max_log_msg_size) {
        log_msg_size = max_log_msg_size;
    }

    size_t packet_size = sizeof(EPacket_D2C_Msg) + log_msg_size;
    epkt_log->header = remote_comm_create_packet_header(EPKT_D2C_MSG, packet_size, 0);

    esp_err_t ret = esp_now_send(s_controller.mac_addr, (uint8_t*) epkt_log, packet_size);
    if (ret != ESP_OK) {
        LOG_ERROR("remote_comm_printf: Failed to send espnow packet, ESP_ERR: %s", esp_err_to_name(ret));
        return REMOTE_COMM_ERR_ESP_NOW_SEND_FAIL;
    }
    return REMOTE_COMM_OK;
}

void remote_comm_set_update_parameters_callback(remote_comm_update_parameters_callback_t callback)
{
    // TODO (yuzeni) maybe make this thread safe with atomics or a mutex.
    s_update_parameters_callback = callback;
}

static inline size_t get_dtype_size(uint8_t dtype)
{
    switch (dtype) {
    case DTYPE_INT32:   return sizeof(int32_t);
    case DTYPE_INT64:   return sizeof(int64_t);
    case DTYPE_FLOAT32: return sizeof(float);
    }
    return 0;
}

bool remote_comm_expect_parameter(const EPacket_C2D_Parameters* epkt_params, const char* name, void* value_out)
{
    const Payload_Named_Value* params = epkt_params->parameters;
    int16_t param_count = epkt_params->parameter_count;
    
    for (int i = 0; i < param_count; ++i) {
        if (strncmp(params[i].name, name, NAMED_VALUE_NAME_LENGTH) == 0) {
            memcpy(value_out, &params[i].data, get_dtype_size(params[i].dtype));
            return true;
        }
    }
    return false;
}

Payload_Named_Value remote_comm_create_named_value(const char* name, uint8_t dtype, void* value_in)
{
    Payload_Named_Value named_value = {0};
    size_t dtype_size = get_dtype_size(dtype);
    size_t name_len = strnlen(name, NAMED_VALUE_NAME_LENGTH);

    named_value.dtype = dtype;
    memcpy(named_value.name, name, name_len);
    if (dtype_size > 0) {
        memcpy(&named_value.data, value_in, dtype_size);
    }

    return named_value;
}

Remote_Comm_Ret remote_comm_send_log_values(const Payload_Named_Value* log_values, int log_value_count)
{
    if (log_value_count < 0)                              return REMOTE_COMM_OK;
    if (log_value_count > 0 && log_values == NULL)        return REMOTE_COMM_ERR_ARG;
    if (log_value_count > EPKT_MAX_LOG_VALUES_PER_PACKET) return REMOTE_COMM_ERR_PACKET_TOO_LARGE;
    
    if (!get_controller_connected()) return REMOTE_COMM_ERR_CONTROLLER_DISCONNECTED;
    if (is_controller_offline())     return REMOTE_COMM_ERR_CONTROLLER_OFFLINE;

    size_t epkt_size = sizeof(EPacket_D2C_Log_Values) + log_value_count * sizeof(Payload_Named_Value);
    ESPNow_Send_Event* e_send_event = espnow_send_event_acquire_raw(s_controller.mac_addr, epkt_size, EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_OTHER_TASK);
    if (e_send_event)
    {
        EPacket_D2C_Log_Values* pkt_log_vals = (EPacket_D2C_Log_Values*) &e_send_event->epkt;
        pkt_log_vals->header = remote_comm_create_packet_header(EPKT_D2C_LOG_VALUES, epkt_size, 0);
        
        pkt_log_vals->log_value_count = log_value_count;
        memcpy(pkt_log_vals->log_values, log_values, log_value_count * sizeof(Payload_Named_Value));
        espnow_send_event_complete(e_send_event);
            
        return REMOTE_COMM_OK;
    }

    return REMOTE_COMM_ERR_ESP_NOW_SEND_FAIL;
}
