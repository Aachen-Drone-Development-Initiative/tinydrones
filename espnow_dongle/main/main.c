#include "espnow_packets.h"
#include "serial_packets.h"

#include <stdarg.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_mac.h"

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_select.h"

// Static array sizes
#define MAX_DRONES 128

// Buffer sizes
#define EVENT_RINGBUF_SIZE              (1024 * 20)
#define USB_SERIAL_JTAG_IN_BUFFER_SIZE  (1024 * 10)
#define USB_SERIAL_JTAG_OUT_BUFFER_SIZE (1024 * 10)

// Packet acknowledging
#define MAX_ESPNOW_PACKETS_WAITING_FOR_ACK 10
static const int s_espnow_wait_for_ack_resend_timeouts_ms[] = {20, 40, 80, 160, 320};
#define ESPNOW_WAIT_FOR_ACK_RESEND_TIMEOUTS_MS_LEN                      \
    (sizeof(s_espnow_wait_for_ack_resend_timeouts_ms) / sizeof(int))

// Ringbuffer waits
#define EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_MAIN_TASK  0
#define EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_OTHER_TASK portMAX_DELAY
#define EVENT_RINGBUF_RECEIVE_TICKS_TO_WAIT                portMAX_DELAY

// USB Serial waits
#define USB_SERIAL_WRITE_MAX_TICKS_TO_WAIT     pdMS_TO_TICKS(10)
#define USB_SERIAL_WRITE_LOG_MAX_TICKS_TO_WAIT pdMS_TO_TICKS(0)
#define USB_SERIAL_READ_MAX_TICKS_TO_WAIT      0

// ESP-Now settings
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#define ESPNOW_SEND_COOLDOWN_WHEN_ESPNOW_ERR_OUT_OF_MEM pdMS_TO_TICKS(100)

// Periodic connected-drone ping timer.
#define PING_CONNECTED_DRONE_INTERVAL_MS (MIN_C2D_SEND_INTERVAL_MS / 4)

static void serial_send_msg_to_master(const char* format, ...);

#define LOG_ERROR(format, ...) serial_send_msg_to_master("ERROR (DONGLE): " format "\n", ##__VA_ARGS__)

#define ENABLE_DEBUG_LOGS 1

#if ENABLE_DEBUG_LOGS
#    define LOG_DEBUG(format, ...) serial_send_msg_to_master("DEBUG (DONGLE): " format "\n", ##__VA_ARGS__)
#else
#    define LOG_DEBUG(...)
#endif

#define ASSERT(condition)                                               \
    do {                                                                \
        if (!(condition)) {                                             \
            LOG_ERROR("ASSERT FAILED (DONGLE):%s:%d (%s)", __FILE__, __LINE__, #condition); \
            abort();                                                    \
        }                                                               \
    } while (0)

typedef enum {
    ESPNOW_RECEIVE_EVENT = 0,
    ESPNOW_SEND_EVENT,
    ESPNOW_RESEND_UNACKNOWLEDGED_EVENT,
    SERIAL_RECEIVE_EVENT,
    SERIAL_SEND_EVENT,
    PING_CONNECTED_DRONES_EVENT,
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
    bool send_simple_response_to_master;
    SPacket_ID origin_serial_packet_id;
    SPacket_Type origin_serial_packet_type;
    esp_timer_handle_t pkt_resend_timer;
    EPacket epkt;
} ESPNow_Send_Event;

typedef struct {
    Event_Type event_type;
    ESPNow_Send_Event* e_send_event;
} ESPNow_Resend_Unacknowledged_Event;

typedef struct {
    Event_Type event_type;
    SPacket spkt;
} Serial_Receive_Event;

typedef struct {
    Event_Type event_type;
    SPacket spkt;
} Serial_Send_Event;

typedef struct {
    Event_Type event_type;
} Ping_Connected_Drones_Event;

#define MAX_ESPNOW_SEND_EVENT_SIZE (sizeof(ESPNow_Send_Event) - sizeof(EPacket) + EPKT_MAX_SIZE)

typedef struct {
    bool occupied;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    char drone_name[DRONE_NAME_LENGTH];
    bool connected;
    bool responding;
} Drone_Info;

static Drone_Info s_drone_infos[MAX_DRONES] = {0};

static RingbufHandle_t s_event_ringbuf = NULL;
static uint8_t s_unacknowledged_espnow_send_events[MAX_ESPNOW_PACKETS_WAITING_FOR_ACK][MAX_ESPNOW_SEND_EVENT_SIZE] = {0};

static uint8_t s_serial_rx_buffer[SPKT_MAX_SIZE] = {0};
static size_t s_serial_rx_buffer_len = 0;

static esp_timer_handle_t s_periodic_ping_timer = NULL;

static int64_t s_time_of_last_espnow_err_out_of_memory_us = 0;

static void espnow_send_callback(const esp_now_send_info_t *tx_info, esp_now_send_status_t status);
static void espnow_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int length);
static void usb_serial_jtag_select_notif_callback(usj_select_notif_t usb_serial_jtag_select_notif, BaseType_t *task_woken);
static void queue_ping_connected_drones_event_callback(void* arg);
static void queue_espnow_resend_unacknowledged_event_callback(void* arg);

static size_t espnow_packet_size(EHeader e_header) { return sizeof(EHeader) + e_header.payload_size; }
static size_t serial_packet_size(SHeader s_header) { return sizeof(SHeader) + s_header.payload_size; }

static int get_drone_idx_from_mac_addr(const uint8_t* mac_addr)
{
    for (int i = 0; i < MAX_DRONES; ++i) {
        if (memcmp(s_drone_infos[i].mac_addr, mac_addr, ESP_NOW_ETH_ALEN) == 0) {
            return i;
        }
    }

    LOG_ERROR("No drone available with MAC: '"MACSTR"'.", MAC2STR(mac_addr));
    return -1;
}

static int search_drone_idx_from_drone_name(const char* drone_name)
{
    for (int i = 0; i < MAX_DRONES; ++i) {
        if (memcmp(s_drone_infos[i].drone_name, drone_name, DRONE_NAME_LENGTH) == 0) {
            return i;
        }
    }
    
    return -1;
}

static int get_drone_idx_from_drone_name(const char* drone_name)
{
    int drone_idx = search_drone_idx_from_drone_name(drone_name);
    if (drone_idx < 0) {
        LOG_ERROR("No drone available with name: '%.*s'.", DRONE_NAME_LENGTH, drone_name);
    }
    
    return drone_idx;
}

static EHeader espnow_header_create(EPacket_Type type, size_t packet_size, EPacket_Flags flags)
{
    ASSERT(packet_size >= sizeof(EPacket));

    EPacket_ID packet_id = esp_random();
    while (packet_id == 0) {
        packet_id = esp_random();
    }

    return (EHeader) {
        .type = type,
        .flags = flags,
        .packet_id = packet_id,
        .payload_size = packet_size - sizeof(EPacket),
    };
}

static SHeader serial_header_create(SPacket_Type type, size_t packet_size)
{
    ASSERT(packet_size >= sizeof(SPacket));

    SPacket_ID packet_id = esp_random();
    while (packet_id == 0) {
        packet_id = esp_random();
    }

    return (SHeader) {
        .delim = SERIAL_DELIMITER,
        .type = type,
        .payload_size = packet_size - sizeof(SPacket),
        .packet_id = packet_id,
    };
}

static void usb_serial_jtag_init()
{
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = USB_SERIAL_JTAG_IN_BUFFER_SIZE,
        .tx_buffer_size = USB_SERIAL_JTAG_OUT_BUFFER_SIZE,
    };

    if (usb_serial_jtag_driver_install(&usb_serial_jtag_config) != ESP_OK) {
        printf("ERROR: init_usb_serial_jtag: usb_serial_jtag_driver_install failed, halting...");
        abort();
    }

    usb_serial_jtag_set_select_notif_callback(usb_serial_jtag_select_notif_callback);
}

static void nvs_init()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ASSERT(nvs_flash_erase() == ESP_OK);
        ret = nvs_flash_init();
    }
    ASSERT(ret == ESP_OK);
}

static void wifi_init(bool long_range_mode)
{
    ASSERT(esp_netif_init() == ESP_OK);
    ASSERT(esp_event_loop_create_default() == ESP_OK);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ASSERT(esp_wifi_init(&cfg) == ESP_OK);
    ASSERT(esp_wifi_set_mode(ESPNOW_WIFI_MODE) == ESP_OK);
    ASSERT(esp_wifi_start() == ESP_OK);

    if (long_range_mode) {
        ASSERT(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR)
               == ESP_OK);
    }
}

static void espnow_init()
{
    ASSERT(esp_now_init() == ESP_OK);
    ASSERT(esp_now_register_send_cb(espnow_send_callback) == ESP_OK);
    ASSERT(esp_now_register_recv_cb(espnow_receive_callback) == ESP_OK);

    esp_now_peer_info_t peer = {0};
    static const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(peer.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    peer.channel = 0;
    peer.ifidx = ESPNOW_WIFI_IF;

    ASSERT(esp_now_add_peer(&peer) == ESP_OK);
}

static void dongle_init()
{
    usb_serial_jtag_init();
    nvs_init();
    wifi_init(false);
    espnow_init();

    s_event_ringbuf = xRingbufferCreate(EVENT_RINGBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    ASSERT(s_event_ringbuf);

    const esp_timer_create_args_t periodic_ping_timer_args = {
        .callback = queue_ping_connected_drones_event_callback,
        .arg = NULL,
        .name = "periodic_ping_timer",
    };
    ASSERT(esp_timer_create(&periodic_ping_timer_args, &s_periodic_ping_timer) == ESP_OK);
    ASSERT(esp_timer_start_periodic(s_periodic_ping_timer, PING_CONNECTED_DRONE_INTERVAL_MS * 1000) == ESP_OK);

    LOG_DEBUG("Dongle Initialized!");
}

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
        ASSERT(relative_ptr % MAX_ESPNOW_SEND_EVENT_SIZE == 0);
        memset(e_send_event, 0, MAX_ESPNOW_SEND_EVENT_SIZE);
    }
}

static bool create_and_start_resend_pkt_timer(ESPNow_Send_Event* e_send_event, int64_t timeout_us)
{
    const esp_timer_create_args_t pkt_resend_timer_args = {
        .callback = queue_espnow_resend_unacknowledged_event_callback,
        .arg = e_send_event,
        .name = "pkt_resend_timer",
    };

    esp_err_t ret = esp_timer_create(&pkt_resend_timer_args, &e_send_event->pkt_resend_timer);
    if (ret != ESP_OK) {
        LOG_ERROR("create_and_start_resend_pkt_timer: esp_timer_create failed, ESP_ERR: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_timer_start_once(e_send_event->pkt_resend_timer, timeout_us);
    if (ret != ESP_OK) {
        LOG_ERROR("create_and_start_resend_pkt_timer: esp_timer_start_once failed, ESP_ERR: %s", esp_err_to_name(ret));
        return false;
    }

    return true;
}

static void delete_and_stop_resend_pkt_timer(ESPNow_Send_Event* e_send_event)
{
    if (e_send_event->pkt_resend_timer == NULL) {
        return;
    }

    esp_err_t ret = esp_timer_stop(e_send_event->pkt_resend_timer);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        LOG_ERROR("delete_and_stop_resend_pkt_timer: esp_timer_stop failed, ESP_ERR: %s", esp_err_to_name(ret));
    }

    ret = esp_timer_delete(e_send_event->pkt_resend_timer);
    if (ret != ESP_OK) {
        LOG_ERROR("delete_and_stop_resend_pkt_timer: esp_timer_delete failed, ESP_ERR: %s", esp_err_to_name(ret));
    }

    e_send_event->pkt_resend_timer = NULL;
}

static ESPNow_Send_Event* espnow_send_event_acquire_raw(const uint8_t* dest_mac_addr, size_t packet_size, TickType_t ticks_to_wait)
{
    size_t e_send_event_size = sizeof(ESPNow_Send_Event) - sizeof(EPacket) + packet_size;
    ESPNow_Send_Event* e_send_event = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_event_ringbuf, (void**) &e_send_event, e_send_event_size, ticks_to_wait);

    if (ret != pdTRUE) {
        LOG_ERROR("espnow_send_event_acquire_raw: Creating send event in ringbuffer failed. The ringbuffer might be full.");
        return NULL;
    }

    memset(e_send_event, 0, e_send_event_size);
    e_send_event->event_type = ESPNOW_SEND_EVENT;
    memcpy(e_send_event->mac_addr, dest_mac_addr, ESP_NOW_ETH_ALEN);

    return e_send_event;
}

static ESPNow_Send_Event* espnow_send_event_acquire_c2d(int dest_drone_idx, EPacket_Type type, size_t packet_size, EPacket_Flags flags)
{
    ASSERT(s_drone_infos[dest_drone_idx].occupied);

    ESPNow_Send_Event* e_send_event = espnow_send_event_acquire_raw(s_drone_infos[dest_drone_idx].mac_addr,
                                                                    packet_size,
                                                                    EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_MAIN_TASK);
    if (e_send_event) {
        e_send_event->epkt.header = espnow_header_create(type, packet_size, flags);
    }

    return e_send_event;
}

static ESPNow_Send_Event* espnow_send_event_acquire_m2d(int dest_drone_idx, const SPacket* src_master_spkt, EPacket_Type type, size_t packet_size, EPacket_Flags flags)
{
    ASSERT(s_drone_infos[dest_drone_idx].occupied);

    ESPNow_Send_Event* e_send_event = espnow_send_event_acquire_raw(s_drone_infos[dest_drone_idx].mac_addr,
                                                                    packet_size,
                                                                    EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_MAIN_TASK);
    if (e_send_event) {
        e_send_event->epkt.header = espnow_header_create(type, packet_size, flags);
        e_send_event->send_simple_response_to_master = true;
        e_send_event->origin_serial_packet_id = src_master_spkt->header.packet_id;
        e_send_event->origin_serial_packet_type = src_master_spkt->header.type;
    }

    return e_send_event;
}

static void espnow_send_event_complete(const ESPNow_Send_Event* e_send_event)
{
    UBaseType_t ret = xRingbufferSendComplete(s_event_ringbuf, (void*) e_send_event);
    if (ret != pdTRUE) {
        LOG_ERROR("espnow_send_event_complete: Completing send event in ringbuffer failed.");
    }
}

static Serial_Send_Event* serial_send_event_acquire(SPacket_Type type, size_t packet_size)
{
    size_t s_send_event_size = sizeof(Serial_Send_Event) - sizeof(SPacket) + packet_size;
    Serial_Send_Event* s_send_event = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_event_ringbuf,
                                             (void**) &s_send_event,
                                             s_send_event_size,
                                             EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_MAIN_TASK);

    if (ret != pdTRUE) {
        LOG_ERROR("serial_send_event_acquire: Creating send event in ringbuffer failed. The ringbuffer might be full.");
        return NULL;
    }

    memset(s_send_event, 0, s_send_event_size);
    s_send_event->event_type = SERIAL_SEND_EVENT;
    s_send_event->spkt.header = serial_header_create(type, packet_size);

    return s_send_event;
}

static void serial_send_event_complete(const Serial_Send_Event* s_send_event)
{
    UBaseType_t ret = xRingbufferSendComplete(s_event_ringbuf, (void*) s_send_event);
    if (ret != pdTRUE) {
        LOG_ERROR("serial_send_event_complete: Completing send event in ringbuffer failed.");
    }
}

static bool queue_serial_receive_event_from_packet(const SPacket* spkt, size_t packet_size)
{
    size_t s_recv_event_size = sizeof(Serial_Receive_Event) - sizeof(SPacket) + packet_size;
    Serial_Receive_Event* s_recv_event = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_event_ringbuf,
                                             (void**) &s_recv_event,
                                             s_recv_event_size,
                                             EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_OTHER_TASK);

    if (ret != pdTRUE) {
        LOG_ERROR("queue_serial_receive_event_from_packet: Creating receive event in ringbuffer failed. The ringbuffer might be full.");
        return false;
    }

    memset(s_recv_event, 0, s_recv_event_size);
    s_recv_event->event_type = SERIAL_RECEIVE_EVENT;
    memcpy(&s_recv_event->spkt, spkt, packet_size);

    ret = xRingbufferSendComplete(s_event_ringbuf, (void*) s_recv_event);
    if (ret != pdTRUE) {
        LOG_ERROR("queue_serial_receive_event_from_packet: Completing receive event in ringbuffer failed.");
        return false;
    }

    return true;
}

static void serial_send_msg_to_master(const char* format, ...)
{
    va_list args;

    static char pkt_buffer[sizeof(SPacket_C2M_Msg) + MAX_LOG_MSG_LENGTH];
    SPacket_C2M_Msg* spkt_log = (SPacket_C2M_Msg*) &pkt_buffer;

    va_start(args, format);
    int log_msg_size = vsnprintf(spkt_log->msg, MAX_LOG_MSG_LENGTH, format, args) + 1;
    va_end(args);

    size_t packet_size = sizeof(SPacket_C2M_Msg) + log_msg_size;
    spkt_log->header = serial_header_create(SPKT_C2M_MSG, packet_size);

    usb_serial_jtag_write_bytes(spkt_log, packet_size, USB_SERIAL_WRITE_LOG_MAX_TICKS_TO_WAIT);
}

static void queue_c2m_simple_response_for_send(SPacket_ID request_pkt_id, SPacket_Type request_pkt_type, uint8_t response)
{
    Serial_Send_Event* s_send_event = serial_send_event_acquire(SPKT_C2M_SIMPLE_RESPONSE,
                                                                sizeof(SPacket_C2M_Simple_Response));
    if (!s_send_event) {
        return;
    }

    SPacket_C2M_Simple_Response* spkt_simple_response = (SPacket_C2M_Simple_Response*) &s_send_event->spkt;
    spkt_simple_response->response = response;
    spkt_simple_response->request_pkt_id = request_pkt_id;
    spkt_simple_response->request_pkt_type = request_pkt_type;

    serial_send_event_complete(s_send_event);
}

static void handle_espnow_packet_acknowledge(ESPNow_Send_Event* e_send_event)
{
    delete_and_stop_resend_pkt_timer(e_send_event);

    const EPacket* epkt = &e_send_event->epkt;
    int drone_idx = get_drone_idx_from_mac_addr(e_send_event->mac_addr);
    if (drone_idx < 0) {
        release_unacknowledged_espnow_send_event(e_send_event);
        return;
    }

    uint8_t response_to_master = RESPONSE_FAIL;

    switch (epkt->header.type) {
        
    case EPKT_C2D_CONNECT:
    {
        s_drone_infos[drone_idx].connected = true;
        response_to_master = RESPONSE_SUCCESS;
        
    } break;

    case EPKT_C2D_DISCONNECT:
    {
        if (!s_drone_infos[drone_idx].connected) break;

        esp_err_t ret = esp_now_del_peer(s_drone_infos[drone_idx].mac_addr);
        if (ret != ESP_OK) {
            LOG_DEBUG("handle_espnow_packet_acknowledge: Failed to delete Drone from peer list, ESP_ERR: %s",
                      esp_err_to_name(ret));
        }

        s_drone_infos[drone_idx].connected = false;
        response_to_master = RESPONSE_SUCCESS;
        
    } break;

    case EPKT_C2D_PING: {
        response_to_master = RESPONSE_SUCCESS;
    } break;

    case EPKT_C2D_DRONE_SETTINGS: {
        response_to_master = RESPONSE_SUCCESS;
    } break;

    default: {
        LOG_DEBUG("Received acknowledge for ESP-Now Packet with unexpected type: '%u'", epkt->header.type);
    } break;
        
    }

    if (e_send_event->send_simple_response_to_master) {
        queue_c2m_simple_response_for_send(e_send_event->origin_serial_packet_id,
                                           e_send_event->origin_serial_packet_type,
                                           response_to_master);
    }

    release_unacknowledged_espnow_send_event(e_send_event);
}

static void handle_espnow_packet_failed_acknowledge(ESPNow_Send_Event* e_send_event)
{
    delete_and_stop_resend_pkt_timer(e_send_event);

    const EPacket* epkt = &e_send_event->epkt;
    int drone_idx = get_drone_idx_from_mac_addr(e_send_event->mac_addr);
    if (drone_idx >= 0) {
        s_drone_infos[drone_idx].responding = false;
    }

    uint8_t response_to_master = RESPONSE_FAIL;

    switch (epkt->header.type) {
    case EPKT_C2D_CONNECT:
    case EPKT_C2D_DISCONNECT:
    case EPKT_C2D_PING:
    case EPKT_C2D_DRONE_SETTINGS:
        response_to_master = RESPONSE_FAIL;
        break;

    default:
        LOG_DEBUG("Acknowledge failed for ESP-Now Packet with unexpected type: '%u'", epkt->header.type);
        break;
    }

    if (e_send_event->send_simple_response_to_master) {
        queue_c2m_simple_response_for_send(e_send_event->origin_serial_packet_id,
                                           e_send_event->origin_serial_packet_type,
                                           response_to_master);
    }

    release_unacknowledged_espnow_send_event(e_send_event);
}

static void espnow_send_callback(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void) tx_info;

    if (status != ESP_NOW_SEND_SUCCESS) {
        LOG_DEBUG("ESP-Now send callback reported that sending a packet failed.");
    }
}

static void espnow_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int length)
{
    if (recv_info == NULL || recv_info->src_addr == NULL || data == NULL || length <= 0) {
        LOG_ERROR("espnow_receive_callback: arg error");
        return;
    }

    EPacket* epkt = (EPacket*) data;
    if ((epkt->header.payload_size != length - (int) sizeof(EPacket)) || (epkt->header.type >= 64) || (epkt->header.packet_id == 0)) {
        LOG_DEBUG("espnow_receive_callback: Received a corrupted ESP-Now Packet");
        return;
    }

    size_t e_recv_event_size = sizeof(ESPNow_Receive_Event) - sizeof(EPacket) + length;
    ESPNow_Receive_Event* e_recv_event = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_event_ringbuf,
                                             (void**) &e_recv_event,
                                             e_recv_event_size,
                                             EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_OTHER_TASK);

    if (ret != pdTRUE) {
        LOG_ERROR("espnow_receive_callback: Creating receive event in ringbuffer failed. The ringbuffer might be full.");
        return;
    }

    memset(e_recv_event, 0, e_recv_event_size);
    e_recv_event->event_type = ESPNOW_RECEIVE_EVENT;
    memcpy(e_recv_event->mac_addr, recv_info->src_addr, ESP_NOW_ETH_ALEN);
    memcpy(&e_recv_event->epkt, data, length);

    ret = xRingbufferSendComplete(s_event_ringbuf, (void*) e_recv_event);
    if (ret != pdTRUE) {
        LOG_ERROR("espnow_receive_callback: Completing receive event in ringbuffer failed.");
    }
}

static void queue_espnow_resend_unacknowledged_event_callback(void* arg)
{
    ESPNow_Send_Event* e_send_event = (ESPNow_Send_Event*) arg;
    
    ESPNow_Resend_Unacknowledged_Event* e_resend_event = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_event_ringbuf,
                                             (void**) &e_resend_event,
                                             sizeof(ESPNow_Resend_Unacknowledged_Event),
                                             EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_OTHER_TASK);

    if (ret != pdTRUE) {
        LOG_ERROR("queue_espnow_resend_unacknowledged_event_callback: Creating resend event in ringbuffer failed. The ringbuffer might be full.");
        return;
    }

    e_resend_event->event_type = ESPNOW_RESEND_UNACKNOWLEDGED_EVENT;
    e_resend_event->e_send_event = e_send_event;

    ret = xRingbufferSendComplete(s_event_ringbuf, (void*) e_resend_event);
    if (ret != pdTRUE) {
        LOG_ERROR("queue_espnow_resend_unacknowledged_event_callback: Completing resend event in ringbuffer failed.");
    }
}

static void queue_ping_connected_drones_event_callback(void* arg)
{
    (void) arg;

    Ping_Connected_Drones_Event* ping_event = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_event_ringbuf,
                                             (void**) &ping_event,
                                             sizeof(Ping_Connected_Drones_Event),
                                             EVENT_RINGBUF_ACQUIRE_TICKS_TO_WAIT_FOR_OTHER_TASK);
    if (ret != pdTRUE) {
        LOG_ERROR("queue_ping_connected_drones_event_callback: Creating ping event in ringbuffer failed. The ringbuffer might be full.");
        return;
    }

    ping_event->event_type = PING_CONNECTED_DRONES_EVENT;

    ret = xRingbufferSendComplete(s_event_ringbuf, (void*) ping_event);
    if (ret != pdTRUE) {
        LOG_ERROR("queue_ping_connected_drones_event_callback: Completing ping event in ringbuffer failed.");
    }
}

static size_t serial_find_delimiter_offset(const uint8_t* buffer, size_t len)
{
    size_t delim_size = sizeof(Serial_Delimiter);

    if (len < delim_size) {
        return len;
    }

    for (size_t i = 0; i <= len - delim_size; ++i) {
        Serial_Delimiter delim = 0;
        memcpy(&delim, &buffer[i], delim_size);
        if (delim == SERIAL_DELIMITER) {
            return i;
        }
    }

    return len - delim_size + 1;
}

static void serial_rx_drop_prefix(size_t drop_len)
{
    if (drop_len == 0) {
        return;
    }

    if (drop_len >= s_serial_rx_buffer_len) {
        s_serial_rx_buffer_len = 0;
        return;
    }

    memmove(s_serial_rx_buffer, s_serial_rx_buffer + drop_len, s_serial_rx_buffer_len - drop_len);
    s_serial_rx_buffer_len -= drop_len;
}

static void serial_rx_process_buffer()
{
    while (1) {
        if (s_serial_rx_buffer_len < sizeof(SHeader)) {
            return;
        }

        size_t delim_offset = serial_find_delimiter_offset(s_serial_rx_buffer, s_serial_rx_buffer_len);
        if (delim_offset > 0) {
            serial_rx_drop_prefix(delim_offset);
            if (s_serial_rx_buffer_len < sizeof(SHeader)) {
                return;
            }
        }

        SHeader header = {0};
        memcpy(&header, s_serial_rx_buffer, sizeof(SHeader));

        if (header.delim != SERIAL_DELIMITER) {
            serial_rx_drop_prefix(1);
            continue;
        }

        if (header.payload_size < 0) {
            LOG_ERROR("serial_rx_process_buffer: Received serial packet with negative payload size.");
            serial_rx_drop_prefix(1);
            continue;
        }

        if (header.packet_id == 0) {
            LOG_ERROR("serial_rx_process_buffer: Received serial packet with packet_id 0.");
            serial_rx_drop_prefix(1);
            continue;
        }

        size_t packet_size = serial_packet_size(header);
        if (packet_size > SPKT_MAX_SIZE) {
            LOG_ERROR("serial_rx_process_buffer: Received oversized serial packet (%u bytes).", (unsigned) packet_size);
            serial_rx_drop_prefix(1);
            continue;
        }

        if (s_serial_rx_buffer_len < packet_size) {
            return;
        }

        queue_serial_receive_event_from_packet((SPacket*) s_serial_rx_buffer, packet_size);
        serial_rx_drop_prefix(packet_size);
    }
}

static void serial_rx_consume_bytes(const uint8_t* bytes, size_t bytes_len)
{
    if (bytes_len == 0) {
        return;
    }

    while (bytes_len > 0) {
        size_t free_space = SPKT_MAX_SIZE - s_serial_rx_buffer_len;
        if (free_space == 0) {
            serial_rx_process_buffer();
            free_space = SPKT_MAX_SIZE - s_serial_rx_buffer_len;

            if (free_space == 0) {
                LOG_ERROR("serial_rx_consume_bytes: Serial RX buffer is full and no complete packet could be extracted.");
                return;
            }
        }

        size_t chunk_size = bytes_len;
        if (chunk_size > free_space) {
            chunk_size = free_space;
        }

        memcpy(s_serial_rx_buffer + s_serial_rx_buffer_len, bytes, chunk_size);
        s_serial_rx_buffer_len += chunk_size;
        bytes += chunk_size;
        bytes_len -= chunk_size;

        serial_rx_process_buffer();
    }
}

static void usb_serial_jtag_select_notif_callback(usj_select_notif_t usb_serial_jtag_select_notif, BaseType_t *task_woken)
{
    (void) task_woken;

    if (usb_serial_jtag_select_notif != USJ_SELECT_READ_NOTIF) {
        return;
    }

    uint8_t tmp_buf[256];

    while (1) {
        size_t bytes_available = usb_serial_jtag_get_read_bytes_available();
        if (bytes_available == 0) {
            break;
        }

        size_t bytes_to_read = bytes_available;
        if (bytes_to_read > sizeof(tmp_buf)) {
            bytes_to_read = sizeof(tmp_buf);
        }

        int bytes_read = usb_serial_jtag_read_bytes(tmp_buf, bytes_to_read, USB_SERIAL_READ_MAX_TICKS_TO_WAIT);
        if (bytes_read <= 0) {
            break;
        }

        serial_rx_consume_bytes(tmp_buf, (size_t) bytes_read);
    }
}

static void handle_incoming_espnow_packet(const ESPNow_Receive_Event* e_recv_event)
{
    const EPacket* epkt = &e_recv_event->epkt;

    if (epkt->header.flags & EPKT_FLAG_REQUIRE_ACK) {
        int drone_idx = get_drone_idx_from_mac_addr(e_recv_event->mac_addr);
        if (drone_idx >= 0) {
            ESPNow_Send_Event* e_send_event = espnow_send_event_acquire_c2d(drone_idx,
                                                                            EPKT_C2D_ACK_RECEIVE,
                                                                            sizeof(EPacket_C2D_Ack_Receive),
                                                                            0);
            if (e_send_event) {
                EPacket_C2D_Ack_Receive* epkt_c2d_ack = (EPacket_C2D_Ack_Receive*) &e_send_event->epkt;
                epkt_c2d_ack->acknowledged_packet_id = epkt->header.packet_id;
                espnow_send_event_complete(e_send_event);
            }
        }
    }

    switch (epkt->header.type) {
    case EPKT_D2C_ADVERTISE:
    {
        EPacket_D2C_Advertise* epkt_advertise = (EPacket_D2C_Advertise*) epkt;

        int first_empty_idx = -1;
        bool drone_already_indexed = false;
        bool unexpected_drone = false;
        int drone_idx = 0;

        for (; drone_idx < MAX_DRONES; ++drone_idx)
        {
            if (first_empty_idx < 0 && !s_drone_infos[drone_idx].occupied) {
                first_empty_idx = drone_idx;
            }

            bool matching_mac_addr = memcmp(s_drone_infos[drone_idx].mac_addr, e_recv_event->mac_addr, ESP_NOW_ETH_ALEN) == 0;
            bool matching_drone_name = strncmp(s_drone_infos[drone_idx].drone_name, epkt_advertise->drone_name, DRONE_NAME_LENGTH) == 0;
            
            if (matching_mac_addr && matching_drone_name) {
                drone_already_indexed = true;
                break;
            }
            else if (matching_mac_addr && !matching_drone_name) {
                LOG_ERROR("Received AD from a drone with DIFFERENT NAME '%.*s' but the SAME MAC ADDRESS '" MACSTR "' as an already indexed drone.",
                          DRONE_NAME_LENGTH, epkt_advertise->drone_name, MAC2STR(e_recv_event->mac_addr));
                unexpected_drone = true;
            }
            else if (!matching_mac_addr && matching_drone_name) {
                LOG_ERROR("Received AD from a drone with DIFFERENT MAC ADDRESS '" MACSTR "' but the SAME NAME '%.*s' as an already indexed drone.",
                          MAC2STR(e_recv_event->mac_addr), DRONE_NAME_LENGTH, epkt_advertise->drone_name);
                unexpected_drone = true;
            }
        }

        if (unexpected_drone) {
            break;
        }

        if (drone_already_indexed) {
            // If the drone sends an advertisement even though its connected, then the drone was probably restarted
            // and lost the information about the connection, so we should just try to reconnect.
            if (s_drone_infos[drone_idx].connected) {
                ESPNow_Send_Event* e_send_event = espnow_send_event_acquire_c2d(drone_idx, EPKT_C2D_CONNECT, sizeof(EPacket_C2D_Connect), EPKT_FLAG_REQUIRE_ACK);
                if (e_send_event) {
                    espnow_send_event_complete(e_send_event);
                }
            }
        }
        else {
            if (first_empty_idx < 0) {
                LOG_ERROR("Failed to index Drone after advertisement, because all (%d) drone slots are full.", MAX_DRONES);
                break;
            }

            int idx_by_name = search_drone_idx_from_drone_name(epkt_advertise->drone_name);
            if (idx_by_name >= 0) {
                LOG_ERROR("Failed to index Drone after advertisement, because the name '%.*s' is already taken.",
                          DRONE_NAME_LENGTH, epkt_advertise->drone_name);
                break;
            }

            s_drone_infos[first_empty_idx].occupied = true;
            memcpy(s_drone_infos[first_empty_idx].mac_addr, e_recv_event->mac_addr, ESP_NOW_ETH_ALEN);
            memcpy(s_drone_infos[first_empty_idx].drone_name, epkt_advertise->drone_name, DRONE_NAME_LENGTH);
            s_drone_infos[first_empty_idx].connected = false;
            s_drone_infos[first_empty_idx].responding = false;
        }
        
    } break;

    case EPKT_D2C_ACK_RECEIVE:
    {
        EPacket_D2C_Ack_Receive* epkt_d2c_ack_recv = (EPacket_D2C_Ack_Receive*) epkt;
        ESPNow_Send_Event* e_send_event = get_unacknowledged_espnow_send_event_from_packet_id(epkt_d2c_ack_recv->acknowledged_packet_id);

        if (e_send_event) {
            handle_espnow_packet_acknowledge(e_send_event);
        }
        else {
            LOG_DEBUG("Received an EPKT_D2C_ACK_RECEIVE for packet with id '%u', but this packet was not in the wait-for-ack array",
                      epkt_d2c_ack_recv->acknowledged_packet_id);
        }
        
    } break;

    case EPKT_D2C_LOG_VALUES:
    {
        EPacket_D2C_Log_Values* epkt_d2c_logs = (EPacket_D2C_Log_Values*) epkt;
        int drone_idx = get_drone_idx_from_mac_addr(e_recv_event->mac_addr);
        if (drone_idx < 0) {
            break;
        }

        size_t spkt_size = sizeof(SPacket_C2M_Drone_Log_Values) + epkt_d2c_logs->log_value_count * sizeof(Payload_Named_Value);
        Serial_Send_Event* s_send_event = serial_send_event_acquire(SPKT_C2M_DRONE_LOG_VALUES, spkt_size);
        if (!s_send_event) {
            break;
        }

        SPacket_C2M_Drone_Log_Values* spkt_c2m_logs = (SPacket_C2M_Drone_Log_Values*) &s_send_event->spkt;
        memcpy(spkt_c2m_imu_data->drone_name, s_drone_infos[drone_idx].drone_name, DRONE_NAME_LENGTH);
        spkt_c2m_logs->log_value_count = epkt_d2c_logs->log_value_count;
        memcpy(spkt_c2m_logs->log_values, epkt_d2c_logs->log_values, epkt_d2c_logs->log_value_count * sizeof(Payload_Named_Value));

        serial_send_event_complete(s_send_event);
        
    } break;

    case EPKT_D2C_MSG:
    {
        EPacket_D2C_Msg* epkt_d2c_log = (EPacket_D2C_Msg*) epkt;
        int drone_idx = get_drone_idx_from_mac_addr(e_recv_event->mac_addr);
        if (drone_idx >= 0) {
            serial_send_msg_to_master("%.*s: %.*s", DRONE_NAME_LENGTH, s_drone_infos[drone_idx].drone_name,
                                      epkt_d2c_log->header.payload_size, epkt_d2c_log->msg);
        }
        
    } break;

    default: {
        LOG_DEBUG("Received ESP-Now Packet with unexpected or unknown type: '%u'", epkt->header.type);
    } break;
        
    }
}

static void handle_incoming_serial_packet(const SPacket* spkt)
{
    switch (spkt->header.type) {
    case SPKT_M2C_CONNECT_TO_DRONE:
    {
        SPacket_M2C_Connect_To_Drone* spkt_m2c_connect = (SPacket_M2C_Connect_To_Drone*) spkt;

        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_connect->drone_name);
        if (drone_idx < 0) {
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_FAIL);
            break;
        }

        if (s_drone_infos[drone_idx].connected) {
            LOG_DEBUG("Can't 'connect' to drone, because its already connected.");
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_SUCCESS);
            break;
        }

        esp_now_peer_info_t peer = {0};
        memcpy(peer.peer_addr, s_drone_infos[drone_idx].mac_addr, ESP_NOW_ETH_ALEN);
        peer.channel = 0;
        peer.ifidx = ESPNOW_WIFI_IF;

        esp_err_t ret = esp_now_add_peer(&peer);
        if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST) {
            LOG_ERROR("handle_incoming_serial_packet: Failed to add Drone to peer list, ESP_ERR: %s", esp_err_to_name(ret));
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_FAIL);
            break;
        }

        ESPNow_Send_Event* e_send_event = espnow_send_event_acquire_m2d(drone_idx, spkt, EPKT_C2D_CONNECT, sizeof(EPacket_C2D_Connect), EPKT_FLAG_REQUIRE_ACK);
        if (e_send_event) {
            espnow_send_event_complete(e_send_event);
        }
        else {
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_FAIL);
        }
        
    } break;

    case SPKT_M2C_DISCONNECT_FROM_DRONE:
    {
        SPacket_M2C_Disconnect_From_Drone* spkt_m2c_disconnect = (SPacket_M2C_Disconnect_From_Drone*) spkt;

        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_disconnect->drone_name);
        if (drone_idx < 0) {
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_FAIL);
            break;
        }

        if (!s_drone_infos[drone_idx].connected) {
            LOG_DEBUG("Can't 'disconnect' from drone, because its already disconnected.");
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_SUCCESS);
            break;
        }

        ESPNow_Send_Event* e_send_event = espnow_send_event_acquire_m2d(drone_idx, spkt, EPKT_C2D_DISCONNECT, sizeof(EPacket_C2D_Disconnect), EPKT_FLAG_REQUIRE_ACK);
        if (e_send_event) {
            espnow_send_event_complete(e_send_event);
        }
        else {
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_FAIL);
        }
        
    } break;

    case SPKT_M2C_CONTROLL_COMMANDS:
    {
        SPacket_M2C_Controll_Commands* spkt_m2c_ctrl_cmds = (SPacket_M2C_Controll_Commands*) spkt;

        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_ctrl_cmds->drone_name);
        if (drone_idx < 0) {
            break;
        }

        if (!s_drone_infos[drone_idx].connected) {
            LOG_ERROR("Can't send controll commands because the drone is not connected.");
            break;
        }

        ESPNow_Send_Event* e_send_event = espnow_send_event_acquire_c2d(drone_idx,
                                                                        EPKT_C2D_CONTROLL_COMMANDS,
                                                                        sizeof(EPacket_C2D_Controll_Commands),
                                                                        0);
        if (e_send_event) {
            EPacket_C2D_Controll_Commands* epkt_c2d_ctrl_cmds = (EPacket_C2D_Controll_Commands*) &e_send_event->epkt;
            memcpy(&epkt_c2d_ctrl_cmds->ctrl_cmds, &spkt_m2c_ctrl_cmds->ctrl_cmds, sizeof(Payload_Controll_Commands));
            espnow_send_event_complete(e_send_event);
        }
        
    } break;

    case SPKT_M2C_GET_DRONE_INFOS:
    {
        size_t drone_infos_count = 0;
        for (int i = 0; i < MAX_DRONES; ++i) {
            if (s_drone_infos[i].occupied) {
                ++drone_infos_count;
            }
        }

        size_t drone_infos_size = sizeof(SPacket_C2M_Drone_Infos) + drone_infos_count * sizeof(Payload_Drone_Info);
        Serial_Send_Event* s_send_event = serial_send_event_acquire(SPKT_C2M_DRONE_INFOS, drone_infos_size);
        if (!s_send_event) {
            break;
        }

        SPacket_C2M_Drone_Infos* spkt_c2m_drone_infos = (SPacket_C2M_Drone_Infos*) &s_send_event->spkt;
        spkt_c2m_drone_infos->drone_infos_count = drone_infos_count;

        int drone_infos_idx = 0;
        for (int i = 0; i < MAX_DRONES; ++i) {
            if (!s_drone_infos[i].occupied) {
                continue;
            }

            Payload_Drone_Info* drone_info = &spkt_c2m_drone_infos->drone_infos[drone_infos_idx];
            if (s_drone_infos[i].connected) {
                drone_info->flags |= DRONE_INFO_FLAG_IS_CONNETED;
            }
            if (s_drone_infos[i].responding) {
                drone_info->flags |= DRONE_INFO_FLAG_IS_RESPONDING;
            }
            memcpy(drone_info->drone_name, s_drone_infos[i].drone_name, DRONE_NAME_LENGTH);
            ++drone_infos_idx;
        }

        serial_send_event_complete(s_send_event);
    } break;

    case SPKT_M2C_PING_DRONE:
    {
        SPacket_M2C_Ping_Drone* spkt_m2c_ping = (SPacket_M2C_Ping_Drone*) spkt;

        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_ping->drone_name);
        if (drone_idx < 0) {
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_FAIL);
            break;
        }

        if (!s_drone_infos[drone_idx].connected) {
            LOG_ERROR("Can't ping drone because the drone is not connected.");
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_FAIL);
            break;
        }

        ESPNow_Send_Event* e_send_event = espnow_send_event_acquire_m2d(drone_idx, spkt, EPKT_C2D_PING, sizeof(EPacket_C2D_Ping), EPKT_FLAG_REQUIRE_ACK);
        if (e_send_event) {
            espnow_send_event_complete(e_send_event);
        }
        else {
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_FAIL);
        }
        
    } break;

    case SPKT_M2C_RESTART_DRONE:
    {
        SPacket_M2C_Restart_Drone* spkt_m2c_restart = (SPacket_M2C_Restart_Drone*) spkt;

        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_restart->drone_name);
        if (drone_idx < 0) {
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_FAIL);
            break;
        }

        if (!s_drone_infos[drone_idx].connected) {
            LOG_ERROR("Can't restart drone because the drone is not connected.");
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_FAIL);
            break;
        }

        ESPNow_Send_Event* e_send_event = espnow_send_event_acquire_m2d(drone_idx, spkt, EPKT_C2D_RESTART, sizeof(EPacket_C2D_Restart), 0);
        if (e_send_event) {
            espnow_send_event_complete(e_send_event);
        }
        else {
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_FAIL);
        }
        
    } break;

    case SPKT_M2C_SET_DRONE_PARAMETERS:
    {
        SPacket_M2C_Set_Drone_Parameters* spkt_m2c_set_params = (SPacket_M2C_Set_Drone_Parameters*) spkt;

        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_set_params->drone_name);
        if (drone_idx < 0) {
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_FAIL);
            break;
        }

        if (!s_drone_infos[drone_idx].connected) {
            LOG_ERROR("Can't set drone parameters because the drone is not connected.");
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_FAIL);
            break;
        }

        size_t epkt_size = sizeof(EPacket_C2D_Parameters) + spkt_m2c_set_params->parameter_count * sizeof(Payload_Named_Value);
        ESPNow_Send_Event* e_send_event = espnow_send_event_acquire_m2d(drone_idx, spkt, EPKT_C2D_PARAMETERS, epkt_size, EPKT_FLAG_REQUIRE_ACK);
        if (e_send_event) {
            EPacket_C2D_Parameters* epkt_c2d_params = (EPacket_C2D_Parameters*) &e_send_event->epkt;
            epkt_c2d_params->parameter_count = spkt_m2c_set_params->parameter_count;
            memcpy(epkt_c2d_params->parameters, spkt_m2c_set_params->parameters, spkt_m2c_set_params->parameter_count * sizeof(Payload_Named_Value));
            espnow_send_event_complete(e_send_event);
        }
        else {
            queue_c2m_simple_response_for_send(spkt->header.packet_id, spkt->header.type, RESPONSE_FAIL);
        }
        
    } break;

    default:
        LOG_DEBUG("Received Serial Packet with unexpected or unknown type: '%u'", spkt->header.type);
        break;
    }
}

static void handle_ping_connected_drones_event()
{
    for (int drone_idx = 0; drone_idx < MAX_DRONES; ++drone_idx) {
        if (!s_drone_infos[drone_idx].connected) {
            continue;
        }

        ESPNow_Send_Event* e_send_event = espnow_send_event_acquire_c2d(drone_idx, EPKT_C2D_PING, sizeof(EPacket_C2D_Ping), EPKT_FLAG_REQUIRE_ACK);
        if (e_send_event) {
            espnow_send_event_complete(e_send_event);
        }
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
        ESPNow_Send_Event* e_send_event_copy = acquire_unacknowledged_espnow_send_event();
        if (e_send_event_copy)
        {
            size_t e_send_event_size = sizeof(ESPNow_Send_Event) - sizeof(EPacket) + espnow_packet_size(epkt->header);
            memcpy(e_send_event_copy, e_send_event, e_send_event_size);
            bool success = create_and_start_resend_pkt_timer(e_send_event_copy, s_espnow_wait_for_ack_resend_timeouts_ms[0] * 1000);
            if (!success) {
                handle_espnow_packet_failed_acknowledge(e_send_event_copy);
            }
        }
        else {
            LOG_ERROR("send_espnow_packet: Failed to find an empty slot for an unacknowledged ESP-Now send event, all (%d) slots are occupied",
                      MAX_ESPNOW_PACKETS_WAITING_FOR_ACK);
        }
    }
}

static void resend_unacknowledged_espnow_packet(ESPNow_Resend_Unacknowledged_Event* e_resend_event)
{
    ESPNow_Send_Event* e_send_event = e_resend_event->e_send_event;
    EPacket* epkt = &e_send_event->epkt;

    espnow_send_bytes(e_send_event->mac_addr, (uint8_t*) epkt, espnow_packet_size(epkt->header));

    e_send_event->resend_count += 1;

    if (e_send_event->resend_count >= ESPNOW_WAIT_FOR_ACK_RESEND_TIMEOUTS_MS_LEN) {
        handle_espnow_packet_failed_acknowledge(e_send_event);
        return;
    }

    // Restart the timer
    esp_err_t ret = esp_timer_start_once(e_send_event->pkt_resend_timer,
                                         s_espnow_wait_for_ack_resend_timeouts_ms[e_send_event->resend_count] * 1000);
    if (ret != ESP_OK) {
        LOG_ERROR("resend_unacknowledged_espnow_packet: esp_timer_start_once failed, ESP_ERR: %s", esp_err_to_name(ret));
        handle_espnow_packet_failed_acknowledge(e_send_event);
    }
}

static void send_serial_packet(const Serial_Send_Event* s_send_event)
{
    const SPacket* spkt = &s_send_event->spkt;
    size_t spkt_size = serial_packet_size(spkt->header);

    int bytes_written = usb_serial_jtag_write_bytes(spkt, spkt_size, USB_SERIAL_WRITE_MAX_TICKS_TO_WAIT);
    if (bytes_written != (int) spkt_size) {
        LOG_ERROR("send_serial_packet: Failed to write packet to the serialport");
    }
}

int app_main(void)
{
    dongle_init();

    while (1) {
        size_t event_size = 0;
        void* event = xRingbufferReceive(s_event_ringbuf, &event_size, EVENT_RINGBUF_RECEIVE_TICKS_TO_WAIT);
        if (!event) {
            continue;
        }

        Event_Type event_type = *((Event_Type*) event);
        switch (event_type) {
        case ESPNOW_RECEIVE_EVENT:
            handle_incoming_espnow_packet((ESPNow_Receive_Event*) event);
            break;

        case ESPNOW_SEND_EVENT:
            send_espnow_packet((ESPNow_Send_Event*) event);
            break;

        case ESPNOW_RESEND_UNACKNOWLEDGED_EVENT:
            resend_unacknowledged_espnow_packet((ESPNow_Resend_Unacknowledged_Event*) event);
            break;

        case SERIAL_RECEIVE_EVENT:
            handle_incoming_serial_packet(&((Serial_Receive_Event*) event)->spkt);
            break;

        case SERIAL_SEND_EVENT:
            send_serial_packet((Serial_Send_Event*) event);
            break;

        case PING_CONNECTED_DRONES_EVENT:
            handle_ping_connected_drones_event();
            break;
        }

        vRingbufferReturnItem(s_event_ringbuf, event);
    }

    esp_restart(); // should be unreachable
}
