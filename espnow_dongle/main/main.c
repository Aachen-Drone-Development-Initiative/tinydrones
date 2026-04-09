#include "espnow_packets.h"
#include "serial_packets.h"

#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_timer.h"

#include "driver/usb_serial_jtag.h"

// Static Array sizes
#define MAX_DRONES 128

// Buffer sizes (the packets life in these buffers, so they should be rather big)
#define ESPNOW_RECEIVE_INFO_RINGBUF_SIZE (1024 * 10)
#define ESPNOW_SEND_INFO_RINGBUF_SIZE    (1024 * 10)
#define SERIAL_SEND_INFO_RINGBUF_SIZE    (1024 * 10)
#define USB_SERIAL_JTAG_IN_BUFFER_SIZE   (1024 * 10)
#define USB_SERIAL_JTAG_OUT_BUFFER_SIZE  (1024 * 10)

// Packet acknowledging
#define MAX_ESPNOW_PACKETS_WAITING_FOR_ACK 10
const int espnow_wait_for_ack_resend_timeouts_ms[] = {20, 40, 80, 160, 320, 640};
#define ESPNOW_WAIT_FOR_ACK_RESEND_TIMEOUTS_MS_LEN (sizeof(espnow_wait_for_ack_resend_timeouts_ms) / sizeof(int))

// Max Latency on blocking ops
#define USB_SERIAL_WRITE_MAX_TICKS_TO_WAIT            pdMS_TO_TICKS(10) 
#define USB_SERIAL_WRITE_LOG_MAX_TICKS_TO_WAIT        pdMS_TO_TICKS(0) // dont stall because of logs
#define USB_SERIAL_READ_MAX_TICKS_TO_WAIT             pdMS_TO_TICKS(10)
#define ESPNOW_RECEIVE_INFO_RINGBUF_MAX_TICKS_TO_WAIT pdMS_TO_TICKS(10)
#define ESPNOW_SEND_INFO_RINGBUF_MAX_TICKS_TO_WAIT    pdMS_TO_TICKS(10)
#define SERIAL_SEND_INFO_RINGBUF_MAX_TICKS_TO_WAIT    pdMS_TO_TICKS(10)

// ESP-Now Settings
#define ESPNOW_WIFI_MODE WIFI_MODE_AP // The dongle should be the Access Point that the drones connect to
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP // We want to use the Access Point interface (IF)

// How often we should ping the drone. We are required to send a message
// within 'MIN_C2D_SEND_INTERVAL_MS', else the drone will think we are offline.
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
            abort();                                                     \
        }                                                               \
    } while (0)

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    EPacket epkt;
} ESPNow_Receive_Info;

typedef struct {
    bool occupied;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    int64_t last_send_time_us;
    int resend_count;
    EPacket epkt;
} ESPNow_Send_Info;

#define MAX_ESPNOW_SEND_INFO_SIZE (sizeof(ESPNow_Send_Info) - sizeof(EPacket) + EPKT_MAX_SIZE)

typedef struct {
    SPacket spkt;
} Serial_Send_Info;

typedef struct {
    bool occupied;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    char drone_name[MAX_DRONE_NAME_LENGTH];
    bool connected;
    bool responding;
    int64_t last_ping_time_us;
} Drone_Info;

static Drone_Info s_drone_infos[MAX_DRONES] = {0};

// incoming packets
static RingbufHandle_t s_espnow_receive_info_ringbuf = NULL;
static uint8_t s_serial_packet_receive_buffer[SPKT_MAX_SIZE] = {0};

// outgoing packets
static RingbufHandle_t s_espnow_send_info_ringbuf = NULL;
static RingbufHandle_t s_serial_send_info_ringbuf = NULL;

// waiting packets
static uint8_t s_espnow_send_infos_waiting_for_ack[MAX_ESPNOW_PACKETS_WAITING_FOR_ACK][MAX_ESPNOW_SEND_INFO_SIZE] = {0};
static SemaphoreHandle_t s_espnow_send_infos_waiting_for_ack_mutex = NULL;

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
        ASSERT(esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR)
               == ESP_OK);
    }
}

static void espnow_send_callback(const esp_now_send_info_t *tx_info, esp_now_send_status_t status);
static void espnow_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int length);

static void espnow_init()
{
    ASSERT(esp_now_init() == ESP_OK);
    ASSERT(esp_now_register_send_cb(espnow_send_callback) == ESP_OK);
    ASSERT(esp_now_register_recv_cb(espnow_receive_callback) == ESP_OK);
    
    // Adding Boradcast peer to peer list.
    esp_now_peer_info_t peer = {0};
    static const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memcpy(peer.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    peer.channel = 0;
    peer.ifidx = ESPNOW_WIFI_IF;
    
    ASSERT(esp_now_add_peer(&peer) == ESP_OK);
}

static void dongle_init()
{
    usb_serial_jtag_init(); // init this first so that we have logging
    nvs_init();
    wifi_init(false);
    espnow_init();

    s_espnow_receive_info_ringbuf = xRingbufferCreate(ESPNOW_RECEIVE_INFO_RINGBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    s_espnow_send_info_ringbuf    = xRingbufferCreate(ESPNOW_SEND_INFO_RINGBUF_SIZE,    RINGBUF_TYPE_NOSPLIT);
    s_serial_send_info_ringbuf    = xRingbufferCreate(SERIAL_SEND_INFO_RINGBUF_SIZE,    RINGBUF_TYPE_NOSPLIT);

    s_espnow_send_infos_waiting_for_ack_mutex = xSemaphoreCreateMutex();

    LOG_DEBUG("Dongle Initialized!");
}

static void espnow_send_callback(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    /* This callback is not really utilized, because even if 'status' is ESP_NOW_SEND_FAIL
       we won't send the packet again. Mainly because it's difficult to know
       which packet 'tx_info' refers to (trust me its difficult). */

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

    ESPNow_Receive_Info* e_recv_info = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_espnow_receive_info_ringbuf,
                                             (void**) &e_recv_info,
                                             sizeof(ESPNow_Receive_Info) - sizeof(EPacket) + length,
                                             ESPNOW_RECEIVE_INFO_RINGBUF_MAX_TICKS_TO_WAIT);

    if (ret != pdTRUE) {
        LOG_DEBUG("espnow_receive_callback: Creating espnow receive info in ringbuffer failed. The ringbuffer might be full.");
        return;
    }

    memcpy(e_recv_info->mac_addr, recv_info->src_addr, ESP_NOW_ETH_ALEN);
    memcpy(&e_recv_info->epkt, data, length);
    // We should probably just remove the packet from the ringbuffer if this fails, but not sure if it ever would
    // do that in case this assert ever triggers.
    ASSERT(e_recv_info->epkt.header.payload_size == length - sizeof(EPacket));

    ret = xRingbufferSendComplete(s_espnow_receive_info_ringbuf, (void*) e_recv_info);
    if (ret != pdTRUE) {
        LOG_DEBUG("espnow_receive_callback: Completing espnow receive info in ringbuffer failed.");
    }
}

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

static int get_drone_idx_from_drone_name(const char* drone_name)
{
    for (int i = 0; i < MAX_DRONES; ++i) {
        if (memcmp(s_drone_infos[i].drone_name, drone_name, MAX_DRONE_NAME_LENGTH) == 0) {
            return i;
        }
    }
    
    LOG_ERROR("No drone available with name: '%.*s'.", MAX_DRONE_NAME_LENGTH, drone_name);
    return -1;
}

static size_t espnow_packet_size(EHeader e_header) { return sizeof(EHeader) + e_header.payload_size; }
static size_t serial_packet_size(SHeader s_header) { return sizeof(SHeader) + s_header.payload_size; }

static EHeader espnow_header_create(EPacket_Type type, size_t packet_size, EPacket_Flags flags)
{
    ASSERT(packet_size >= sizeof(EPacket));
    
    return (EHeader) {
        .type = type,
        .flags = flags,
        .packet_id = esp_random(),
        .payload_size = packet_size - sizeof(EPacket)
    };
}

static SHeader serial_header_create(SPacket_Type type, size_t packet_size)
{
    ASSERT(packet_size >= sizeof(SPacket));
    
    return (SHeader) {
        .delim = SERIAL_DELIMITER,
        .type = type,
        .payload_size = packet_size - sizeof(SPacket)
    };
}

static ESPNow_Send_Info* espnow_send_info_acquire(int dest_drone_idx, EPacket_Type type, size_t packet_size, EPacket_Flags flags)
{
    ASSERT(s_drone_infos[dest_drone_idx].occupied == true);

    size_t e_send_info_size = sizeof(ESPNow_Send_Info) - sizeof(EPacket) + packet_size;
    ESPNow_Send_Info* e_send_info = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_espnow_send_info_ringbuf,
                                             (void**) &e_send_info,
                                             e_send_info_size,
                                             SERIAL_SEND_INFO_RINGBUF_MAX_TICKS_TO_WAIT);

    if (ret != pdTRUE) {
        LOG_DEBUG("espnow_send_info_acquire: Creating espnow send info in ringbuffer failed. The ringbuffer might be full.");
        return NULL;
    }

    memset(e_send_info, 0, e_send_info_size);
    e_send_info->occupied = true;
    memcpy(e_send_info->mac_addr, s_drone_infos[dest_drone_idx].mac_addr, ESP_NOW_ETH_ALEN);
    e_send_info->epkt.header = espnow_header_create(type, packet_size, flags);

    return e_send_info;
}

static Serial_Send_Info* serial_send_info_acquire(SPacket_Type type, size_t packet_size)
{
    size_t s_send_info_size = packet_size;
    Serial_Send_Info* s_send_info = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_serial_send_info_ringbuf,
                                             (void**) &s_send_info,
                                             s_send_info_size,
                                             SERIAL_SEND_INFO_RINGBUF_MAX_TICKS_TO_WAIT);

    if (ret != pdTRUE) {
        LOG_DEBUG("serial_send_info_acquire: Creating serial send info in ringbuffer failed. The ringbuffer might be full.");
        return NULL;
    }

    memset(s_send_info, 0, s_send_info_size);
    s_send_info->spkt.header = serial_header_create(type, packet_size);

    return s_send_info;
}

static void serial_send_info_complete(const Serial_Send_Info* s_send_info)
{
    UBaseType_t ret = xRingbufferSendComplete(s_serial_send_info_ringbuf, (void*) s_send_info);
    
    if (ret != pdTRUE) {
        LOG_DEBUG("serial_send_info_complete: Completing serial send info in ringbuffer failed.");
    }
}

static void espnow_send_info_complete(const ESPNow_Send_Info* e_send_info)
{
    UBaseType_t ret = xRingbufferSendComplete(s_espnow_send_info_ringbuf, (void*) e_send_info);
    
    if (ret != pdTRUE) {
        LOG_DEBUG("espnow_send_info_complete: Completing espnow send info in ringbuffer failed.");
    }
}

static void serial_send_msg_to_master(const char* format, ...)
{
    va_list args;

    // We dont use 'serial_send_info_acquire/complete', but instead allocate
    // the required memory statically and send the packet immediately.
    static char pkt_buffer[sizeof(SPacket_C2M_Log) + MAX_LOG_MSG_LENGTH];
    SPacket_C2M_Log* spkt_log = (SPacket_C2M_Log*) &pkt_buffer;

    va_start(args, format);
    int log_msg_size = vsnprintf(spkt_log->msg, MAX_LOG_MSG_LENGTH, format, args) + 1; // + 1 because '\0' is not counted
    va_end(args);    

    size_t packet_size = sizeof(SPacket_C2M_Log) + log_msg_size;
    spkt_log->header = serial_header_create(SPKT_C2M_LOG, packet_size);
    
    usb_serial_jtag_write_bytes(spkt_log, packet_size, USB_SERIAL_WRITE_LOG_MAX_TICKS_TO_WAIT);
    // Importantly we don't log any errors that might occur in this function!
}

static void send_c2m_drone_connection_update(int drone_idx, uint8_t update)
{
    Serial_Send_Info* s_send_info = serial_send_info_acquire(SPKT_C2M_DRONE_CONNECTION_UPDATE, sizeof(SPacket_C2M_Drone_Connection_Update));
    if (!s_send_info) return;

    SPacket_C2M_Drone_Connection_Update* spkt_conn_update = (SPacket_C2M_Drone_Connection_Update*) &s_send_info->spkt;
    spkt_conn_update->update = update;
    memcpy(spkt_conn_update->drone_name, s_drone_infos[drone_idx].drone_name, MAX_DRONE_NAME_LENGTH);

    serial_send_info_complete(s_send_info);
}

static void send_c2m_ping_drone_result(int drone_idx, uint8_t drone_responded)
{
    Serial_Send_Info* s_send_info = serial_send_info_acquire(SPKT_C2M_PING_DRONE_RESULT, sizeof(SPacket_C2M_Ping_Drone_Result));
    if (!s_send_info) return;

    SPacket_C2M_Ping_Drone_Result* spkt_ping_res = (SPacket_C2M_Ping_Drone_Result*) &s_send_info->spkt;
    spkt_ping_res->drone_responded = drone_responded;
    memcpy(spkt_ping_res->drone_name, s_drone_infos[drone_idx].drone_name, MAX_DRONE_NAME_LENGTH);

    serial_send_info_complete(s_send_info);
}

static void process_espnow_packet_acknowledge(const ESPNow_Send_Info* e_send_info)
{
    const EPacket* epkt = &e_send_info->epkt;

    int drone_idx = get_drone_idx_from_mac_addr(e_send_info->mac_addr);
    if (drone_idx < 0) return;
        
    switch (epkt->header.type) {
        
    case EPKT_C2D_CONNECT:
    {
        if (s_drone_infos[drone_idx].connected == true) break;
        
        s_drone_infos[drone_idx].connected = true;
        send_c2m_drone_connection_update(drone_idx, DRONE_CONN_UPDATE_SUCCESSFULLY_CONNECTED);
        
    } break;

    case EPKT_C2D_DISCONNECT:
    {
        if (s_drone_infos[drone_idx].connected == false) break;
        
        // Try to remove the drone to the espnow peer list
        esp_err_t ret = esp_now_del_peer(s_drone_infos[drone_idx].mac_addr);
        if (ret != ESP_OK) {
            LOG_DEBUG("process_espnow_packet_acknowledge: Failed to delete Drone from peer list, ESP_ERR: %s", esp_err_to_name(ret));
        }

        s_drone_infos[drone_idx].connected = false;
        send_c2m_drone_connection_update(drone_idx, DRONE_CONN_UPDATE_SUCCESSFULLY_DISCONNECTED);
        
    } break;

    case EPKT_C2D_PING:
    {
        const EPacket_C2D_Ping* epkt_ping = (EPacket_C2D_Ping*) epkt;
        if (epkt_ping->notify_master) {
            send_c2m_ping_drone_result(drone_idx, true);
        }
        
    } break;
        
    default:
        LOG_DEBUG("process_espnow_packet_acknowledge: Unexpected packet type '%u'", epkt->header.type);
    }
}

static void process_espnow_packet_failed_acknowledge(const ESPNow_Send_Info* e_send_info)
{
    const EPacket* epkt = &e_send_info->epkt;

    int drone_idx = get_drone_idx_from_mac_addr(e_send_info->mac_addr);
    if (drone_idx < 0) return;

    switch (epkt->header.type) {
        
    case EPKT_C2D_CONNECT: {
        send_c2m_drone_connection_update(drone_idx, DRONE_CONN_UPDATE_FAILED_TO_CONNECT);
    } break;
    
    case EPKT_C2D_DISCONNECT: {
        send_c2m_drone_connection_update(drone_idx, DRONE_CONN_UPDATE_FAILED_TO_DISCONNECT);
    } break;

    case EPKT_C2D_PING: {
        send_c2m_ping_drone_result(drone_idx, false);
    } break;

    default:
        LOG_DEBUG("process_espnow_packet_failed_acknowledge: Unexpected packet type '%u'", epkt->header.type);
    }
}

static void process_incoming_espnow_packet(const ESPNow_Receive_Info* e_recv_info)
{
    const EPacket* epkt = &e_recv_info->epkt;

    // If we need to send an ack, we should do that before sending any other espnow packets
    if (epkt->header.flags & EPKT_FLAG_REQUIRE_ACK)
    {
        int drone_idx = get_drone_idx_from_mac_addr(e_recv_info->mac_addr);
        if (drone_idx >= 0) {
            ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(drone_idx, EPKT_C2D_ACK_RECEIVE, sizeof(EPacket_C2D_Ack_Receive), 0);
            if (e_send_info) {
                EPacket_C2D_Ack_Receive* epkt_c2d_ack = (EPacket_C2D_Ack_Receive*) &e_send_info->epkt;
                epkt_c2d_ack->acknowledged_packet_id = epkt->header.packet_id;
                espnow_send_info_complete(e_send_info);
            }
        }
    }

    switch (epkt->header.type) {
        
    case EPKT_D2C_ADVERTISE:
    {
        EPacket_D2C_Advertise* epkt_advertise = (EPacket_D2C_Advertise*) epkt;

        // check if the drone is already indexed, and if not find the first empty index
        int first_empty_idx = -1;
        bool drone_already_indexed = false;
        bool unexpected_drone = false;
        int drone_idx = 0;
        for (; drone_idx < MAX_DRONES; ++drone_idx) {
            if (first_empty_idx < 0 && !s_drone_infos[drone_idx].occupied) {
                first_empty_idx = drone_idx;
            }
            
            bool matching_mac_addr = false;
            if (memcmp(s_drone_infos[drone_idx].mac_addr, e_recv_info->mac_addr, ESP_NOW_ETH_ALEN) == 0) {
                matching_mac_addr = true;
            }
            bool matching_drone_name = false;
            if (strncmp(s_drone_infos[drone_idx].drone_name, epkt_advertise->drone_name, MAX_DRONE_NAME_LENGTH) == 0) {
                matching_drone_name = true;
            }
            
            if (matching_mac_addr && matching_drone_name) {
                drone_already_indexed = true;
                break;
            }
            else if (matching_mac_addr && !matching_drone_name) {
                LOG_ERROR("Received AD from a drone with DIFFERENT NAME '%.*s' but the SAME MAC ADDRESS '"MACSTR"' as an already indexed drone.",
                          MAX_DRONE_NAME_LENGTH, epkt_advertise->drone_name, MAC2STR(e_recv_info->mac_addr));
                unexpected_drone = true;
            }
            else if (!matching_mac_addr && matching_drone_name) {
                LOG_ERROR("Received AD from a drone with DIFFERENT MAC ADDRESS '"MACSTR"' but the SAME NAME '%.*s' as an already indexed drone.",
                          MAC2STR(e_recv_info->mac_addr), MAX_DRONE_NAME_LENGTH, epkt_advertise->drone_name);
                unexpected_drone = true;
            }
        }

        if (unexpected_drone) break;

        if (drone_already_indexed) {
            // If we receive an advertisement from a drone that we are also connected to,
            // then the drone probably restarted and lost the information that it was
            // connected to us. Therefore we need to reconnect.
            if (s_drone_infos[drone_idx].connected) {
                ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(drone_idx, EPKT_C2D_CONNECT, sizeof(EPacket_C2D_Connect), EPKT_FLAG_REQUIRE_ACK);
                if (e_send_info) {
                    espnow_send_info_complete(e_send_info);
                }
            }
        }
        else {
            // Check if there is an empty slot
            if (first_empty_idx < 0) {
                LOG_ERROR("Failed to index Drone after advertisement, because all (%d) drone slots are full.", MAX_DRONES);
                break;
            }
            
            // Check if the drone name is already taken
            int idx_by_name = get_drone_idx_from_drone_name(epkt_advertise->drone_name);
            if (idx_by_name > 0) {
                LOG_ERROR("Failed to index Drone after advertisement, because the name '%.*s' is already taken.",
                              MAX_DRONE_NAME_LENGTH, epkt_advertise->drone_name);
                break;
            }

            // Finally index the new drone
            s_drone_infos[first_empty_idx].occupied = true;
            memcpy(s_drone_infos[first_empty_idx].mac_addr, e_recv_info->mac_addr, ESP_NOW_ETH_ALEN);
            memcpy(s_drone_infos[first_empty_idx].drone_name, epkt_advertise->drone_name, MAX_DRONE_NAME_LENGTH);
            s_drone_infos[first_empty_idx].connected = false;
        }
        
    } break;

    case EPKT_D2C_ACK_RECEIVE:
    {
        EPacket_D2C_Ack_Receive* epkt_d2c_ack_recv = (EPacket_D2C_Ack_Receive*) epkt;
        EPacket_ID acknowledged_packet_id = epkt_d2c_ack_recv->acknowledged_packet_id;

        int drone_idx = get_drone_idx_from_mac_addr(e_recv_info->mac_addr);
        if (drone_idx < 0) break;
        
        int i = 0;
        xSemaphoreTake(s_espnow_send_infos_waiting_for_ack_mutex, portMAX_DELAY);
        {
            for (; i < MAX_ESPNOW_PACKETS_WAITING_FOR_ACK; ++i)
            {
                ESPNow_Send_Info* e_send_info = (ESPNow_Send_Info*) &s_espnow_send_infos_waiting_for_ack[i];
                if (e_send_info->occupied && e_send_info->epkt.header.packet_id == acknowledged_packet_id)
                {
                    int drone_idx = get_drone_idx_from_mac_addr(e_send_info->mac_addr);
                    if (drone_idx >= 0) {
                        s_drone_infos[drone_idx].responding = true;
                    }
                    process_espnow_packet_acknowledge(e_send_info);
                    memset(e_send_info, 0, MAX_ESPNOW_SEND_INFO_SIZE);
                    break;
                }
            }
        }
        xSemaphoreGive(s_espnow_send_infos_waiting_for_ack_mutex);

        if (i == MAX_ESPNOW_PACKETS_WAITING_FOR_ACK) {
            LOG_DEBUG("Received an EPKT_D2C_ACK_RECEIVE for packet with id '%u', but this packet was not in the 'wait for ack array'", acknowledged_packet_id);
        }
        
    } break;

    case EPKT_D2C_LOG:
    {
        EPacket_D2C_Log* epkt_d2c_log = (EPacket_D2C_Log*) epkt;

        int drone_idx = get_drone_idx_from_mac_addr(e_recv_info->mac_addr);
        if (drone_idx < 0) break;

        serial_send_msg_to_master("%.*s: %.*s", MAX_DRONE_NAME_LENGTH, s_drone_infos[drone_idx].drone_name,
                                                epkt_d2c_log->header.payload_size, epkt_d2c_log->msg);
    } break;
        
    case EPKT_D2C_IMU_DATA:
    {
        EPacket_D2C_Imu_Data* epkt_d2c_imu_data = (EPacket_D2C_Imu_Data*) epkt;
        
        int drone_idx = get_drone_idx_from_mac_addr(e_recv_info->mac_addr);
        if (drone_idx < 0) break;

        Serial_Send_Info* s_send_info = serial_send_info_acquire(SPKT_C2M_DRONE_IMU_DATA, sizeof(SPacket_C2M_Drone_Imu_Data));
        if (!s_send_info) break;

        SPacket_C2M_Drone_Imu_Data* spkt_c2m_imu_data = (SPacket_C2M_Drone_Imu_Data*) &s_send_info->spkt;
        memcpy(&spkt_c2m_imu_data->imu_data, &epkt_d2c_imu_data->imu_data, sizeof(Payload_Imu_Data));
        memcpy(spkt_c2m_imu_data->drone_name, s_drone_infos[drone_idx].drone_name, MAX_DRONE_NAME_LENGTH);

        serial_send_info_complete(s_send_info);
        
    } break;

    case EPKT_D2C_TELEMETRY:
    {
        EPacket_D2C_Telemetry* epkt_d2c_telem = (EPacket_D2C_Telemetry*) epkt;
        
        int drone_idx = get_drone_idx_from_mac_addr(e_recv_info->mac_addr);
        if (drone_idx < 0) break;

        Serial_Send_Info* s_send_info = serial_send_info_acquire(SPKT_C2M_DRONE_TELEMETRY, sizeof(SPacket_C2M_Drone_Telemetry));
        if (!s_send_info) break;

        SPacket_C2M_Drone_Telemetry* spkt_c2m_telem = (SPacket_C2M_Drone_Telemetry*) &s_send_info->spkt;
        memcpy(&spkt_c2m_telem->telemetry, &epkt_d2c_telem->telemetry, sizeof(Payload_Telemetry));
        memcpy(spkt_c2m_telem->drone_name, s_drone_infos[drone_idx].drone_name, MAX_DRONE_NAME_LENGTH);

        serial_send_info_complete(s_send_info);
                
    } break;
        
    default:
        LOG_DEBUG("Received ESP-Now Packet with unexpected or unknown type: '%u'", epkt->header.type);
    }
}

static bool read_next_serial_packet(SPacket* spkt, TickType_t ticks_to_wait)
{
    int bytes_read_header = usb_serial_jtag_read_bytes(&spkt->header, sizeof(SHeader), ticks_to_wait);
    if (bytes_read_header != sizeof(SHeader)) {
        return false;
    }

    if (spkt->header.delim != SERIAL_DELIMITER) {
        LOG_ERROR("Failed to read serial packet: Delimiter was not at the expected location");
        // TODO: 'skip to next delimiter'
        return false;
    }

    int bytes_read_payload = usb_serial_jtag_read_bytes(spkt->payload, spkt->header.payload_size, ticks_to_wait);
    
    if (bytes_read_payload != spkt->header.payload_size) {
        LOG_ERROR("Failed to read payload of serial packet: Not enough data available, but timeout reached.");
        return false;
    }

    return true;
}

static void process_incoming_serial_packet(const SPacket* spkt)
{
    switch (spkt->header.type) {
        
    case SPKT_M2C_CONNECT_TO_DRONE:
    {
        SPacket_M2C_Connect_To_Drone* spkt_m2c_connect = (SPacket_M2C_Connect_To_Drone*) spkt;
        
        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_connect->drone_name);
        if (drone_idx < 0) {
            send_c2m_drone_connection_update(drone_idx, DRONE_CONN_UPDATE_FAILED_TO_CONNECT);
            break;
        }

        if (s_drone_infos[drone_idx].connected) {
            LOG_DEBUG("Can't 'connect' to drone, because its already connected.");
            send_c2m_drone_connection_update(drone_idx, DRONE_CONN_UPDATE_SUCCESSFULLY_CONNECTED);
            break; 
        }
        
        // Try to add the drone to the espnow peer list
        esp_now_peer_info_t peer = {0};
        memcpy(peer.peer_addr, s_drone_infos[drone_idx].mac_addr, ESP_NOW_ETH_ALEN);
        peer.channel = 0;
        peer.ifidx = ESPNOW_WIFI_IF;
            
        esp_err_t ret = esp_now_add_peer(&peer);
        if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST) { // ESP_ERR_ESPNOW_EXIST means the drone already was already in the peer list!
            LOG_ERROR("process_incoming_espnow_packet: Failed to add Drone to peer list, ESP_ERR: %s", esp_err_to_name(ret));
            send_c2m_drone_connection_update(drone_idx, DRONE_CONN_UPDATE_FAILED_TO_CONNECT);
            break;
        }

        ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(drone_idx, EPKT_C2D_CONNECT, sizeof(EPacket_C2D_Connect), EPKT_FLAG_REQUIRE_ACK);
        if (e_send_info) {
            espnow_send_info_complete(e_send_info);
        }
        
    } break;
    
    case SPKT_M2C_DISCONNECT_FROM_DRONE:
    {
        SPacket_M2C_Disconnect_From_Drone* spkt_m2c_disconnect = (SPacket_M2C_Disconnect_From_Drone*) spkt;
        
        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_disconnect->drone_name);
        if (drone_idx < 0) break;

        if (!s_drone_infos[drone_idx].connected) {
            LOG_DEBUG("Can't 'disconnect' from drone, because its already disconnected.");
            send_c2m_drone_connection_update(drone_idx, DRONE_CONN_UPDATE_SUCCESSFULLY_DISCONNECTED);
            break;
        }
        
        ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(drone_idx, EPKT_C2D_DISCONNECT, sizeof(EPacket_C2D_Disconnect), EPKT_FLAG_REQUIRE_ACK);
        if (e_send_info) {
            espnow_send_info_complete(e_send_info);
        }
        
    } break;
    
    case SPKT_M2C_CONTROLL_COMMANDS:
    {
        SPacket_M2C_Controll_Commands* spkt_m2c_ctrl_cmds = (SPacket_M2C_Controll_Commands*) spkt;
        
        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_ctrl_cmds->drone_name);
        if (drone_idx < 0) break;

        if (!s_drone_infos[drone_idx].connected) {
            LOG_ERROR("Can't send controll commands because the drone is not connected.");
            break;
        }

        ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(drone_idx, EPKT_C2D_CONTROLL_COMMANDS, sizeof(EPacket_C2D_Controll_Commands), 0);
        if (e_send_info) {
            EPacket_C2D_Controll_Commands* epkt_c2d_ctrl_cmds = (EPacket_C2D_Controll_Commands*) &e_send_info->epkt;
            memcpy(&epkt_c2d_ctrl_cmds->ctrl_cmds, &spkt_m2c_ctrl_cmds->ctrl_cmds, sizeof(Payload_Controll_Commands));
            espnow_send_info_complete(e_send_info);
        }
        
    } break;
    
    case SPKT_M2C_GET_DRONE_INFOS:
    {
        size_t drone_infos_count = 0;
        for (int i = 0; i < MAX_DRONES; ++i) {
            if (s_drone_infos[i].occupied) ++drone_infos_count;
        }

        int drone_infos_size = sizeof(SPacket_C2M_Drone_Infos) + drone_infos_count * sizeof(Payload_Drone_Info);

        Serial_Send_Info* s_send_info = serial_send_info_acquire(SPKT_C2M_DRONE_INFOS, drone_infos_size);
        if (!s_send_info) break;
        
        SPacket_C2M_Drone_Infos* spkt_c2m_drone_infos = (SPacket_C2M_Drone_Infos*) &s_send_info->spkt;
        spkt_c2m_drone_infos->drone_infos_count = drone_infos_count;

        int drone_infos_idx = 0;
        for (int i = 0; i < MAX_DRONES; ++i) {
            if (s_drone_infos[i].occupied) {
                Payload_Drone_Info* drone_info = &spkt_c2m_drone_infos->drone_infos[drone_infos_idx];
                if (s_drone_infos[i].connected) {
                    drone_info->flags |= DRONE_INFO_FLAG_IS_CONNETED;
                }
                if (s_drone_infos[i].responding) {
                    drone_info->flags |= DRONE_INFO_FLAG_IS_RESPONDING;
                }
                memcpy(drone_info->drone_name, s_drone_infos[i].drone_name, MAX_DRONE_NAME_LENGTH);
                ++drone_infos_idx;
            }
        }

        serial_send_info_complete(s_send_info);
        
    } break;

    case SPKT_M2C_PING_DRONE:
    {
        SPacket_M2C_Ping_Drone* spkt_m2c_ping = (SPacket_M2C_Ping_Drone*) spkt;
        
        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_ping->drone_name);
        if (drone_idx < 0) break;
        
        if (!s_drone_infos[drone_idx].connected) {
            LOG_ERROR("Can't ping drone because the drone is not connected.");
            send_c2m_ping_drone_result(drone_idx, false);
            break; 
        }

        ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(drone_idx, EPKT_C2D_PING, sizeof(EPacket_C2D_Ping), EPKT_FLAG_REQUIRE_ACK);
        if (e_send_info) {
            EPacket_C2D_Ping* epkt_ping = (EPacket_C2D_Ping*) &e_send_info->epkt;
            epkt_ping->notify_master = true;
            espnow_send_info_complete(e_send_info);
        }
        
    } break;

    case SPKT_M2C_RESTART_DRONE:
    {
        SPacket_M2C_Restart_Drone* spkt_m2c_restart = (SPacket_M2C_Restart_Drone*) spkt;
        
        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_restart->drone_name);
        if (drone_idx < 0) break;
        
        if (!s_drone_infos[drone_idx].connected) {
            LOG_ERROR("Can't restart drone because the drone is not connected.");
            break; 
        }

        ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(drone_idx, EPKT_C2D_RESTART, sizeof(EPacket_C2D_Restart), EPKT_FLAG_REQUIRE_ACK);
        if (e_send_info) {
            espnow_send_info_complete(e_send_info);
        }
        
    } break;
    
    default:
        LOG_DEBUG("Received Serial Packet with unexpected or unknown type: '%u'", spkt->header.type);
    }
}

static void ping_connected_drones()
{
    int64_t time_us = esp_timer_get_time();

    for (int drone_idx = 0; drone_idx < MAX_DRONES; ++drone_idx) {
        if (s_drone_infos[drone_idx].connected) {
            if (time_us - s_drone_infos[drone_idx].last_ping_time_us > PING_CONNECTED_DRONE_INTERVAL_MS * 1000)
            {
                ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(drone_idx, EPKT_C2D_PING, sizeof(EPacket_C2D_Ping), EPKT_FLAG_REQUIRE_ACK);
                if (e_send_info) {
                    EPacket_C2D_Ping* epkt_ping = (EPacket_C2D_Ping*) &e_send_info->epkt;
                    epkt_ping->notify_master = false;
                    espnow_send_info_complete(e_send_info);
                }
                s_drone_infos[drone_idx].last_ping_time_us = time_us;
            }
        }
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

        esp_err_t ret = esp_now_send(e_send_info->mac_addr, (uint8_t*) epkt, espnow_packet_size(epkt->header));
        if (ret == ESP_OK)
        {
            // Record the time this packet was sent
            e_send_info->last_send_time_us = esp_timer_get_time();

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
                    LOG_DEBUG("send_all_queued_espnow_packets: Failed to add ESPNow_Send_Info to the 'wait for ack array', "
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
        
        int64_t time_us = esp_timer_get_time();

        if (e_send_info->resend_count >= ESPNOW_WAIT_FOR_ACK_RESEND_TIMEOUTS_MS_LEN)
        {
            int drone_idx = get_drone_idx_from_mac_addr(e_send_info->mac_addr);
            if (drone_idx >= 0) {
                s_drone_infos[drone_idx].responding = false;
            }
            process_espnow_packet_failed_acknowledge(e_send_info);
            memset(e_send_info, 0, MAX_ESPNOW_SEND_INFO_SIZE);
        }
        else if (time_us - e_send_info->last_send_time_us > espnow_wait_for_ack_resend_timeouts_ms[e_send_info->resend_count] * 1000)
        {
            esp_err_t ret = esp_now_send(e_send_info->mac_addr, (uint8_t*) &e_send_info->epkt, espnow_packet_size(e_send_info->epkt.header));
            if (ret == ESP_OK) {
                e_send_info->last_send_time_us = time_us;
                e_send_info->resend_count += 1;
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

static void send_all_queued_serial_packets()
{
    while (1)
    {
        size_t s_send_info_size;
        Serial_Send_Info* s_send_info = (Serial_Send_Info*) xRingbufferReceive(s_serial_send_info_ringbuf, &s_send_info_size, pdMS_TO_TICKS(0));
        if (!s_send_info) break;

        SPacket* spkt = &s_send_info->spkt;
        size_t spkt_size = serial_packet_size(spkt->header);

        int bytes_written = usb_serial_jtag_write_bytes(spkt, spkt_size, USB_SERIAL_WRITE_MAX_TICKS_TO_WAIT);
        if (bytes_written != spkt_size) {
            LOG_ERROR("Failed to write packet to the serialport");
        }

        vRingbufferReturnItem(s_serial_send_info_ringbuf, s_send_info);
    }
}

int app_main(void)
{
    dongle_init();

    while (1)
    {
        size_t e_recv_info_size;
        ESPNow_Receive_Info* e_recv_info = xRingbufferReceive(s_espnow_receive_info_ringbuf, &e_recv_info_size, pdMS_TO_TICKS(0));
        if (e_recv_info) {
            // TODO: check that e_recv_info_size matches the packet size
            process_incoming_espnow_packet(e_recv_info);
            vRingbufferReturnItem(s_espnow_receive_info_ringbuf, e_recv_info);
        }

        if (usb_serial_jtag_is_connected()) {
            bool success = read_next_serial_packet((SPacket*) &s_serial_packet_receive_buffer, pdMS_TO_TICKS(0));
            if (success) {
                process_incoming_serial_packet((SPacket*) &s_serial_packet_receive_buffer);
                memset(&s_serial_packet_receive_buffer, 0, SPKT_MAX_SIZE);
            }
        }

        ping_connected_drones();
        
        send_all_queued_espnow_packets();

        if (usb_serial_jtag_is_connected()) {
            send_all_queued_serial_packets();
        }

        // TODO: this delay is necessary because we need to yield for other tasks
        // and also give some time to lower priority tasks. We should actually block
        // on a queue in this task instead, but we cant because we have two queues, that we would
        // have to block on, so they should probably be unified or the task should be split.
        vTaskDelay(1);
    }

    esp_restart();
}
