#include "espnow_packets.h"
#include "serial_packets.h"

#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_random.h"
#include "sys/time.h"

#include "driver/usb_serial_jtag.h"

// Static Array sizes
#define MAX_DRONES 128
#define MAX_LOG_MSG_LENGTH 1024

// Buffer sizes (the packets life in these buffers, so they should be rather big)
#define ESPNOW_RECEIVE_INFO_RINGBUF_SIZE (1024 * 10)
#define ESPNOW_SEND_INFO_RINGBUF_SIZE (1024 * 10)
#define SERIAL_SEND_INFO_RINGBUF_SIZE (1024 * 10)
#define USB_SERIAL_JTAG_IN_BUFFER_SIZE (1024 * 10)
#define USB_SERIAL_JTAG_OUT_BUFFER_SIZE (1024 * 10)

// Packet Acknowledge system
#define MAX_ESPNOW_PACKETS_WAITING_FOR_ACK 10
#define ESPNOW_WAIT_FOR_ACK_RESEND_TIMEOUT_MS 100

// Max Latency on blocking ops
#define USB_SERIAL_WRITE_MAX_TICKS_TO_WAIT pdMS_TO_TICKS(10)
#define USB_SERIAL_WRITE_LOG_MAX_TICKS_TO_WAIT pdMS_TO_TICKS(1000) // logs are important
#define USB_SERIAL_READ_MAX_TICKS_TO_WAIT  pdMS_TO_TICKS(10)
#define ESPNOW_RECEIVE_INFO_RINGBUF_MAX_TICKS_TO_WAIT pdMS_TO_TICKS(10)
#define ESPNOW_SEND_INFO_RINGBUF_MAX_TICKS_TO_WAIT pdMS_TO_TICKS(10)
#define SERIAL_SEND_INFO_RINGBUF_MAX_TICKS_TO_WAIT pdMS_TO_TICKS(10)

// ESP-Now Settings
#define ESPNOW_WIFI_MODE WIFI_MODE_AP // The dongle should be the Access Point that the drones connect to
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP // We want to use the Access Point interface (IF)

static void serial_send_log(const char* format, ...);

#define DGL_LOG_ERROR(format, ...) serial_send_log("ERROR (DONGLE): " format "\n", ##__VA_ARGS__)

#define ENABLE_DEBUG_LOGS 1

#if ENABLE_DEBUG_LOGS
#    define DGL_LOG_DEBUG(format, ...) serial_send_log("DEBUG (DONGLE): " format "\n", ##__VA_ARGS__)
#else
#    define DGL_LOG_DEBUG(...)
#endif

#define DGL_ASSERT(condition)                                           \
    do {                                                                \
        if (!(condition)) {                                             \
            DGL_LOG_ERROR("ASSERTION FAILED (DONGLE):%s:%d", __FILE__, __LINE__); \
            abort();                                                    \
        }                                                               \
    } while (0);

#define DGL_ABORT(...)                          \
    do {                                        \
        DGL_LOG_ERROR(__VA_ARGS__);             \
        abort();                                \
    } while (0);

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
    SPacket spkt;
} Serial_Send_Info;

typedef struct {
    bool occupied;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    char drone_name[MAX_DRONE_NAME_LENGTH];
    bool connected;
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

static void usb_serial_jtag_init()
{
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = USB_SERIAL_JTAG_IN_BUFFER_SIZE,
        .tx_buffer_size = USB_SERIAL_JTAG_OUT_BUFFER_SIZE,
    };

    if (usb_serial_jtag_driver_install(&usb_serial_jtag_config) != ESP_OK) {
        // Not using DGL_ABORT, becasue that one requires usb-serial/JTAG to log the error, instead just try to printf
        printf("ERROR: init_usb_serial_jtag: usb_serial_jtag_driver_install failed, aborting...");
        abort();
    }
}

static void nvs_init()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (nvs_flash_erase() != ESP_OK) DGL_ABORT("init_nvs: nvs_flash_erase failed");
        ret = nvs_flash_init();
    }
    if(ret != ESP_OK) DGL_ABORT("init_nvs: nvs_flash_init failed, ESP_ERR: %s", esp_err_to_name(ret));
}

static void wifi_init(bool long_range_mode)
{
    if (esp_netif_init() != ESP_OK)                DGL_ABORT("init_wifi: esp_netif_init failed");
    if (esp_event_loop_create_default() != ESP_OK) DGL_ABORT("init_wifi: esp_event_loop_create_default failed");
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    
    if (esp_wifi_init(&cfg) != ESP_OK)                 DGL_ABORT("init_wifi: esp_wifi_init failed");
    if (esp_wifi_set_mode(ESPNOW_WIFI_MODE) != ESP_OK) DGL_ABORT("init_wifi: esp_wifi_set_mode failed");
    if (esp_wifi_start() != ESP_OK)                    DGL_ABORT("init_wifi: esp_wifi_start failed");

    if (long_range_mode) {
        if (esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) != ESP_OK) {
            DGL_ABORT("init_wifi: esp_wifi_set_protocol failed");
        }
    }
}

static void espnow_send_callback(const esp_now_send_info_t *tx_info, esp_now_send_status_t status);
static void espnow_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int length);

static void espnow_init()
{
    if (esp_now_init() != ESP_OK)                                    DGL_ABORT("init_espnow: esp_now_init failed");
    if (esp_now_register_send_cb(espnow_send_callback) != ESP_OK)    DGL_ABORT("init_espnow: esp_now_register_send_cb failed");
    if (esp_now_register_recv_cb(espnow_receive_callback) != ESP_OK) DGL_ABORT("init_espnow: esp_now_register_recv_cb failed");
    
    // Adding Boradcast peer to peer list.
    esp_now_peer_info_t peer = {0};
    static const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    memcpy(&peer.peer_addr, &broadcast_mac, ESP_NOW_ETH_ALEN);
    peer.channel = 0;
    peer.ifidx = ESPNOW_WIFI_IF;
    
    if (esp_now_add_peer(&peer) != ESP_OK) {
        DGL_ABORT("init_espnow: esp_now_add_peer failed");
    }
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

    DGL_LOG_DEBUG("Dongle Initialized!");
}

static void espnow_send_callback(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    /* This callback is not really utilized, because even if 'status' is ESP_NOW_SEND_FAIL
       we won't send the packet again. Mainly because it's difficult to know
       which packet 'tx_info' refers to (trust me its difficult). */

    if (status != ESP_NOW_SEND_SUCCESS) {
        DGL_LOG_DEBUG("ESP-Now send callback reported that sending a packet failed.");
    }
}

static void espnow_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int length)
{
    if (recv_info == NULL || recv_info->src_addr == NULL || data == NULL || length <= 0) {
        DGL_LOG_ERROR("espnow_receive_callback: arg error");
        return;
    }

    ESPNow_Receive_Info* e_recv_info = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_espnow_receive_info_ringbuf,
                                             (void**) &e_recv_info,
                                             sizeof(ESPNow_Receive_Info) - sizeof(EPacket) + length,
                                             ESPNOW_RECEIVE_INFO_RINGBUF_MAX_TICKS_TO_WAIT);

    if (ret != pdTRUE) {
        DGL_LOG_DEBUG("espnow_receive_callback: Creating espnow receive info in ringbuffer failed. The ringbuffer might be full.");
        return;
    }

    memcpy(&e_recv_info->mac_addr, &recv_info->src_addr, ESP_NOW_ETH_ALEN);
    memcpy(&e_recv_info->epkt, data, length);
    // We should probably just remove the packet from the ringbuffer if this fails, but not sure if it ever would
    // do that in case this assert ever triggers.
    DGL_ASSERT(e_recv_info->epkt.header.payload_size == length - sizeof(EPacket));

    ret = xRingbufferSendComplete(s_espnow_receive_info_ringbuf, (void*) e_recv_info);
    if (ret != pdTRUE) {
        DGL_LOG_DEBUG("espnow_receive_callback: Completing espnow receive info in ringbuffer failed.");
    }
}

static int get_drone_idx_from_mac_addr(const uint8_t* mac_addr)
{
    for (int i = 0; i < MAX_DRONES; ++i) {
        if (memcmp(&s_drone_infos[i].mac_addr, mac_addr, ESP_NOW_ETH_ALEN) == 0) {
            return i;
        }
    }
    
    DGL_LOG_DEBUG("No esp indexed with MAC: '"MACSTR"'. Send PKT_D2C_ADVERTISE for your esp to be indexed.", MAC2STR(mac_addr));
    return -1;
}

static int get_drone_idx_from_drone_name(const char* drone_name)
{
    for (int i = 0; i < MAX_DRONES; ++i) {
        if (memcmp(&s_drone_infos[i].drone_name, drone_name, MAX_DRONE_NAME_LENGTH) == 0) {
            return i;
        }
    }
    
    DGL_LOG_DEBUG("No esp indexed with name: '%.*s'. Send PKT_D2C_ADVERTISE for your esp to be indexed.", MAX_DRONE_NAME_LENGTH, drone_name);
    return -1;
}

static size_t espnow_packet_size(EHeader e_header) { return sizeof(EHeader) + e_header.payload_size; }
static size_t serial_packet_size(SHeader s_header) { return sizeof(SHeader) + s_header.payload_size; }

static EHeader espnow_header_create(EPacket_Type type, size_t packet_size, EPacket_Flags flags)
{
    DGL_ASSERT(packet_size >= sizeof(EPacket));
    
    return (EHeader) {
        .type = type,
        .flags = flags,
        .packet_id = esp_random(),
        .payload_size = packet_size - sizeof(EPacket)
    };
}

static SHeader serial_header_create(SPacket_Type type, size_t packet_size)
{
    DGL_ASSERT(packet_size >= sizeof(SPacket));
    
    return (SHeader) {
        .delim = SERIAL_DELIMITER,
        .type = type,
        .payload_size = packet_size - sizeof(SPacket)
    };
}

static ESPNow_Send_Info* espnow_send_info_acquire(int dest_drone_idx, EPacket_Type type, size_t packet_size, EPacket_Flags flags)
{
    DGL_ASSERT(s_drone_infos[dest_drone_idx].occupied == true);

    ESPNow_Send_Info* e_send_info = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_espnow_send_info_ringbuf,
                                             (void**) &e_send_info,
                                             sizeof(ESPNow_Send_Info) - sizeof(EPacket) + packet_size,
                                             SERIAL_SEND_INFO_RINGBUF_MAX_TICKS_TO_WAIT);

    if (ret != pdTRUE) {
        DGL_LOG_DEBUG("espnow_send_info_acquire: Creating espnow send info in ringbuffer failed. The ringbuffer might be full.");
        return NULL;
    }
    
    memcpy(&e_send_info->mac_addr, &s_drone_infos[dest_drone_idx].mac_addr, ESP_NOW_ETH_ALEN);
    e_send_info->last_send_time_us = 0;
    e_send_info->epkt.header = espnow_header_create(type, packet_size, flags);

    return e_send_info;
}

static Serial_Send_Info* serial_send_info_acquire(SPacket_Type type, size_t packet_size)
{
    Serial_Send_Info* s_send_info = NULL;
    UBaseType_t ret = xRingbufferSendAcquire(s_serial_send_info_ringbuf,
                                             (void**) &s_send_info,
                                             packet_size,
                                             SERIAL_SEND_INFO_RINGBUF_MAX_TICKS_TO_WAIT);

    if (ret != pdTRUE) {
        DGL_LOG_DEBUG("serial_send_info_acquire: Creating serial send info in ringbuffer failed. The ringbuffer might be full.");
        return NULL;
    }

    s_send_info->spkt.header = serial_header_create(type, packet_size);

    return s_send_info;
}

static void serial_send_info_complete(const Serial_Send_Info* s_send_info)
{
    UBaseType_t ret = xRingbufferSendComplete(s_serial_send_info_ringbuf, (void*) s_send_info);
    
    if (ret != pdTRUE) {
        DGL_LOG_DEBUG("serial_send_info_complete: Completing serial send info in ringbuffer failed.");
    }
}

static void espnow_send_info_complete(const ESPNow_Send_Info* e_send_info)
{
    UBaseType_t ret = xRingbufferSendComplete(s_espnow_send_info_ringbuf, (void*) e_send_info);
    
    if (ret != pdTRUE) {
        DGL_LOG_DEBUG("espnow_send_info_complete: Completing espnow send info in ringbuffer failed.");
    }
}

static void serial_send_log(const char* format, ...)
{
    va_list args;

    // Allocate statically to reduce stress from calling this function in critical situations
    static char pkt_buffer[sizeof(SPacket_C2M_Log) + MAX_LOG_MSG_LENGTH];
    SPacket_C2M_Log* spkt_log = (SPacket_C2M_Log*) &pkt_buffer;

    va_start(args, format);
    int log_msg_len = vsnprintf(spkt_log->msg, MAX_LOG_MSG_LENGTH, format, args) + 1; // + 1 because '\0' is not counted
    va_end(args);    

    size_t packet_size = sizeof(SPacket_C2M_Log) + log_msg_len;
    spkt_log->header = serial_header_create(SPKT_C2M_LOG, packet_size);
    
    usb_serial_jtag_write_bytes(spkt_log, packet_size, USB_SERIAL_WRITE_LOG_MAX_TICKS_TO_WAIT);
    // Importantly we don't log any errors that might occur in this function!
}

static void process_espnow_packet_acknowledge(const ESPNow_Send_Info* e_send_info)
{
    EPacket* epkt = &e_recv_info->epkt;
    
    switch (epkt->header.type) {
        
    case EPKT_C2D_CONNECT:
    {
        int drone_idx = get_drone_idx_from_mac_addr(e_send_info->mac_addr);
        if (drone_idx < 0) break;

        // Do nothing when we are already connected
        if (s_drone_infos[drone_idx].connected == true) break;
        
        s_drone_infos[drone_idx].connected = true;

        Serial_Send_Info* s_send_info = serial_send_info_acquire(SPKT_C2M_DRONE_CONNECTED, sizeof(SPacket_C2M_Drone_Connected));
        if (!s_send_info) break;

        SPacket_C2M_Drone_Connected* spkt_connected = (SPacket_C2M_Drone_Connected*) &s_send_info->spkt;
        memcpy(&spkt_connected->drone_name, s_drone_infos[drone_idx].drone_name, MAX_DRONE_NAME_LENGTH);

        serial_send_info_complete(s_send_info);       
        
    } break;
        
    case EPKT_C2D_DISCONNECT:
    {
        int drone_idx = get_drone_idx_from_mac_addr(e_send_info->mac_addr);
        if (drone_idx < 0) break;

        // Do nothing when we are already disconnected
        if (s_drone_infos[drone_idx].connected == false) break;

        s_drone_infos[drone_idx].connected = false;

        // Try to remove the drone to the espnow peer list
        esp_err_t ret = esp_now_del_peer(&s_drone_infos[drone_idx].mac_addr);
        if (ret != ESP_OK) {
            DGL_LOG_DEBUG("process_espnow_packet_acknowledge: Failed to delete Drone from peer list, ESP_ERR: %s", esp_err_to_name(ret));
        }

        Serial_Send_Info* s_send_info = serial_send_info_acquire(SPKT_C2M_DRONE_DISCONNECTED, sizeof(SPacket_C2M_Drone_Disconnected));
        if (!s_send_info) break;

        SPacket_C2M_Drone_Disconnected* spkt_disconnected = (SPacket_C2M_Drone_Disconnected*) &s_send_info->spkt;
        memcpy(&spkt_disconnected->drone_name, s_drone_infos[drone_idx].drone_name, MAX_DRONE_NAME_LENGTH);

        serial_send_info_complete(s_send_info);
    } break;
        
    default:
        DGL_LOG_DEBUG("Received acknowledge for ESP-Now Packet of type '%d', for which we do not support ackowledges", epkt->header.type);
    }
}

static void process_incoming_espnow_packet(const ESPNow_Receive_Info* e_recv_info)
{
    EPacket* epkt = &e_recv_info->epkt;

    // If we need to send an ack, we should do that before sending any other espnow packets
    if (epkt->header.flags & EPKT_FLAG_REQUIRE_ACK)
    {
        int drone_idx = get_drone_idx_from_mac_addr(e_recv_info->mac_addr);
        if (drone_idx >= 0) {
            ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(drone_idx, EPKT_C2D_ACK_RECEIVE, sizeof(EPacket_C2D_Ack_Receive), 0);
            EPacket_C2D_Ack_Receive* epkt_c2d_ack = (EPacket_C2D_Ack_Receive*) e_send_info->epkt;
            epkt_c2d_ack->acknowledged_packet_id = epkt->header.packet_id;
        }
    }

    switch (epkt->header.type) {
        
    case EPKT_D2C_ADVERTISE:
    {
        EPacket_D2C_Advertise* epkt_advertise = (EPacket_D2C_Advertise*) epkt;

        // check if the drone is already indexed, and if not find the first empty index
        int first_empty_idx = -1;
        bool drone_already_indexed = false;
        for (int i = 0; i < MAX_DRONES; ++i) {
            if (first_empty_idx < 0 && !s_drone_infos[i].occupied) {
                first_empty_idx = i;
            }
            if (memcmp(&s_drone_infos[i].mac_addr, e_recv_info->mac_addr, ESP_NOW_ETH_ALEN) == 0) {
                drone_already_indexed = true;
                break;
            }
        }
        
        if (!drone_already_indexed) {
            // Check if there is an empty slot
            if (first_empty_idx < 0) {
                DGL_LOG_ERROR("Failed to index Drone after advertisement, because all (%d) drone slots are full.", MAX_DRONES);
                break;
            }
            
            // Check if the drone name is already taken
            int idx_by_name = get_drone_idx_from_drone_name(epkt_advertise->drone_name);
            if (idx_by_name > 0) {
                DGL_LOG_ERROR("Failed to index Drone after advertisement, because the name '%.*s' is already taken.",
                              MAX_DRONE_NAME_LENGTH, epkt_advertise->drone_name);
                break;
            }

            // Finally index the new drone
            s_drone_infos[first_empty_idx].occupied = true;
            memcpy(&s_drone_infos[first_empty_idx].mac_addr, e_recv_info->mac_addr, ESP_NOW_ETH_ALEN);
            memcpy(&s_drone_infos[first_empty_idx].drone_name, epkt_advertise->drone_name, MAX_DRONE_NAME_LENGTH);
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
            DGL_LOG_DEBUG("Received an EPKT_D2C_ACK_RECEIVE for packet '%d', but this packet was not in the 'wait for ack array'", acknowledged_packet_id);
        }
        
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
        memcpy(&spkt_c2m_imu_data->drone_name, s_drone_infos[drone_idx].drone_name, MAX_DRONE_NAME_LENGTH);

        serial_send_info_complete(s_send_info);
        
    } break;
        
    default:
        DGL_LOG_DEBUG("Received ESP-Now Packet with unexpected or unknown type: '%d'", epkt->header.type);
    }
}

static bool read_next_serial_packet(SPacket* spkt, TickType_t ticks_to_wait)
{
    /* First checks for the delimiter then reads the next header which is used to create
       the serial packet in memory. Then the payload of the packet is read.
       Remember: packet = header + payload */

    /* SHeader s_header = {0}; */
    int bytes_read_header = usb_serial_jtag_read_bytes(&spkt->header, sizeof(SHeader), ticks_to_wait);
    if (bytes_read_header != sizeof(SHeader)) {
        return false;
    }

    if (spkt->header.delim != SERIAL_DELIMITER) {
        DGL_LOG_ERROR("Failed to read serial packet: Delimiter was not at the expected location");
        // TODO: 'skip to next delimiter'
        return false;
    }

    int bytes_read_payload = usb_serial_jtag_read_bytes(spkt->payload, spkt->header.payload_size, ticks_to_wait);
    
    if (bytes_read_payload != spkt->header.payload_size) {
        DGL_LOG_ERROR("Failed to read payload of serial packet: Not enough data available, but timeout reached.");
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
        if (drone_idx < 0) break;

        // Already connected, do nothing.
        if (s_drone_infos[drone_idx].connected) {
            DGL_LOG_DEBUG("Master requested to connect to drone, even though it was already connected.");
            break; 
        }
        
        // Try to add the drone to the espnow peer list
        esp_now_peer_info_t peer = {0};
        memcpy(&peer.peer_addr, s_drone_infos[drone_idx].mac_addr, ESP_NOW_ETH_ALEN);
        peer.channel = 0;
        peer.ifidx = ESPNOW_WIFI_IF;
            
        esp_err_t ret = esp_now_add_peer(&peer);
        if (ret != ESP_OK) {
            DGL_LOG_ERROR("process_incoming_espnow_packet: Failed to add Drone to peer list, ESP_ERR: %s", esp_err_to_name(ret));
            break;
        }

        ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(drone_idx, EPKT_C2D_CONNECT, sizeof(EPacket_C2D_Connect), EPKT_FLAG_REQUIRE_ACK);
        if (!e_send_info) break;
            
        espnow_send_info_complete(e_send_info);
        
    } break;
    
    case SPKT_M2C_DISCONNECT_FROM_DRONE:
    {
        SPacket_M2C_Disconnect_From_Drone* spkt_m2c_disconnect = (SPacket_M2C_Disconnect_From_Drone*) spkt;
        
        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_disconnect->drone_name);
        if (drone_idx < 0) break;

        // Already not connected, do nothing.
        if (!s_drone_infos[drone_idx].connected) {
            DGL_LOG_DEBUG("Master requested to disconnect from drone, even though it was already disconnected.");
            break;
        }
        
        ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(drone_idx, EPKT_C2D_DISCONNECT, sizeof(EPacket_C2D_Disconnect), EPKT_FLAG_REQUIRE_ACK);
        if (!e_send_info) break;
            
        espnow_send_info_complete(e_send_info);
        
    } break;
    
    case SPKT_M2C_CONTROLL_COMMANDS:
    {
        SPacket_M2C_Controll_Commands* spkt_m2c_ctrl_cmds = (SPacket_M2C_Controll_Commands*) spkt;
        
        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_ctrl_cmds->drone_name);
        if (drone_idx < 0) break;

        ESPNow_Send_Info* e_send_info = espnow_send_info_acquire(drone_idx, EPKT_C2D_CONTROLL_COMMANDS, sizeof(EPacket_C2D_Controll_Commands), 0);
        if (!e_send_info) break;
        
        EPacket_C2D_Controll_Commands* epkt_c2d_ctrl_cmds = (EPacket_C2D_Controll_Commands*) &e_send_info->epkt;
        memcpy(&epkt_c2d_ctrl_cmds->ctrl_cmds, &spkt_m2c_ctrl_cmds->ctrl_cmds, sizeof(Payload_Controll_Commands));

        espnow_send_info_complete(e_send_info);
        
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
                spkt_c2m_drone_infos->drone_infos[drone_infos_idx].connected = s_drone_infos[i].connected;
                memcpy(spkt_c2m_drone_infos->drone_infos[drone_infos_idx].drone_name, s_drone_infos[i].drone_name, MAX_DRONE_NAME_LENGTH);
                ++drone_infos_idx;
            }
        }

        serial_send_info_complete(s_send_info);
        
    } break;
    
    default:
        DGL_LOG_DEBUG("Received Serial Packet with unexpected or unknown type: '%d'", spkt->header.type);
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
            
            DGL_LOG_DEBUG("send_all_queued_espnow_packets: esp_now_send failed");
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
                DGL_LOG_DEBUG("send_all_queued_espnow_packets: Failed to add ESPNow Send Info to the 'wait for ack array', because all '%d' slots are full",
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
                    DGL_LOG_DEBUG("send_all_queued_espnow_packets: esp_now_send failed");
                    // When esp_now_send failed once, we shouldn't immeditely try to send more,
                    // instead we return to continue where we left of the next time.
                    return;
                }

                e_send_info->last_send_time_us = time_us;
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

        // TODO: We are trying to send the packet until it goes through, because its difficult to put
        // it back in the ringbuffer for some reason. This will be less problematic, when this
        // function would just be a task, because it wouldn't block everything.
        
        int bytes_written = 0;
        while (bytes_written == 0) {
             bytes_written = usb_serial_jtag_write_bytes(spkt, spkt_size, 0);
        }
        DGL_ASSERT(bytes_written == spkt_size);

        vRingbufferReturnItem(s_serial_send_info_ringbuf, &s_send_info);
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
            process_incoming_espnow_packet(e_recv_info);
            vRingbufferReturnItem(s_espnow_receive_info_ringbuf, &e_recv_info);
        }

        if (usb_serial_jtag_is_connected()) {
            bool success = read_next_serial_packet((SPacket*) &s_serial_packet_receive_buffer, pdMS_TO_TICKS(0));
            if (success) {
                process_incoming_serial_packet((SPacket*) &s_serial_packet_receive_buffer);
                memset(&s_serial_packet_receive_buffer, 0, SPKT_MAX_SIZE);
            }
        }
        
        send_all_queued_espnow_packets();

        if (usb_serial_jtag_is_connected()) {
            send_all_queued_serial_packets();
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // for debugging
    }

    esp_restart();
}
