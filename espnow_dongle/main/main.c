#include "espnow_packets.h"
#include "serial_packets.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_random.h"

#include "driver/usb_serial_jtag.h"

#define MAX_DRONES 128
#define MAX_ESPNOW_PACKETS_WAITING_FOR_ACK 10

#define ESPNOW_RECEIVE_INFO_QUEUE_SIZE 10
#define ESPNOW_SEND_INFO_QUEUE_SIZE 10
#define SERIAL_SEND_INFO_QUEUE_SIZE 10

#define USB_SERIAL_JTAG_BUFFER_SIZE 1024

#define USB_SERIAL_WRITE_MAX_TICKS_TO_WAIT pdMS_TO_TICKS(10)
#define USB_SERIAL_READ_MAX_TICKS_TO_WAIT  pdMS_TO_TICKS(10)

#define ESPNOW_MAXDELAY portMAX_DELAY
#define ESPNOW_RECEIVE_INFO_QUEUE_SEND_DELAY pdMS_TO_TICKS(10)
#define ESPNOW_SEND_INFO_QUEUE_SEND_DELAY pdMS_TO_TICKS(10)
#define SERIAL_SEND_INFO_QUEUE_SEND_DELAY pdMS_TO_TICKS(10)

#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP

/* Macros for logging. Because stdout is set to Serial/JTAG,
   printf will just insert the text in our outgoing
   serial-communication. The special delimiter signals that
   the next bytes will be text instead of a packet. */

#define DGL_LOG_ERROR(format, ...) printf(SERIAL_LOG_MSG_DELIMITER_STRING " " format "\n", ##__VA_ARGS__)

#define ENABLE_DEBUG_LOGS 1

#if ENABLE_DEBUG_LOGS
#    define DGL_LOG_DEBUG(format, ...) printf(SERIAL_LOG_MSG_DELIMITER_STRING " " format "\n", ##__VA_ARGS__)
#else
#    define DGL_LOG_DEBUG(...)
#endif

#define DGL_ASSERT(condition)                                           \
    do {                                                                \
        if (!(condition)) {                                             \
            DGL_LOG_ERROR("ASSERTION FAILED:%s:%d", __FILE__, __LINE__); \
            abort();                                                    \
        }                                                               \
    } while (0);

#define DGL_ABORT(...)                          \
    do {                                        \
        DGL_LOG_ERROR(__VA_ARGS__);             \
        abort();                                \
    } while (0);

/* like malloc, but aborts when out of memory */
static inline void* __dgl_malloc_impl(size_t bytes, const char* file_name, int line) {
    void* memory = malloc(bytes);
    if (memory == NULL) {
        DGL_LOG_ERROR("%s:%d: Failed to allocated memory!", file_name, line);
        abort();
    }
    return memory;
}

#define DGL_MALLOC(bytes) __dgl_malloc_impl(bytes, __FILE__, __LINE__)

/* Special mac address for broadcasting to all devices in range */
static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *packet_data;
    uint16_t packet_data_length;
} ESPNow_Receive_Info;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    EPacket* epkt;
} ESPNow_Send_Info;

typedef struct {
    SPacket* spkt;
} Serial_Send_Info;

typedef struct {
    bool empty;
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    char drone_name[MAX_DRONE_NAME_LENGTH];
    bool connected;
} Drone_Info;

/*
 * Static global variables
 */

static QueueHandle_t s_espnow_receive_info_queue = NULL;
static QueueHandle_t s_espnow_send_info_queue    = NULL;
static QueueHandle_t s_serial_send_info_queue    = NULL;

Drone_Info s_drone_infos[MAX_DRONES] = {0};
ESPNow_Send_Info s_espnow_send_infos_waiting_for_ack[MAX_ESPNOW_PACKETS_WAITING_FOR_ACK] = {0};

/*
 * Implementation
 */

static void nvs_init()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (nvs_flash_erase() != ESP_OK) DGL_ABORT("init_nvs: nvs_flash_erase failed");
        ret = nvs_flash_init();
    }
    if(ret != ESP_OK) DGL_ABORT("init_nvs: nvs_flash_init failed");
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
    esp_now_peer_info_t *peer = DGL_MALLOC(sizeof(esp_now_peer_info_t));
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = 0;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_broadcast_mac, sizeof(s_broadcast_mac));
    
    if (esp_now_add_peer(peer) != ESP_OK) DGL_ABORT("init_espnow: esp_now_add_peer failed");
    
    free(peer);
}

static void usb_serial_jtag_init()
{
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = USB_SERIAL_JTAG_BUFFER_SIZE,
        .tx_buffer_size = USB_SERIAL_JTAG_BUFFER_SIZE,
    };

    if (usb_serial_jtag_driver_install(&usb_serial_jtag_config) != ESP_OK) {
        DGL_ABORT("init_usb_serial_jtag: usb_serial_jtag_driver_install failed");
    }
}

static void dongle_init()
{
    nvs_init();
    wifi_init(false);
    espnow_init();
    usb_serial_jtag_init();

    s_espnow_receive_info_queue = xQueueCreate(ESPNOW_RECEIVE_INFO_QUEUE_SIZE, sizeof(ESPNow_Receive_Info));
    s_espnow_send_info_queue    = xQueueCreate(ESPNOW_SEND_INFO_QUEUE_SIZE, sizeof(ESPNow_Send_Info));
    s_serial_send_info_queue    = xQueueCreate(SERIAL_SEND_INFO_QUEUE_SIZE, sizeof(Serial_Send_Info));

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

static void espnow_receive_info_destroy(ESPNow_Receive_Info* e_recv_info)
{
    free(e_recv_info->packet_data);
    e_recv_info->packet_data = NULL;
    e_recv_info->packet_data_length = 0;
}

static void espnow_receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int length)
{
    if (recv_info == NULL || recv_info->src_addr == NULL || data == NULL || length <= 0) {
        DGL_LOG_ERROR("espnow_receive_callback: arg error");
        return;
    }
    
    ESPNow_Receive_Info e_recv_info = {0};
    memcpy(e_recv_info.mac_addr, recv_info->src_addr, ESP_NOW_ETH_ALEN);
    e_recv_info.packet_data = DGL_MALLOC(length);
    memcpy(e_recv_info.packet_data, data, length);
    e_recv_info.packet_data_length = length;
    
    if (!xQueueSend(s_espnow_receive_info_queue, &e_recv_info, ESPNOW_RECEIVE_INFO_QUEUE_SEND_DELAY)) {
        DGL_LOG_DEBUG("espnow_receive_callback: Sending espnow receive info to queue failed. The queue might be full.");
        espnow_receive_info_destroy(&e_recv_info);
    }
}

static int get_drone_idx_from_mac_addr(uint8_t* mac_addr)
{
    for (int i = 0; i < MAX_DRONES; ++i) {
        if (memcmp(&s_drone_infos[i].mac_addr, mac_addr, ESP_NOW_ETH_ALEN) == 0) {
            return i;
        }
    }
    
    DGL_LOG_DEBUG("No esp indexed with MAC: '"MACSTR"'. Send PKT_D2C_ADVERTISE for your esp to be indexed.", MAC2STR(mac_addr));
    return -1;
}

static int get_drone_idx_from_drone_name(char* drone_name)
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

static EPacket* espnow_packet_create(EPacket_Type type, size_t packet_size, EPacket_Flags flags)
{
    assert(packet_size >= sizeof(EPacket));
    
    EPacket* epkt = DGL_MALLOC(packet_size);
    epkt->header.type = type;
    epkt->header.flags = flags;
    epkt->header.packet_id = esp_random();
    epkt->header.payload_size = packet_size - sizeof(EPacket);
    
    return epkt;
}

static SPacket* serial_packet_create(SPacket_Type type, size_t packet_size)
{
    assert(packet_size >= sizeof(SPacket));
    
    SPacket* spkt = DGL_MALLOC(packet_size);
    spkt->header.delim = SERIAL_DELIMITER;
    spkt->header.type = type;
    spkt->header.payload_size = packet_size - sizeof(SPacket);
    
    return spkt;
}

static void espnow_packet_destroy(EPacket** epkt)
{
    free(*epkt);
    *epkt = NULL;
}

static void serial_packet_destroy(SPacket** spkt)
{
    free(*spkt);
    *spkt = NULL;
}

static ESPNow_Send_Info espnow_send_info_create(EPacket* epkt, int dest_drone_idx)
{
    ESPNow_Send_Info e_send_info = {0};
    memcpy(e_send_info.mac_addr, s_drone_infos[dest_drone_idx].mac_addr, ESP_NOW_ETH_ALEN);
    e_send_info.epkt = epkt;
    return e_send_info;
}

static void espnow_send_info_add_to_send_queue(ESPNow_Send_Info e_send_info)
{
    if (!xQueueSend(s_espnow_send_info_queue, &e_send_info, ESPNOW_SEND_INFO_QUEUE_SEND_DELAY)) {
        DGL_LOG_DEBUG("espnow_send_info_add_to_send_queue: Sending espnow send info to queue failed. The queue might be full.");
        espnow_packet_destroy(&e_send_info.epkt);
    }
}

static void serial_send_info_add_to_send_queue(Serial_Send_Info s_send_info)
{
    if (!xQueueSend(s_serial_send_info_queue, &s_send_info, SERIAL_SEND_INFO_QUEUE_SEND_DELAY)) {
        DGL_LOG_DEBUG("serial_send_info_add_to_send_queue: Sending serial send info to queue failed. The queue might be full.");
        serial_packet_destroy(&s_send_info.spkt);
    }    
}

void process_incoming_espnow_packet(ESPNow_Receive_Info recv_info)
{
    EPacket* epkt = (EPacket*) recv_info.packet_data;

    switch (epkt->header.type) {
        
    case EPKT_D2C_ADVERTISE:
    {
        EPacket_D2C_Advertise* epkt_advertise = (EPacket_D2C_Advertise*) epkt;

        // check if the drone is already indexed, and if not find the first empty index
        int first_empty_idx = -1;
        bool drone_already_indexed = false;
        for (int i = 0; i < MAX_DRONES; ++i) {
            if (first_empty_idx < 0 && s_drone_infos[i].empty) {
                first_empty_idx = i;
            }
            if (memcmp(&s_drone_infos[i].mac_addr, recv_info.mac_addr, ESP_NOW_ETH_ALEN) == 0) {
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
            s_drone_infos[first_empty_idx].empty = false;
            memcpy(&s_drone_infos[first_empty_idx].mac_addr, recv_info.mac_addr, ESP_NOW_ETH_ALEN);
            memcpy(&s_drone_infos[first_empty_idx].drone_name, epkt_advertise->drone_name, MAX_DRONE_NAME_LENGTH);
            s_drone_infos[first_empty_idx].connected = false;
        }
        
    } break;
        
    case EPKT_D2C_ACCEPT_CONNECTION:
    {
        int drone_idx = get_drone_idx_from_mac_addr(recv_info.mac_addr);
        if (drone_idx < 0) break;

        s_drone_infos[drone_idx].connected = true;
        
        SPacket_C2M_Drone_Connected* spkt_connected = (SPacket_C2M_Drone_Connected*)
            serial_packet_create(SPKT_C2M_DRONE_CONNECTED, sizeof(SPacket_C2M_Drone_Connected));
        memcpy(&spkt_connected->drone_name, s_drone_infos[drone_idx].drone_name, MAX_DRONE_NAME_LENGTH);
        
        Serial_Send_Info s_send_info = { (SPacket*) spkt_connected };
        serial_send_info_add_to_send_queue(s_send_info);
        
    } break;
        
    case EPKT_D2C_ACCEPT_DISCONNECT:
    {
        int drone_idx = get_drone_idx_from_mac_addr(recv_info.mac_addr);
        if (drone_idx < 0) break;

        s_drone_infos[drone_idx].connected = false;

        SPacket_C2M_Drone_Disconnected* spkt_disconnected = (SPacket_C2M_Drone_Disconnected*)
            serial_packet_create(SPKT_C2M_DRONE_DISCONNECTED, sizeof(SPacket_C2M_Drone_Disconnected));
        memcpy(&spkt_disconnected->drone_name, s_drone_infos[drone_idx].drone_name, MAX_DRONE_NAME_LENGTH);
        
        Serial_Send_Info s_send_info = { (SPacket*) spkt_disconnected };
        serial_send_info_add_to_send_queue(s_send_info);
        
    } break;

    case EPKT_D2C_ACK_RECEIVE:
    {
        int drone_idx = get_drone_idx_from_mac_addr(recv_info.mac_addr);
        if (drone_idx < 0) break;

        // CONTINUE HERE !!!!!!!!!!
        
    } break;
        
    case EPKT_D2C_IMU_DATA:
    {
        EPacket_D2C_Imu_Data* epkt_d2c_imu_data = (EPacket_D2C_Imu_Data*) epkt;
        
        int drone_idx = get_drone_idx_from_mac_addr(recv_info.mac_addr);
        if (drone_idx < 0) break;

        SPacket_C2M_Drone_Imu_Data* spkt_c2m_imu_data = (SPacket_C2M_Drone_Imu_Data*) serial_packet_create(SPKT_C2M_DRONE_IMU_DATA, sizeof(SPacket_C2M_Drone_Imu_Data));
        memcpy(&spkt_c2m_imu_data->imu_data, &epkt_d2c_imu_data->imu_data, sizeof(Payload_Imu_Data));
        memcpy(&spkt_c2m_imu_data->drone_name, s_drone_infos[drone_idx].drone_name, MAX_DRONE_NAME_LENGTH);
        
        Serial_Send_Info s_send_info = { (SPacket*) spkt_c2m_imu_data };
        serial_send_info_add_to_send_queue(s_send_info);
        
    } break;
        
    default:
        DGL_LOG_DEBUG("Received ESP-Now Packet with unexpected or unknown type: '%d'", epkt->header.type);
    }
}

static SPacket* read_next_serial_packet(TickType_t ticks_to_wait)
{
    /* First checks for the delimiter then reads the next header which is used to create
       the serial packet in memory. Then the payload of the packet is read.
       Remember: packet = header + payload */

    SHeader s_header = {0};
    int bytes_read_header = usb_serial_jtag_read_bytes(&s_header, sizeof(SHeader), ticks_to_wait);
    if (bytes_read_header != sizeof(SHeader)) {
        return NULL;
    }

    if (s_header.delim != SERIAL_DELIMITER) {
        DGL_LOG_ERROR("Failed to read serial packet: Delimiter was not at the expected location");
        // TODO: 'skip to next delimiter'
        return NULL;
    }

    SPacket* spkt = serial_packet_create(s_header.type, serial_packet_size(s_header));
    int bytes_read_payload = usb_serial_jtag_read_bytes(spkt->payload, spkt->header.payload_size, ticks_to_wait);
    
    if (bytes_read_payload != spkt->header.payload_size) {
        DGL_LOG_ERROR("Failed to read payload of serial packet: Not enough data available, but timeout reached.");
        return NULL;
    }

    return spkt;
}

void process_incoming_serial_packet(SPacket* spkt)
{
    switch (spkt->header.type) {
        
    case SPKT_M2C_CONNECT_TO_DRONE:
    {
        SPacket_M2C_Connect_To_Drone* spkt_m2c_connect = (SPacket_M2C_Connect_To_Drone*) spkt;
        
        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_connect->drone_name);
        if (drone_idx < 0) break;
        
        if (s_drone_infos[drone_idx].connected == false)
        {
            EPacket_C2D_Connect* epkt_connect = DGL_MALLOC(sizeof(EPacket_C2D_Connect));
            epkt_connect->header = espnow_header_create(EPKT_C2D_CONNECT, sizeof(EPacket_C2D_Connect), EPKT_FLAG_REQUIRE_ACK);
            
            ESPNow_Send_Info e_send_info = espnow_send_info_create((EPacket*) epkt_connect, drone_idx);
            espnow_send_info_add_to_send_queue(e_send_info);
        }
        else {
            DGL_LOG_DEBUG("Master requested to connect to drone, even though it was already connected.");
        }
        
    } break;
    
    case SPKT_M2C_DISCONNECT_FROM_DRONE:
    {
        SPacket_M2C_Disconnect_From_Drone* spkt_m2c_disconnect = (SPacket_M2C_Disconnect_From_Drone*) spkt;
        
        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_disconnect->drone_name);
        if (drone_idx < 0) break;
        
        if (s_drone_infos[drone_idx].connected == true)
        {
            EPacket_C2D_Disconnect* epkt_disconnect = (EPacket_C2D_Disconnect*)
                espnow_packet_create(EPKT_C2D_DISCONNECT, sizeof(EPacket_C2D_Disconnect), EPKT_FLAG_REQUIRE_ACK);
            ESPNow_Send_Info e_send_info = espnow_send_info_create((EPacket*) epkt_disconnect, drone_idx);
            espnow_send_info_add_to_send_queue(e_send_info);
        }
        else {
            DGL_LOG_DEBUG("Master requested to disconnect from drone, even though it was already disconnected.");
        }
        
    } break;
    
    case SPKT_M2C_CONTROLL_COMMANDS:
    {
        SPacket_M2C_Controll_Commands* spkt_m2c_ctrl_cmds = (SPacket_M2C_Controll_Commands*) spkt;
        
        int drone_idx = get_drone_idx_from_drone_name(spkt_m2c_ctrl_cmds->drone_name);
        if (drone_idx < 0) break;

        EPacket_C2D_Controll_Commands* epkt_c2d_ctrl_cmds = (EPacket_C2D_Controll_Commands*)
            espnow_packet_create(EPKT_C2D_CONTROLL_COMMANDS, sizeof(EPacket_C2D_Controll_Commands), EPKT_FLAG_REQUIRE_ACK);
        memcpy(&epkt_c2d_ctrl_cmds->ctrl_cmds, &spkt_m2c_ctrl_cmds->ctrl_cmds, sizeof(Payload_Controll_Commands));
        
        ESPNow_Send_Info e_send_info = espnow_send_info_create((EPacket*) epkt_c2d_ctrl_cmds, drone_idx);
        espnow_send_info_add_to_send_queue(e_send_info);
        
    } break;
    
    case SPKT_M2C_GET_DRONE_INFOS:
    {
        size_t drone_infos_count = 0;
        for (int i = 0; i < MAX_DRONES; ++i) {
            if (!s_drone_infos[i].empty) ++drone_infos_count;
        }

        int drone_infos_size = sizeof(SPacket_C2M_Drone_Infos) + drone_infos_count * sizeof(Payload_Drone_Info);
        SPacket_C2M_Drone_Infos* spkt_c2m_drone_infos = (SPacket_C2M_Drone_Infos*) serial_packet_create(SPKT_C2M_DRONE_INFOS, drone_infos_size);
        spkt_c2m_drone_infos->drone_infos_count = drone_infos_count;

        int drone_infos_idx = 0;
        for (int i = 0; i < MAX_DRONES; ++i) {
            if (!s_drone_infos[i].empty) {
                spkt_c2m_drone_infos->drone_infos[drone_infos_idx].connected = s_drone_infos[i].connected;
                memcpy(spkt_c2m_drone_infos->drone_infos[drone_infos_idx].drone_name, s_drone_infos[i].drone_name, MAX_DRONE_NAME_LENGTH);
                ++drone_infos_idx;
            }
        }

        Serial_Send_Info s_send_info = { (SPacket*) spkt_c2m_drone_infos };
        serial_send_info_add_to_send_queue(s_send_info);
        
    } break;
    
    default:
        DGL_LOG_DEBUG("Received Serial Packet with unexpected or unknown type: '%d'", spkt->header.type);
    }
}

void send_all_queued_espnow_packets()
{
    ESPNow_Send_Info e_send_info = {0};
    while (xQueueReceive(s_espnow_send_info_queue, &e_send_info, pdMS_TO_TICKS(0)))
    {
        EPacket* epkt = e_send_info.epkt;
        DGL_ASSERT(epkt);
        
        esp_err_t ret = esp_now_send(e_send_info.mac_addr, (uint8_t*) epkt, espnow_packet_size(epkt->header));
        if (ret != ESP_OK) {
            DGL_LOG_DEBUG("send_all_queued_espnow_packets: esp_now_send failed");
            // return send_info to the queue
            bool success = xQueueSend(s_espnow_send_info_queue, &e_send_info, pdMS_TO_TICKS(0));
            DGL_ASSERT(success);
            // When esp_now_send failed once, we shouldn't immeditely try to send more,
            // instead we return to continue where we left of the next time.
            return;
        }
        
        if (epkt->header.flags & EPKT_FLAG_REQUIRE_ACK) {
            // find empty slot in the wait for ack array
            int first_empty_idx = 0;
            for (; first_empty_idx < MAX_ESPNOW_PACKETS_WAITING_FOR_ACK; ++first_empty_idx) {
                if (s_espnow_send_infos_waiting_for_ack[first_empty_idx].epkt == NULL) break;
            }

            if (first_empty_idx < MAX_ESPNOW_PACKETS_WAITING_FOR_ACK) {
                s_espnow_send_infos_waiting_for_ack[first_empty_idx] = e_send_info;
                continue;
            }
            else {
                DGL_LOG_DEBUG("send_all_queued_espnow_packets: Failed to add ESPNow Send Info to the 'wait for ack array', because all '%d' slots are full",
                    MAX_ESPNOW_PACKETS_WAITING_FOR_ACK);
            }
        }

        espnow_packet_destroy(&epkt);
    }
}

void send_all_queued_serial_packets()
{
    Serial_Send_Info s_send_info = {0};
    while (xQueueReceive(s_serial_send_info_queue, &s_send_info, pdMS_TO_TICKS(0)))
    {
        SPacket* spkt = s_send_info.spkt;
        DGL_ASSERT(spkt);

        size_t spkt_size = serial_packet_size(spkt->header);
        int bytes_written = usb_serial_jtag_write_bytes(spkt, spkt_size, 0);
        
        if (bytes_written == 0) {
            // return send_info to the queue
            bool success = xQueueSend(s_serial_send_info_queue, &s_send_info, pdMS_TO_TICKS(0));
            DGL_ASSERT(success);
            // When writing bytes to the serial port failed once, we shouldn't immeditely try to
            // send more, instead we return to continue where we left of the next time.
            return;
        }
        else {
            DGL_ASSERT(bytes_written == spkt_size);            
        }
    }
}

int app_main(void)
{
    dongle_init();

    while (1)
    {
        ESPNow_Receive_Info e_recv_info = {0};
        if (xQueueReceive(s_espnow_receive_info_queue, &e_recv_info, 0)) {
            process_incoming_espnow_packet(e_recv_info);
            espnow_receive_info_destroy(&e_recv_info);
        }

        if (usb_serial_jtag_is_connected()) {
            SPacket* spkt = read_next_serial_packet(pdMS_TO_TICKS(0));
            if (spkt) {
                process_incoming_serial_packet(spkt);
                serial_packet_destroy(&spkt);
            }
        }
        
        send_all_queued_espnow_packets();

        if (usb_serial_jtag_is_connected()) {
            send_all_queued_serial_packets();
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    esp_restart();
}
