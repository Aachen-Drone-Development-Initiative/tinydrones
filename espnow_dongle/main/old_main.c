#include "espnow_packets.h"
#include "serial_packets.h"

#include <string.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "esp_log.h"
#include "esp_random.h"

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_select.h"

#define MAX_ESP_DEVICES (128)
#define MAX_EPKTS_WAITING_FOR_ACK (10)

#define ESPNOW_RECEIVE_CB_QUEUE_SIZE (10)
#define ESPNOW_PACKET_SEND_QUEUE_SIZE (10)
#define ESPNOW_PACKET_ACK_QUEUE_SIZE (10)
#define SERIAL_PACKET_SEND_QUEUE_SIZE (10)

#define USB_SERIAL_JTAG_BUFFER_SIZE (1024)
#define USB_SERIAL_WRITE_MAX_TICKS_TO_WAIT (20 / portTICK_PERIOD_MS)
#define USB_SERIAL_READ_MAX_TICKS_TO_WAIT  (20 / portTICK_PERIOD_MS)

#define XTASK_DEFAULT_STACK_DEPTH (100)
#define XTASK_DEFAULT_PRIORITY    (10)

#define ESPNOW_MAXDELAY (portMAX_DELAY)

#define TRY_TO_CONNECT_TO_ESP_SEND_DELAY  (100 / portTICK_PERIOD_MS)
#define TRY_TO_CONNECT_TO_ESP_TIMEOUT     (1000 / portTICK_PERIOD_MS)
#define TRY_TO_DISCONNECT_FROM_ESP_SEND_DELAY  (100 / portTICK_PERIOD_MS)
#define TRY_TO_DISCONNECT_FROM_ESP_TIMEOUT     (1000 / portTICK_PERIOD_MS)

#define DONGLE_MAIN_LOOP_DELAY (100 / portTICK_PERIOD_MS)

#define ESPNOW_WIFI_MODE (WIFI_MODE_AP)
#define ESPNOW_WIFI_IF   (ESP_IF_WIFI_AP)

#define SEND_LOGS_AS_SERIAL_PACKETS (0)
#define ENABLE_DBG_INFO_LOGS (1)



#if SEND_LOGS_AS_SERIAL_PACKETS
    #define DGL_ERROR(format, ...) send_c2m_log(LOG_ERROR, format, ##__VA_ARGS__)
    #if ENABLE_DBG_INFO_LOGS
        #define DGL_DBG_INFO(format, ...) send_c2m_log(LOG_DBG_INFO, format, ##__VA_ARGS__)
    #endif
#else
    #define DGL_ERROR(format, ...) ESP_LOGE("DONGLE", format, ##__VA_ARGS__)
    #if ENABLE_DBG_INFO_LOGS
        #define DGL_DBG_INFO(format, ...) ESP_LOGI("DONGLE", format, ##__VA_ARGS__)
    #endif
#endif

#ifndef DGL_DBG_INFO
    #define DGL_DBG_INFO(...)
#endif

#define DGL_ABORT(...)                                  \
    do {                                                \
        ESP_LOGE("DONGLE (FATAL ERROR)", __VA_ARGS__);  \
        abort();                                        \
    } while (0);

#define DGL_ASSERT(condition, format, ...)                              \
    do {                                                                \
    if (!(condition)) {                                                 \
        DGL_ERROR("DONGLE (ASSERTION FAILED):%s:%d:", function_name, line, ); \
        DGL_ABORT("DONGLE (ASSERTION FAILED)", ##__VA_ARGS__);          \
        abort();                                                        \
    }
    } while (0);

#define DGL_MALLOC(bytes) dgl_malloc(bytes, __func__, __LINE__)

static inline void* dgl_malloc(size_t bytes, const char* function_name, int line) {
    void* memory = malloc(bytes);
    if (memory == NULL) {
        DGL_ERROR("%s:%d: Failed to allocated memory!", function_name, line);
        abort();
    }
    return memory;
}

// Special mac address for broadcasting to all devices in range, not just a single device
static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint8_t s_zero_mac[ESP_NOW_ETH_ALEN]      = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *packet_data;
    uint16_t packet_data_length;
} ESPNow_Receive_Callback_Info;

static void destroy_espnow_receive_cb_info(ESPNow_Receive_Callback_Info recv_info) {
    free(recv_info.packet_data);
}

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    bool wait_for_acknowledge;
    EPacket* epkt;
} ESPNow_Send_Info;

typedef struct {
    SPacket* spkt;
} Serial_Send_Info;

typedef enum {
    ESP_DISCONNECTED = 0,
    ESP_TRY_TO_CONNECT,
    ESP_CONNECTED,
    ESP_TRY_TO_DISCONNECT,
} ESP_Connection_Status;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    char drone_name[MAX_DRONE_NAME_LENGTH];
    ESP_Connection_Status connection_status;
} ESP_Device;

/*
 * Static global variables
 */

static QueueHandle_t s_espnow_receive_cb_queue = NULL;
static QueueHandle_t s_espnow_packet_send_queue = NULL;
static QueueHandle_t s_serial_packet_send_queue = NULL;

static typedef struct {
    EPacket* items[MAX_EPKTS_WAITING_FOR_ACK];
    SemaphoreHandle_t mutex;
} s_array_epkts_waiting_for_ack = {0};

static typedef struct {
    ESP_Device items[MAX_ESP_DEVICES];
    SemaphoreHandle_t mutex;
} s_array_esp_devices = {0};

/*
 * Initialization
 */

static void init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (nvs_flash_erase() != ESP_OK) DGL_ABORT("init_nvs: nvs_flash_erase failed");
        ret = nvs_flash_init();
    }
    if(ret != ESP_OK) DGL_ABORT("init_nvs: nvs_flash_init failed");
}

static void init_wifi(bool long_range_mode)
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

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status);
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int length);

static void init_espnow()
{
    if (esp_now_init() != ESP_OK)                           DGL_ABORT("init_espnow: esp_now_init failed");
    if (esp_now_register_send_cb(espnow_send_cb) != ESP_OK) DGL_ABORT("init_espnow: esp_now_register_send_cb failed");
    if (esp_now_register_recv_cb(espnow_recv_cb) != ESP_OK) DGL_ABORT("init_espnow: esp_now_register_recv_cb failed");
    
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

static void init_usb_serial_jtag()
{
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = USB_SERIAL_JTAG_BUFFER_SIZE,
        .tx_buffer_size = USB_SERIAL_JTAG_BUFFER_SIZE,
    };

    if (usb_serial_jtag_driver_install(&usb_serial_jtag_config) != ESP_OK) DGL_ABORT("init_usb_serial_jtag: usb_serial_jtag_driver_install failed");
}

static void send_c2m_log(Log_Type type, const char* format, ...);

static void init_dongle()
{
    init_nvs();
    init_wifi(false);
    init_espnow();
    init_usb_serial_jtag();

    // Initializing static global variables
    s_espnow_receive_cb_queue = xQueueCreate(ESPNOW_RECEIVE_CB_QUEUE_SIZE, sizeof(ESPNow_Receive_Callback_Info));
    s_espnow_packet_send_queue = xQueueCreate(ESPNOW_PACKET_SEND_QUEUE_SIZE, sizeof(ESPNow_Send_Info));
    s_serial_packet_send_queue = xQueueCreate(SERIAL_PACKET_SEND_QUEUE_SIZE, sizeof(Serial_Send_Info));

    s_array_init(&s_array_epkts_waiting_for_ack);
    s_array_init(&s_array_esp_devices);

    DGL_DBG_INFO("Dongle Initialized!");
}

/*
 * ESP-Now send and receive callbacks...
 */

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    // This callback is not used, because even if 'status' is ESP_NOW_SEND_FAIL
    // we won't send the packet again. Mainly because it's difficult to know
    // which packet 'tx_info' refers to (trust me its difficult).

    if (status != ESP_NOW_SEND_SUCCESS) {
        DGL_DBG_INFO("Sending ESP-Now packet failed according to the send-callback.");
    }
    
    return;
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int length)
{
    ESPNow_Receive_Callback_Info recv_info;
    uint8_t* mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || length <= 0) {
        DGL_ERROR("espnow_recv_info: arg error");
        return;
    }

    memcpy(recv_info.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    
    recv_info.packet_data = DGL_MALLOC(length);
    memcpy(recv_info.packet_data, data, length);
    
    recv_info.packet_data_length = length;
    
    if (!xQueueSend(s_espnow_receive_cb_queue, &recv_info, ESPNOW_MAXDELAY)) {
        DGL_DBG_INFO("espnow_recv_cb: Pushing receive info to queue failed.");
        free(recv_info.packet_data);
    }
}

/*
 * MAC-Address/ESP-Device management
 */

static bool is_esp_mac_addr_indexed(uint8_t* mac_addr)
{
    for (int esp_idx = 0; esp_idx < MAX_ESP_DEVICES; ++esp_idx) {
        if (memcmp(&s_esp_devices[esp_idx].mac_addr, mac_addr, ESP_NOW_ETH_ALEN) == 0) {
            return true;
        }
    }
    return false;
}

static Esp_Idx get_esp_idx_from_mac_addr(uint8_t* mac_addr)
{
    for (int idx = 0; idx < MAX_ESP_DEVICES; ++idx) {
        if (memcmp(&s_esp_devices[idx].mac_addr, mac_addr, ESP_NOW_ETH_ALEN) == 0) {
            return idx;
        }
    }
    DGL_ERROR("No esp indexed/detected with MAC: "MACSTR". Send PKT_D2C_ADVERTISE for your esp to be indexed/detected.", MAC2STR(mac_addr));
    return INVALID_ESP_IDX;
}

static bool valid_esp_device_at_esp_idx(Esp_Idx esp_idx)
{
    if (!(esp_idx >= 0 && esp_idx < MAX_ESP_DEVICES)) {
        DGL_ERROR("ESP-Index '%d' is not inside the valid range of [0, %d]", esp_idx, MAX_ESP_DEVICES);
        return NULL;
    }

    if (memcmp(s_esp_devices[esp_idx].mac_addr, s_zero_mac, ESP_NOW_ETH_ALEN) == 0) {
        return false;
    }
    return true;
}

/*
 * ESPNow-/Serial Packet management
 */

static int espnow_packet_size(EHeader header) { return sizeof(EHeader) + header.payload_size; }
static int serial_packet_size(SHeader header) { return sizeof(SHeader) + header.payload_size; }

static EPacket* create_espnow_packet(EPacket_Type type, uint32_t packet_size)
{
    assert(packet_size >= sizeof(EPacket));
    EPacket* pkt = DGL_MALLOC(packet_size);
    pkt->header.type = type;
    pkt->header.packet_id = (EPacket_ID) esp_random();
    pkt->header.payload_size = packet_size - sizeof(EPacket);
    
    return pkt;
}

static SPacket* create_serial_packet(SPacket_Type type, uint32_t packet_size)
{
    assert(packet_size >= sizeof(SPacket));
    SPacket* pkt = DGL_MALLOC(packet_size);
    pkt->header.delim = SERIAL_DELIMITER;
    pkt->header.type = type;
    pkt->header.payload_size = packet_size - sizeof(SPacket);
    
    return pkt;
}

static void destroy_espnow_packet(EPacket* pkt) { free(pkt); }
static void destroy_serial_packet(SPacket* pkt) { free(pkt); }

/*
 * Sending and receiving ESPNow-/Serial-Packets
 */ 

static bool send_espnow_packet_to_drone(uint8_t* mac_addr, EPacket* pkt)
{
    esp_err_t ret = esp_now_send(mac_addr, (uint8_t*) pkt, espnow_packet_size(pkt->header));
    
    if (ret != ESP_OK) {
        DGL_ERROR("send_espnow_packet_to_drone: send packet error");
        return false;
    }
    else {
        return true;
    }
}

static void send_espnow_packets_task(void*)
{
    while (1)
    {
        // We wait until there is something in the queue (and just block the task in the meantime).
        ESPNow_Send_Info esend_info = {0};
        if (xQueueReceive(s_espnow_packet_send_queue, &esend_info, portMAX_DELAY))
        {
            EPacket* epkt = esend_info.epkt;
            esp_err_t ret = ret = esp_now_send(esend_info.mac_addr, (uint8_t*) epkt, espnow_packet_size(epkt->header));

            // Out of memory just means the queue is full and we need to wait
            while (ret == ESP_ERR_ESPNOW_NO_MEM) {
                DGL_DBG_INFO("send_espnow_packets_task: %s", esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(1));
                ret = esp_now_send(esend_info.mac_addr, (uint8_t*) epkt, espnow_packet_size(epkt->header));
            }

            if (ret != ESP_OK) {
                DGL_ERROR("send_espnow_packets_task: %s", esp_err_to_name(ret));
                continue;
            }

            if (esend_info.wait_for_acknowledge) {
                xQueueSend(s_espnow_packet_ack_queue, &esend_info, xTicksToWait)
            }
        }
    }

    vTaskDelete(NULL);
}

/* Returns how many bytes were sent or 0 on error. */
static int send_serial_packet_to_master(SPacket* pkt)
{
    if (pkt == NULL) return 0;

    if (!usb_serial_jtag_is_connected()) {
        DGL_ERROR("send_serial_packet_to_master: USB Serial/JTAG is not connected");
        return 0;
    }
    
    int pkt_size = serial_packet_size(pkt->header);
    int bytes_written = usb_serial_jtag_write_bytes(pkt, pkt_size, USB_SERIAL_WRITE_MAX_TICKS_TO_WAIT);

    if (bytes_written != pkt_size) {
        DGL_DBG_INFO("usb_serial_jtag_write_bytes failed");
        return 0;
    }
    
    return bytes_written;
}

/*
 * Sending specific packets
 */

static void send_c2m_log(Log_Type type, const char* format, ...)
{
    va_list args;
    
    va_start(args, format);
    int msg_len = vsnprintf(NULL, 0, format, args);
    va_end(args);
    
    int pkt_log_size = sizeof(SPacket_C2M_Log) + (msg_len + 1) * sizeof(char);
    
    SPacket_C2M_Log* pkt_log = (SPacket_C2M_Log*) create_serial_packet(SPKT_C2M_LOG, pkt_log_size);
    pkt_log->log.type = type;
    pkt_log->log.msg_length = msg_len + 1;
    
    va_start(args, format);
    vsnprintf(pkt_log->log.msg, msg_len + 1, format, args);
    va_end(args);
    
    send_serial_packet_to_master((SPacket*) pkt_log);
    destroy_serial_packet((SPacket*) pkt_log);
}

/*
 * Incoming ESP-Now packet handling task
 */

static void espnow_packet_receive_task(void*)
{
    while (1)
    {
        ESPNow_Receive_Callback_Info recv_info = {0};
        if (xQueueReceive(s_espnow_receive_cb_queue, &recv_info, portMAX_DELAY))
        {
            EPacket* epkt = (EPacket*) recv_info.packet_data;

            switch (epkt->header.type) {
        
            case EPKT_D2C_ADVERTISE:
            {
                if (!is_esp_mac_addr_indexed(mac_addr))
                {
                    bool found_and_inserted_in_empty_slot = false;
                    for (int esp_idx = 0; esp_idx < MAX_ESP_DEVICES; ++esp_idx) {
                        if (memcmp(&s_esp_devices[esp_idx].mac_addr, s_zero_mac, ESP_NOW_ETH_ALEN) == 0)
                        {
                            memcpy(&s_esp_devices[esp_idx].mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
                            memcpy(&s_esp_devices[esp_idx].drone_name, ((EPacket_D2C_Advertise*) epkt)->drone_name, MAX_DRONE_NAME_LENGTH);
                            found_and_inserted_in_empty_slot = true;
                            break;
                        }
                    }

                    if (!found_and_inserted_in_empty_slot) {
                        DGL_ERROR("Failed to add detected ESP to the static detected-esps-array of size '%d', "
                                  "because all slots are full", MAX_ESP_DEVICES);
                    }
                }
            } break;
        
            case EPKT_D2C_ACCEPT_CONNECTION:
            {
                Esp_Idx esp_idx = get_esp_idx_from_mac_addr(mac_addr);
                if (esp_idx == INVALID_ESP_IDX) break;
    
                s_esp_devices[esp_idx].connection_status = ESP_CONNECTED;

                SPacket_C2M_Drone_Connected* pkt_connected = (SPacket_C2M_Drone_Connected*)
                    create_serial_packet(SPKT_C2M_DRONE_CONNECTED, sizeof(SPacket_C2M_Drone_Connected));
                
                pkt_connected->esp_idx = esp_idx;

                send_serial_packet_to_master((SPacket*) pkt_connected);
                destroy_serial_packet((SPacket*) pkt_connected);
            } break;
        
            case EPKT_D2C_ACCEPT_DISCONNECT:
            {
                Esp_Idx esp_idx = get_esp_idx_from_mac_addr(mac_addr);
                if (esp_idx == INVALID_ESP_IDX) break;
    
                s_esp_devices[esp_idx].connection_status = ESP_DISCONNECTED;

                SPacket_C2M_Drone_Disconnected* pkt_disconnected = (SPacket_C2M_Drone_Disconnected*)
                    create_serial_packet(SPKT_C2M_DRONE_DISCONNECTED, sizeof(SPacket_C2M_Drone_Disconnected));
                
                pkt_disconnected->esp_idx = esp_idx;

                send_serial_packet_to_master((SPacket*) pkt_disconnected);
                destroy_serial_packet((SPacket*) pkt_disconnected);            
            } break;
        
            case EPKT_D2C_IMU_DATA:
            {
                EPacket_D2C_Imu_Data* pkt_d2c_imu_data = (EPacket_D2C_Imu_Data*) epkt;
                assert(pkt_d2c_imu_data->header.type == EPKT_D2C_IMU_DATA);

                Esp_Idx esp_idx = get_esp_idx_from_mac_addr(mac_addr);
                if (esp_idx == INVALID_ESP_IDX) return;

                SPacket_C2M_Imu_Data* pkt_c2m_imu_data = (SPacket_C2M_Imu_Data*) create_serial_packet(SPKT_C2M_IMU_DATA, sizeof(SPacket_C2M_Imu_Data));
                memcpy(&pkt_c2m_imu_data->imu_data, &pkt_d2c_imu_data->imu_data, sizeof(Payload_Imu_Data));
                pkt_c2m_imu_data->esp_idx = esp_idx;
    
                send_serial_packet_to_master((SPacket*) pkt_c2m_imu_data);
                destroy_serial_packet((SPacket*) pkt_c2m_imu_data);
            } break;
        
            case EPKT_D2C_LOG:
            {
                EPacket_D2C_Log* pkt_d2c_log = (EPacket_D2C_Log*) epkt;
                assert(pkt_d2c_log->header.type == EPKT_D2C_LOG);

                Esp_Idx esp_idx = get_esp_idx_from_mac_addr(mac_addr);
                if (esp_idx == INVALID_ESP_IDX) return;

                int msg_size = pkt_d2c_log->log.msg_length * sizeof(char);
                SPacket_C2M_Log* pkt_c2m_log = (SPacket_C2M_Log*) create_serial_packet(SPKT_C2M_LOG, sizeof(SPacket_C2M_Log) + msg_size);
                memcpy(&pkt_c2m_log->log, &pkt_d2c_log->log, sizeof(Payload_Log) + msg_size);
                pkt_c2m_log->esp_idx = esp_idx;
    
                send_serial_packet_to_master((SPacket*) pkt_c2m_log);
                destroy_serial_packet((SPacket*) pkt_c2m_log);
            } break;

            default:
                DGL_ERROR("Received packet with unexpected or unknown type: '%d'", epkt->header.type);
            }
        }
    }
    
    vTaskDelete(NULL);
}

static SPacket* get_next_serial_packet_from_master(TickType_t ticks_to_wait)
{
    /* First checks for the delimiter then reads the next header which is used to create
       the serial packet in memory. Then the payload of the packet is read.
       Remember: packet = header + payload */

    /* size_t bytes_available = usb_serial_jtag_get_read_bytes_available(); */
    /* // We should at least be able to read the header */
    /* if (bytes_available < sizeof(SHeader)) return NULL; */

    SHeader header = {0};
    int bytes_read_header = usb_serial_jtag_read_bytes(&header, sizeof(SHeader), ticks_to_wait);
    assert(bytes_read_header == sizeof(SHeader));

    if (header.delim != SERIAL_DELIMITER) {
        DGL_ERROR("Failed to read serial packet: Delimiter was not at the expected location");
        // TODO: 'skip to next delimiter'
        return NULL;
    }

    if (header.type <= SPKT_LOWER_BOUND || header.type >= SPKT_UPPER_BOUND) {
        DGL_ERROR("Received serial packet from master with invalid type '%d'; outside the expected range of [%d, %d].",
                  header.type, SPKT_LOWER_BOUND, SPKT_UPPER_BOUND);
        return NULL;
    }

    SPacket* pkt = create_serial_packet(header.type, sizeof(SHeader) + header.payload_size);
    int bytes_read_payload = usb_serial_jtag_read_bytes(pkt->payload, pkt->header.payload_size, ticks_to_wait);
    
    if (bytes_read_payload != pkt->header.payload_size) {
        DGL_ERROR("Failed to read payload of serial packet: Not enough data available, but timeout reached.");
        return NULL;
    }

    return pkt;
}

static void serial_packet_receive_task(void*)
{
    while (1)
    {
        SPacket* spkt = get_next_serial_packet_from_master(portMAX_DELAY);
        if (spkt != NULL)
        {
            switch (spkt->header.type) {
                
            case SPKT_M2C_CONNECT_TO_DRONE:
            {
                Esp_Idx esp_idx = ((Packet_M2C_Connect_To_Drone*) spkt)->esp_idx;
                if (!valid_esp_device_at_esp_idx(esp_idx)) break;
    
                if (s_esp_devices[esp_idx].connection_status == ESP_DISCONNECTED) {
                    s_esp_devices[esp_idx].connection_status = ESP_TRY_TO_CONNECT;

                    // TODO: send 'pls I want connect' to the drone with mandatory ack
                    
                }
            } break;
                
            case SPKT_M2C_DISCONNECT_FROM_DRONE: {
                
            } break;
                
            case SPKT_M2C_CONTROLL_COMMANDS: {
                
            } break;
                
            case SPKT_M2C_GET_DETECTED_DRONES: {
                
            } break;
                
            }
        }
    }
}

static void xtask_try_to_connect_to_esp(void* voidp_esp_idx)
{
    Esp_Idx esp_idx = *((Esp_Idx*) voidp_esp_idx);
    free(voidp_esp_idx);

    Packet_C2D_Connect* pkt_connect = (Packet_C2D_Connect*) create_packet(sizeof(Packet_C2D_Connect), PKT_C2D_CONNECT);

    TickType_t tickcount_begin = xTaskGetTickCount();

    while (1)
    {
        if (!valid_esp_device_at_esp_idx(esp_idx))                            break;
        if (s_esp_devices[esp_idx].connection_status != ESP_TRY_TO_CONNECT)   break;

        TickType_t tickcount_now = xTaskGetTickCount();

        if (tickcount_now - tickcount_begin >= TRY_TO_CONNECT_TO_ESP_TIMEOUT) {
            DGL_ERROR("xtask_try_to_connect_to_esp: Trying to connect to the esp at index '%d' Timed out after '%d'ms",
                      esp_idx, TRY_TO_CONNECT_TO_ESP_TIMEOUT * portTICK_PERIOD_MS);
            break;
        }
        
        send_espnow_packet_to_drone(s_esp_devices[esp_idx].mac_addr, (Packet*) pkt_connect);
        vTaskDelay(TRY_TO_CONNECT_TO_ESP_SEND_DELAY);
    }

    destroy_packet((Packet*) pkt_connect);
    vTaskDelete(NULL); // Delete this task
}

static void receive_m2c_connect(Packet* pkt)
{
    assert(pkt->header.type == PKT_M2C_CONNECT_TO_DRONE);
    Packet_M2C_Connect_To_Drone* pkt_connect = (Packet_M2C_Connect_To_Drone*) pkt;

    Esp_Idx* esp_idx = DGL_MALLOC(sizeof(Esp_Idx));
    *esp_idx = pkt_connect->esp_idx;
    
    if (!valid_esp_device_at_esp_idx(*esp_idx)) return;
    
    if (s_esp_devices[*esp_idx].connection_status == ESP_DISCONNECTED)
    {
        s_esp_devices[*esp_idx].connection_status = ESP_TRY_TO_CONNECT;
        BaseType_t task_create_result = xTaskCreate(xtask_try_to_connect_to_esp,
                                                    "try_to_connect_to_esp",
                                                    XTASK_DEFAULT_STACK_DEPTH,
                                                    (void*) esp_idx,
                                                    XTASK_DEFAULT_PRIORITY,
                                                    s_esp_devices[*esp_idx].task_handle_try_to_connect);
        if (task_create_result != pdPASS) {
            DGL_ERROR("receive_2d_connect: Failed to create task");
        }
    }
}

static void xtask_try_to_disconnect_from_esp(void* voidp_esp_idx)
{
    Esp_Idx esp_idx = *((Esp_Idx*) voidp_esp_idx);
    free(voidp_esp_idx);

    Packet_C2D_Disconnect* pkt_disconnect = (Packet_C2D_Disconnect*) create_packet(sizeof(Packet_C2D_Disconnect), PKT_C2D_DISCONNECT);

    TickType_t tickcount_begin = xTaskGetTickCount();

    while (1)
    {
        if (!valid_esp_device_at_esp_idx(esp_idx))                               break;
        if (s_esp_devices[esp_idx].connection_status != ESP_TRY_TO_DISCONNECT)   break;

        TickType_t tickcount_now = xTaskGetTickCount();

        if (tickcount_now - tickcount_begin >= TRY_TO_DISCONNECT_FROM_ESP_TIMEOUT) {
            DGL_ERROR("xtask_try_to_disconnect_from_esp: Trying to disconnect from the esp at index '%d' "
                      "Timed out after '%d'ms. The ESP is probably offline so it gets declared as disconnected anyway",
                      esp_idx, TRY_TO_DISCONNECT_FROM_ESP_TIMEOUT * portTICK_PERIOD_MS);
            s_esp_devices[esp_idx].connection_status = ESP_DISCONNECTED;
            break;
        }
        
        send_espnow_packet_to_drone(s_esp_devices[esp_idx].mac_addr, (Packet*) pkt_disconnect);
        vTaskDelay(TRY_TO_DISCONNECT_FROM_ESP_SEND_DELAY);
    }

    destroy_packet((Packet*) pkt_disconnect);
    vTaskDelete(NULL); // Delete this task
}

static void receive_m2c_disconnect(Packet* pkt)
{
    assert(pkt->header.type == PKT_M2C_DISCONNECT_FROM_DRONE);
    Packet_M2C_Disconnect_From_Drone* pkt_disconnect = (Packet_M2C_Disconnect_From_Drone*) pkt;

    Esp_Idx* esp_idx = DGL_MALLOC(sizeof(Esp_Idx));
    *esp_idx = pkt_disconnect->esp_idx;
    
    if (!valid_esp_device_at_esp_idx(*esp_idx)) return;
    
    if (s_esp_devices[*esp_idx].connection_status == ESP_CONNECTED)
    {
        s_esp_devices[*esp_idx].connection_status = ESP_TRY_TO_DISCONNECT;
        BaseType_t task_create_result = xTaskCreate(xtask_try_to_disconnect_from_esp,
                                                    "try_to_disconnect_from_esp",
                                                    XTASK_DEFAULT_STACK_DEPTH,
                                                    (void*) esp_idx,
                                                    XTASK_DEFAULT_PRIORITY,
                                                    s_esp_devices[*esp_idx].task_handle_try_to_disconnect);
        if (task_create_result != pdPASS) {
            DGL_ERROR("receive_2d_disconnect: Failed to create task");
        }
    }
}

/* static void receive_m2c_controll_commands(Packet* pkt) */
/* { */
/*     assert(pkt->header.type == PKT_M2C_CONTROLL_COMMANDS); */
/*     Packet_M2C_Controll_Commands* pkt_m2c_ctrl_cmds = (Packet_M2C_Controll_Commands*) pkt; */

/*     Packet_C2D_Controll_Commands* pkt_c2d_ctrl_cmds = (Packet_C2D_Controll_Commands*) create_packet(sizeof(Packet_C2D_Controll_Commands), PKT_C2D_CONTROLL_COMMANDS); */
/*     memcpy(&pkt_c2d_ctrl_cmds->ctrl_cmds, &pkt_m2c_ctrl_cmds->ctrl_cmds, sizeof(Payload_Controll_Commands)); */

/*     if (!valid_esp_device_at_esp_idx(pkt_m2c_ctrl_cmds->esp_idx)) return; */

/*     send_espnow_packet_to_drone(s_esp_devices[pkt_m2c_ctrl_cmds->esp_idx].mac_addr, (Packet*) pkt_c2d_ctrl_cmds); */
/*     destroy_packet((Packet*) pkt_c2d_ctrl_cmds); */
/* } */

/* static void receive_m2c_get_detected_drones(Packet* pkt) */
/* { */
/*     // Sends a PKT_C2M_DETECTED_DRONES back. */
    
/*     int detected_drones_length = 0; */
/*     for (Esp_Idx esp_idx = 0; esp_idx < MAX_ESP_DEVICES; ++esp_idx) { */
/*         if (valid_esp_device_at_esp_idx(esp_idx)) ++detected_drones_length; */
/*     } */

/*     int detected_drones_pkt_size = sizeof(Packet_C2M_Detected_Drones) + detected_drones_length * sizeof(Payload_Detected_Drone); */

/*     Packet_C2M_Detected_Drones* pkt_detected_drones = (Packet_C2M_Detected_Drones*) create_packet(detected_drones_pkt_size, PKT_C2M_DETECTED_DRONES); */
/*     if (pkt_detected_drones == NULL) return; */

/*     pkt_detected_drones->detected_drones_length = detected_drones_length; */

/*     int detected_drones_idx = 0; */
/*     for (Esp_Idx esp_idx = 0; esp_idx < MAX_ESP_DEVICES; ++esp_idx) { */
/*         if (valid_esp_device_at_esp_idx(esp_idx)) { */
/*             assert(detected_drones_idx < detected_drones_length); */
            
/*             pkt_detected_drones->detected_drones[detected_drones_idx].esp_idx = esp_idx; */
/*             memcpy(&pkt_detected_drones->detected_drones[detected_drones_idx].drone_name, s_esp_devices[esp_idx].drone_name, MAX_DRONE_NAME_LENGTH); */
            
/*             ++detected_drones_idx; */
/*         } */
/*     } */

/*     send_serial_packet_to_master((Packet*) pkt_detected_drones); */
/*     destroy_packet((Packet*) pkt_detected_drones); */
/* } */

/*
 * Application loop
 */

void dongle_loop()
{
    while (1)
    {
        // Handle incoming packets from the drone via ESP-Now
        
        ESPNow_Receive_Callback_Info recv_info = {0};
        if (xQueueReceive(s_espnow_receive_cb_queue, &recv_info, 0))
        {
            Packet* pkt = (Packet*) recv_info.packet_data;
            From_Drone_Handler_fn from_drone_handler = get_from_drone_handler(pkt->header.type);

            if (recv_info.packet_data_length == packet_size(pkt))
            {
                if (from_drone_handler != NULL) {
                    from_drone_handler(pkt, recv_info.mac_addr);
                }
                else DGL_ERROR("Received unexpected or unknown packet from the drone");
            }
            else DGL_ERROR("recv_info.packet_data_length != packet_size(pkt)");
        }
        destroy_espnow_receive_cb_info(recv_info);

        // Handle ESP-Now send Callback

        // Handle incoming packets from the master via USB Serial/JTAG
        
        SPacket* spkt = get_next_serial_packet_from_master();
        if (pkt != NULL)
        {
            DGL_DBG_INFO("Received packet!");
            From_Master_Handler_fn from_master_handler = get_from_master_handler(pkt->header.type);
            
            if (pkt_info.from_master_handler != NULL) {
                pkt_info.from_master_handler(pkt);
            }
            else DGL_ERROR("Received unexpected or unknown packet from the master");
        }
        destroy_packet(pkt);
        
        vTaskDelay(DONGLE_MAIN_LOOP_DELAY);
    }
}

void test()
{
    EPacket_C2D_Controll_Commands* ctrl_cmds = (EPacket_C2D_Controll_Commands*) create_espnow_packet(EPKT_C2D_CONTROLL_COMMANDS, sizeof(EPacket_C2D_Controll_Commands));
    ctrl_cmds->ctrl_cmds.ctrl_state_flags = 0x01;
    ctrl_cmds->ctrl_cmds.pitch = 0x02;
    ctrl_cmds->ctrl_cmds.yaw = 0x03;
    ctrl_cmds->ctrl_cmds.roll = 0x04;
    ctrl_cmds->ctrl_cmds.throttle = 0x05;

    printf("sizeof(EPacket_C2D_Controll_Commands) = %d\n", sizeof(EPacket_C2D_Controll_Commands));
    send_espnow_packet_to_drone(s_broadcast_mac, (EPacket*) ctrl_cmds);
}

void app_main(void)
{
    init_dongle();
    while (1) {
        test();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    /* dongle_loop(); */
    esp_restart();
}
