#include "drone_controller_communication_packets.h"
#include "master_controller_communication_packets.h"

#include <string.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
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

// User changeable constants

#define MAX_ESP_DEVICES (128)

#define RECEIVE_CALLBACK_QUEUE_SIZE (10)
#define SEND_CALLBACK_QUEUE_SIZE (10)

#define USB_SERIAL_JTAG_BUFFER_SIZE (1024)
#define USB_SERIAL_WRITE_MAX_TICKS_TO_WAIT (20 / portTICK_PERIOD_MS)
#define USB_SERIAL_READ_MAX_TICKS_TO_WAIT  (20 / portTICK_PERIOD_MS)

#define XTASK_DEFAULT_STACK_DEPTH (100)
#define XTASK_DEFAULT_PRIORITY    (10)

#define TRY_TO_CONNECT_TO_ESP_SEND_DELAY  (100 / portTICK_PERIOD_MS)
#define TRY_TO_CONNECT_TO_ESP_TIMEOUT     (1000 / portTICK_PERIOD_MS)
#define TRY_TO_DISCONNECT_FROM_ESP_SEND_DELAY  (100 / portTICK_PERIOD_MS)
#define TRY_TO_DISCONNECT_FROM_ESP_TIMEOUT     (1000 / portTICK_PERIOD_MS)

#define DONGLE_MAIN_LOOP_DELAY (100 / portTICK_PERIOD_MS)

// Implementation constants (do not touch)

#define ESPNOW_WIFI_MODE (WIFI_MODE_AP)
#define ESPNOW_WIFI_IF   (ESP_IF_WIFI_AP)

// Error messaging and logging

#define SEND_LOGS_AS_SERIAL_PACKETS (1)
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

// Special mac address for broadcasting to all devices in range, not just a single device
static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint8_t s_zero_mac[ESP_NOW_ETH_ALEN]      = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} ESPNow_Send_Callback_Info;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *packet_data;
    uint16_t packet_data_length;
} ESPNow_Receive_Callback_Info;

static void destroy_espnow_receive_cb_info(ESPNow_Receive_Callback_Info recv_info)
{
    free(recv_info.packet_data);
}

typedef void (*From_Master_Handler_fn)(Packet*, uint8_t* mac_addr);

static void receive_m2c_connect(SPacket*);
static void receive_m2c_disconnect(SPacket*);
static void receive_m2c_controll_commands(SPacket*);
static void receive_m2c_get_detected_drones(SPacket*);

static From_Master_Handler_fn get_from_master_handler(SPacket_Type type)
{
    static const From_Master_Handler_fn s_fn_table[SPKT_COUNT] = {
        [SPKT_IDX(SPKT_M2C_CONNECT_TO_DRONE)]      = receive_m2c_connect,
        [SPKT_IDX(SPKT_M2C_DISCONNECT_FROM_DRONE)] = receive_m2c_disconnect,
        [SPKT_IDX(SPKT_M2C_CONTROLL_COMMANDS)]     = receive_m2c_controll_commands,
        [SPKT_IDX(SPKT_M2C_GET_DETECTED_DRONES)]   = receive_m2c_get_detected_drones,
    };
    
    if (type > SPKT_LOWER_BOUND && type < SPKT_UPPER_BOUND) {
        return s_fn_table[type - SPKT_LOWER_BOUND];
    }
    else {
        return NULL;
    }
}

typedef void (*From_Drone_Handler_fn)(EPacket*, uint8_t* mac_addr);

static void receive_d2c_drone_advertise(EPacket*, uint8_t* mac_addr);
static void receive_d2c_accept_connection(EPacket*, uint8_t* mac_addr);
static void receive_d2c_accept_disconnect(EPacket*, uint8_t* mac_addr);
static void receive_d2c_imu_data(EPacket*, uint8_t* mac_addr);
static void receive_d2c_log(EPacket*, uint8_t* mac_addr);

static From_Drone_Handler_fn get_from_drone_handler(EPacket_Type type)
{
    static const From_Drone_Handler_fn s_fn_table[EPKT_COUNT] = {
        [EPKT_IDX(EPKT_D2C_ADVERTISE)]         = receive_d2c_drone_advertise,
        [EPKT_IDX(EPKT_D2C_ACCEPT_CONNECTION)] = receive_d2c_accept_connection,
        [EPKT_IDX(EPKT_D2C_ACCEPT_DISCONNECT)] = receive_d2c_accept_disconnect,
        [EPKT_IDX(EPKT_D2C_IMU_DATA)]          = receive_d2c_imu_data,
        [EPKT_IDX(EPKT_D2C_LOG)]               = receive_d2c_log,
    };

    if (type > EPKT_LOWER_BOUND && type < EPKT_UPPER_BOUND) {
        return s_fn_table[type - EPKT_LOWER_BOUND];
    }
    else {
        return NULL;
    }
}

typedef enum {
    ESP_DISCONNECTED = 0,
    ESP_TRY_TO_CONNECT,
    ESP_CONNECTED,
    ESP_TRY_TO_DISCONNECT,
} ESP_Connection_State;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    char drone_name[MAX_DRONE_NAME_LENGTH];
    ESP_Connection_State connected_state;
    TaskHandle_t* task_handle_try_to_connect;
    TaskHandle_t* task_handle_try_to_disconnect;
} ESP_Device;

// Global dongle (dgl) state
static struct {
    QueueHandle_t espnow_send_cb_queue;
    QueueHandle_t espnow_receive_cb_queue;
    ESP_Device esp_devices[MAX_ESP_DEVICES];
} dgl = { 0 };

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
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) DGL_ABORT("init_espnow: malloc failed");
    
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

    // Initialize the global dongle state
    
    dgl.espnow_receive_cb_queue = xQueueCreate(RECEIVE_CALLBACK_QUEUE_SIZE, sizeof(ESPNow_Receive_Callback_Info));
    dgl.espnow_send_cb_queue    = xQueueCreate(SEND_CALLBACK_QUEUE_SIZE, sizeof(ESPNow_Send_Callback_Info));

    DGL_DBG_INFO("Dongle Initialized!");
}

/*
 * ESP-Now send and receive callbacks...
 */

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    ESPNow_Send_Callback_Info send_cb;
    if (tx_info == NULL) {
        DGL_ERROR("espnow_send_cb: arg error");
        return;
    }

    memcpy(send_cb.mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    send_cb.status = status;
    if (!xQueueSend(dgl.espnow_send_cb_queue, &send_cb, 0)) {
        DGL_DBG_INFO("espnow_send_cb: send queue fail");
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int length)
{
    ESPNow_Receive_Callback_Info recv_cb;
    uint8_t* mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || length <= 0) {
        DGL_ERROR("espnow_recv_cb: arg error");
        return;
    }

    memcpy(recv_cb.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb.packet_data = malloc(length);
    if (recv_cb.packet_data == NULL) DGL_ABORT("espnow_recv_cb: malloc failed");
    
    memcpy(recv_cb.packet_data, data, length);
    recv_cb.packet_data_length = length;
    if (!xQueueSend(dgl.espnow_receive_cb_queue, &recv_cb, 0)) {
        DGL_DBG_INFO("espnow_recv_cb: receive queue fail");
        free(recv_cb.packet_data);
    }
}

/*
 * MAC-Address/ESP-Device management
 */

static bool is_esp_mac_addr_indexed(uint8_t* mac_addr)
{
    for (int esp_idx = 0; esp_idx < MAX_ESP_DEVICES; ++esp_idx) {
        if (memcmp(&dgl.esp_devices[esp_idx].mac_addr, mac_addr, ESP_NOW_ETH_ALEN) == 0) {
            return true;
        }
    }
    return false;
}

static Esp_Idx get_esp_idx_from_mac_addr(uint8_t* mac_addr)
{
    for (int idx = 0; idx < MAX_ESP_DEVICES; ++idx) {
        if (memcmp(&dgl.esp_devices[idx].mac_addr, mac_addr, ESP_NOW_ETH_ALEN) == 0) {
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

    if (memcmp(dgl.esp_devices[esp_idx].mac_addr, s_zero_mac, ESP_NOW_ETH_ALEN) == 0) {
        return false;
    }
    return true;
}

/*
 * ESPNow-/Serial Packet management
 */

static int espnow_packet_size(EHeader header) { return sizeof(EHeader) + header.payload_size; }
static int serial_packet_size(SPacket header) { return sizeof(SHeader) + header.payload_size; }

static EPacket* create_espnow_packet(int packet_size, EPacket_Type type)
{
    assert(packet_size >= sizeof(EPacket));
    EPacket* pkt = malloc(packet_size);
    if (pkt == NULL) DGL_ABORT("create_espnow_packet: malloc failed");

    pkt->header.type = type;
    pkt->header.packet_id = (EPacket_ID) esp_random();
    pkt->header.payload_size = packet_size - sizeof(EPacket);
    
    return pkt;
}

static SPacket* create_serial_packet(SPacket_Type type, uint32_t payload_size)
{
    /* assert(packet_size >= sizeof(SPacket)); */
    SPacket* pkt = malloc(sizeof(SHeader) + payload_size);
    if (pkt == NULL) DGL_ABORT("create_serial_packet: malloc failed");

    pkt->header.delim = SERIAL_DELIMITER;
    pkt->header.type = type;
    pkt->header.payload_size = payload_size;
    
    return pkt;
}

static void destroy_espnow_packet(EPacket* pkt) { free(pkt); }
static void destroy_serial_packet(SPacket* pkt) { free(pkt); }

/*
 * Sending and receiving ESPNow-/Serial-Packets
 */ 

static bool send_espnow_packet_to_drone(uint8_t* mac_addr, Packet* pkt)
{
    esp_err_t ret = esp_now_send(mac_addr, (uint8_t*) pkt, packet_size(pkt));
    
    if (ret != ESP_OK) {
        DGL_ERROR("send_espnow_packet_to_drone: send packet error");
        return false;
    }
    else {
        return true;
    }
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

static SPacket* get_next_serial_packet_from_master()
{
    /* First checks for the delimiter then reads the next header which is used to create
       the serial packet in memory. Then the payload of the packet is read.
       Remember: packet = header + payload */

    size_t bytes_available = usb_serial_jtag_get_read_bytes_available();

    // We should at least be able to read the header
    if (bytes_available < sizeof(SHeader)) return NULL;

    SHeader header = {0};
    int bytes_read_header = usb_serial_jtag_read_bytes(&header, sizeof(SHeader), USB_SERIAL_READ_MAX_TICKS_TO_WAIT);
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

    SPacket* pkt = create_serial_packet(header.type, header.payload_size);
    int bytes_read_payload = usb_serial_jtag_read_bytes(pkt->payload, pkt->header.payload_size, USB_SERIAL_READ_MAX_TICKS_TO_WAIT);
    
    if (bytes_read_payload != pkt->header.payload_size) {
        DGL_ERROR("Failed to read payload of serial packet: Not enough data available, but timeout reached.");
        return NULL;
    }

    return pkt;
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
    
    int pkt_log_size = sizeof(Packet_C2M_Log) + (msg_len + 1) * sizeof(char);

    Packet_C2M_Log* pkt_log = (Packet_C2M_Log*) create_serial_packet(pkt_log_size, PKT_C2M_LOG);
    pkt_log->log.type = type;
    pkt_log->log.msg_length = msg_len + 1;
    
    va_start(args, format);
    vsnprintf(pkt_log->log.msg, msg_len + 1, format, args);
    va_end(args);

    send_serial_packets_to_master((SPacket*) pkt_log);
    destroy_serial_packet((SPacket*) pkt_log);
}

/*
 * Receiving specific packets
 */

static void receive_d2c_drone_advertise(Packet* pkt, uint8_t* mac_addr)
{
    if (!is_esp_mac_addr_indexed(mac_addr))
    {
        bool found_and_inserted_in_empty_slot = false;
        for (int esp_idx = 0; esp_idx < MAX_ESP_DEVICES; ++esp_idx) {
            if (memcmp(&dgl.esp_devices[esp_idx].mac_addr, s_zero_mac, ESP_NOW_ETH_ALEN) == 0)
            {
                memcpy(&dgl.esp_devices[esp_idx].mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
                memcpy(&dgl.esp_devices[esp_idx].drone_name, ((Packet_D2C_Advertise*) pkt)->drone_name, MAX_DRONE_NAME_LENGTH);
                found_and_inserted_in_empty_slot = true;
                break;
            }
        }

        if (!found_and_inserted_in_empty_slot) {
            DGL_ERROR("Failed to add detected ESP to the static detected-esps-array of size '%d', "
                      "because all slots are full", MAX_ESP_DEVICES);
        }
    }
}

static void receive_d2c_accept_connection(Packet* pkt, uint8_t* mac_addr)
{
    Esp_Idx esp_idx = get_esp_idx_from_mac_addr(mac_addr);
    if (esp_idx == INVALID_ESP_IDX) return;
    
    dgl.esp_devices[esp_idx].connected_state = ESP_CONNECTED;

    Packet_C2M_Drone_Connected* pkt_connected = (Packet_C2M_Drone_Connected*) create_packet(sizeof(Packet_C2M_Drone_Connected), PKT_C2M_DRONE_CONNECTED);
    pkt_connected->esp_idx = esp_idx;

    send_serial_packets_to_master((Packet*) pkt_connected);
    destroy_packet((Packet*) pkt_connected);
}

static void receive_d2c_accept_disconnect(Packet* pkt, uint8_t* mac_addr)
{
    Esp_Idx esp_idx = get_esp_idx_from_mac_addr(mac_addr);
    if (esp_idx == INVALID_ESP_IDX) return;
    
    dgl.esp_devices[esp_idx].connected_state = ESP_DISCONNECTED;

    Packet_C2M_Drone_Disconnected* pkt_disconnected = (Packet_C2M_Drone_Disconnected*) create_packet(sizeof(Packet_C2M_Drone_Disconnected), PKT_C2M_DRONE_DISCONNECTED);
    pkt_disconnected->esp_idx = esp_idx;

    send_serial_packets_to_master((Packet*) pkt_disconnected);
    destroy_packet((Packet*) pkt_disconnected);
}

static void receive_d2c_imu_data(Packet* pkt, uint8_t* mac_addr)
{
    Packet_D2C_Imu_Data* pkt_d2c_imu_data = (Packet_D2C_Imu_Data*) pkt;
    assert(pkt_d2c_imu_data->header.type == PKT_D2C_IMU_DATA);

    Esp_Idx esp_idx = get_esp_idx_from_mac_addr(mac_addr);
    if (esp_idx == INVALID_ESP_IDX) return;

    Packet_C2M_Imu_Data* pkt_c2m_imu_data = (Packet_C2M_Imu_Data*) create_packet(sizeof(Packet_C2M_Imu_Data), PKT_C2M_IMU_DATA);
    memcpy(&pkt_c2m_imu_data->imu_data, &pkt_d2c_imu_data->imu_data, sizeof(Payload_Imu_Data));
    pkt_c2m_imu_data->esp_idx = esp_idx;
    
    send_serial_packets_to_master((Packet*) pkt_c2m_imu_data);
    destroy_packet((Packet*) pkt_c2m_imu_data);
}

static void receive_d2c_log(Packet* pkt, uint8_t* mac_addr)
{
    Packet_D2C_Log* pkt_d2c_log = (Packet_D2C_Log*) pkt;
    assert(pkt_d2c_log->header.type == PKT_D2C_LOG);

    Esp_Idx esp_idx = get_esp_idx_from_mac_addr(mac_addr);
    if (esp_idx == INVALID_ESP_IDX) return;

    Packet_C2M_Log* pkt_c2m_log = (Packet_C2M_Log*) create_packet(packet_size(pkt), PKT_C2M_LOG);
    
    int log_size = sizeof(Payload_Log) + pkt_d2c_log->log.msg_length * sizeof(char);
    memcpy(&pkt_c2m_log->log, &pkt_d2c_log->log, log_size);
    pkt_c2m_log->esp_idx = esp_idx;
    
    send_serial_packets_to_master((Packet*) pkt_c2m_log);
    destroy_packet((Packet*) pkt_c2m_log);
}

static void xtask_try_to_connect_to_esp(void* voidp_esp_idx)
{
    Esp_Idx esp_idx = *((Esp_Idx*) voidp_esp_idx);
    free(voidp_esp_idx);

    Packet_C2D_Connect* pkt_connect = (Packet_C2D_Connect*) create_packet(sizeof(Packet_C2D_Connect), PKT_C2D_CONNECT);

    TickType_t tickcount_begin = xTaskGetTickCount();

    while (true)
    {
        if (!valid_esp_device_at_esp_idx(esp_idx))                            break;
        if (dgl.esp_devices[esp_idx].connected_state != ESP_TRY_TO_CONNECT)   break;

        TickType_t tickcount_now = xTaskGetTickCount();

        if (tickcount_now - tickcount_begin >= TRY_TO_CONNECT_TO_ESP_TIMEOUT) {
            DGL_ERROR("xtask_try_to_connect_to_esp: Trying to connect to the esp at index '%d' Timed out after '%d'ms",
                      esp_idx, TRY_TO_CONNECT_TO_ESP_TIMEOUT * portTICK_PERIOD_MS);
            break;
        }
        
        send_espnow_packet_to_drone(dgl.esp_devices[esp_idx].mac_addr, (Packet*) pkt_connect);
        vTaskDelay(TRY_TO_CONNECT_TO_ESP_SEND_DELAY);
    }

    destroy_packet((Packet*) pkt_connect);
    vTaskDelete(NULL); // Delete this task
}

static void receive_m2c_connect(Packet* pkt)
{
    assert(pkt->header.type == PKT_M2C_CONNECT_TO_DRONE);
    Packet_M2C_Connect_To_Drone* pkt_connect = (Packet_M2C_Connect_To_Drone*) pkt;

    Esp_Idx* esp_idx = malloc(sizeof(Esp_Idx));
    if (esp_idx == NULL) DGL_ABORT("receive_m2c_connect: malloc failed");
    
    *esp_idx = pkt_connect->esp_idx;
    
    if (!valid_esp_device_at_esp_idx(*esp_idx)) return;
    
    if (dgl.esp_devices[*esp_idx].connected_state == ESP_DISCONNECTED)
    {
        dgl.esp_devices[*esp_idx].connected_state = ESP_TRY_TO_CONNECT;
        BaseType_t task_create_result = xTaskCreate(xtask_try_to_connect_to_esp,
                                                    "try_to_connect_to_esp",
                                                    XTASK_DEFAULT_STACK_DEPTH,
                                                    (void*) esp_idx,
                                                    XTASK_DEFAULT_PRIORITY,
                                                    dgl.esp_devices[*esp_idx].task_handle_try_to_connect);
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

    while (true)
    {
        if (!valid_esp_device_at_esp_idx(esp_idx))                               break;
        if (dgl.esp_devices[esp_idx].connected_state != ESP_TRY_TO_DISCONNECT)   break;

        TickType_t tickcount_now = xTaskGetTickCount();

        if (tickcount_now - tickcount_begin >= TRY_TO_DISCONNECT_FROM_ESP_TIMEOUT) {
            DGL_ERROR("xtask_try_to_disconnect_from_esp: Trying to disconnect from the esp at index '%d' "
                      "Timed out after '%d'ms. The ESP is probably offline so it gets declared as disconnected anyway",
                      esp_idx, TRY_TO_DISCONNECT_FROM_ESP_TIMEOUT * portTICK_PERIOD_MS);
            dgl.esp_devices[esp_idx].connected_state = ESP_DISCONNECTED;
            break;
        }
        
        send_espnow_packet_to_drone(dgl.esp_devices[esp_idx].mac_addr, (Packet*) pkt_disconnect);
        vTaskDelay(TRY_TO_DISCONNECT_FROM_ESP_SEND_DELAY);
    }

    destroy_packet((Packet*) pkt_disconnect);
    vTaskDelete(NULL); // Delete this task
}

static void receive_m2c_disconnect(Packet* pkt)
{
    assert(pkt->header.type == PKT_M2C_DISCONNECT_FROM_DRONE);
    Packet_M2C_Disconnect_From_Drone* pkt_disconnect = (Packet_M2C_Disconnect_From_Drone*) pkt;

    Esp_Idx* esp_idx = malloc(sizeof(Esp_Idx));
    if (esp_idx == NULL) DGL_ABORT("receive_m2c_disconnect: malloc failed");
    
    *esp_idx = pkt_disconnect->esp_idx;
    
    if (!valid_esp_device_at_esp_idx(*esp_idx)) return;
    
    if (dgl.esp_devices[*esp_idx].connected_state == ESP_CONNECTED)
    {
        dgl.esp_devices[*esp_idx].connected_state = ESP_TRY_TO_DISCONNECT;
        BaseType_t task_create_result = xTaskCreate(xtask_try_to_disconnect_from_esp,
                                                    "try_to_disconnect_from_esp",
                                                    XTASK_DEFAULT_STACK_DEPTH,
                                                    (void*) esp_idx,
                                                    XTASK_DEFAULT_PRIORITY,
                                                    dgl.esp_devices[*esp_idx].task_handle_try_to_disconnect);
        if (task_create_result != pdPASS) {
            DGL_ERROR("receive_2d_disconnect: Failed to create task");
        }
    }
}

static void receive_m2c_controll_commands(Packet* pkt)
{
    assert(pkt->header.type == PKT_M2C_CONTROLL_COMMANDS);
    Packet_M2C_Controll_Commands* pkt_m2c_ctrl_cmds = (Packet_M2C_Controll_Commands*) pkt;

    Packet_C2D_Controll_Commands* pkt_c2d_ctrl_cmds = (Packet_C2D_Controll_Commands*) create_packet(sizeof(Packet_C2D_Controll_Commands), PKT_C2D_CONTROLL_COMMANDS);
    memcpy(&pkt_c2d_ctrl_cmds->ctrl_cmds, &pkt_m2c_ctrl_cmds->ctrl_cmds, sizeof(Payload_Controll_Commands));

    if (!valid_esp_device_at_esp_idx(pkt_m2c_ctrl_cmds->esp_idx)) return;

    send_espnow_packet_to_drone(dgl.esp_devices[pkt_m2c_ctrl_cmds->esp_idx].mac_addr, (Packet*) pkt_c2d_ctrl_cmds);
    destroy_packet((Packet*) pkt_c2d_ctrl_cmds);
}

static void receive_m2c_get_detected_drones(Packet* pkt)
{
    // Sends a PKT_C2M_DETECTED_DRONES back.
    
    int detected_drones_length = 0;
    for (Esp_Idx esp_idx = 0; esp_idx < MAX_ESP_DEVICES; ++esp_idx) {
        if (valid_esp_device_at_esp_idx(esp_idx)) ++detected_drones_length;
    }

    int detected_drones_pkt_size = sizeof(Packet_C2M_Detected_Drones) + detected_drones_length * sizeof(Payload_Detected_Drone);

    Packet_C2M_Detected_Drones* pkt_detected_drones = (Packet_C2M_Detected_Drones*) create_packet(detected_drones_pkt_size, PKT_C2M_DETECTED_DRONES);
    if (pkt_detected_drones == NULL) return;

    pkt_detected_drones->detected_drones_length = detected_drones_length;

    int detected_drones_idx = 0;
    for (Esp_Idx esp_idx = 0; esp_idx < MAX_ESP_DEVICES; ++esp_idx) {
        if (valid_esp_device_at_esp_idx(esp_idx)) {
            assert(detected_drones_idx < detected_drones_length);
            
            pkt_detected_drones->detected_drones[detected_drones_idx].esp_idx = esp_idx;
            memcpy(&pkt_detected_drones->detected_drones[detected_drones_idx].drone_name, dgl.esp_devices[esp_idx].drone_name, MAX_DRONE_NAME_LENGTH);
            
            ++detected_drones_idx;
        }
    }

    send_serial_packets_to_master((Packet*) pkt_detected_drones);
    destroy_packet((Packet*) pkt_detected_drones);
}

/*
 * Application loop
 */

void dongle_loop()
{
    while (true)
    {
        // Handle incoming packets from the drone via ESP-Now
        
        ESPNow_Receive_Callback_Info recv_info = { 0 };
        if (xQueueReceive(dgl.espnow_receive_cb_queue, &recv_info, 0))
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

        ESPNow_Send_Callback_Info send_info = { 0 };
        if (xQueueReceive(dgl.espnow_send_callback_queue, &send_info, 0))
        {
            if (send_info.status == ESP_NOW_SEND_FAIL) {
                DGL_DBG_INFO("Failed to send packet to ESP with MAC: "MACSTR, MAC2STR(send_info.mac_addr));
            }
        }

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

void app_main(void)
{
    init_dongle();
    dongle_loop();
    esp_restart();
}
