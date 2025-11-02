
#include "../../shared_types/communication_packets.h"
#include "sdkconfig.h"

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

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_select.h"

// User changeable constants

#define MAX_ESP_DEVICES (128)

#define ESPNOW_CHANNEL (1)
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
    #define DGL_ERROR(format, ...) send_c2u_log_msg(LOG_MSG_ERROR, format, ##__VA_ARGS__)
    #if ENABLE_DBG_INFO_LOGS
        #define DGL_DBG_INFO(format, ...) send_c2u_log_msg(LOG_MSG_DBG_INFO, format, ##__VA_ARGS__)
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

static void destroy_espnow_receive_callback_info(ESPNow_Receive_Callback_Info recv_info)
{
    free(recv_info.packet_data);
}

typedef struct {
    int pkt_size;
    void (*from_drone_handler)(Packet*, uint8_t* mac_addr);
    void (*from_user_handler)(Packet*);
} Packet_Info;

static void receive_d2c_drone_advertise(Packet*, uint8_t* mac_addr);
static void receive_d2c_accept_connection(Packet*, uint8_t* mac_addr);
static void receive_d2c_accept_disconnect(Packet*, uint8_t* mac_addr);
static void receive_d2c_imu_data(Packet*, uint8_t* mac_addr);

static void receive_u2c_connect(Packet*);
static void receive_u2c_disconnect(Packet*);
static void receive_u2c_controll_commands(Packet*);
static void receive_u2c_get_detected_drones(Packet*);

static const Packet_Info s_pkt_info_table[PKT_TYPE_SIZE] = {
    [PKT_INVALID]                 = { 0 },
    
    [PKT_D2C_ADVERTISE]           = { .from_drone_handler = receive_d2c_drone_advertise },
    
    [PKT_U2C_CONNECT]             = { .from_user_handler = receive_u2c_connect },
    [PKT_C2D_CONNECT]             = { 0 },
    [PKT_D2C_ACCEPT_CONNECTION]   = { .from_drone_handler = receive_d2c_accept_connection },
    [PKT_C2U_DRONE_CONNECTED]     = { 0 },
    
    [PKT_U2C_DISCONNECT]          = { .from_user_handler = receive_u2c_disconnect },
    [PKT_C2D_DISCONNECT]          = { 0 },
    [PKT_D2C_ACCEPT_DISCONNECT]   = { .from_drone_handler = receive_d2c_accept_disconnect },
    [PKT_C2U_DRONE_DISCONNECTED]  = { 0 },
    
    [PKT_U2C_CONTROLL_COMMANDS]   = { .from_user_handler = receive_u2c_controll_commands },
    [PKT_C2D_CONTROLL_COMMANDS]   = { 0 },

    [PKT_U2C_GET_DETECTED_DRONES] = { .from_user_handler = receive_u2c_get_detected_drones },
    [PKT_C2U_DETECTED_DRONES]     = { 0 },

    [PKT_D2C_IMU_DATA]            = { .from_drone_handler = receive_d2c_imu_data },
    [PKT_C2U_IMU_DATA]            = { 0 },
    
    [PKT_C2U_LOG_MSG]             = { 0 },
};

typedef enum {
    ESP_DISCONNECTED = 0,
    ESP_TRY_TO_CONNECT,
    ESP_CONNECTED,
    ESP_TRY_TO_DISCONNECT,
} ESP_Connection_State;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    ESP_Connection_State connected_state;
    TaskHandle_t* task_handle_try_to_connect;
    TaskHandle_t* task_handle_try_to_disconnect;
} ESP_Device;

// Global dongle (dgl) state
static struct {
    QueueHandle_t espnow_send_callback_queue;
    QueueHandle_t espnow_receive_callback_queue;

    ESP_Device esp_devices[MAX_ESP_DEVICES];
} dgl = { 0 };

static void init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (nvs_flash_erase() != ESP_OK) DGL_ABORT("init_nvs: nvs_flash_erase failed");
        ret = nvs_flash_init();
    }
    if(ret != ESP_OK) DGL_ABORT("init_nvs: nvs_flash_init failed");
}

static void init_wifi(uint8_t wifi_channel, bool long_range_mode)
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

static void send_c2u_log_msg(Log_Msg_Type type, const char* format, ...);

static void init_dongle()
{
    init_nvs();
    init_wifi(ESPNOW_CHANNEL, false);
    init_espnow();
    init_usb_serial_jtag();

    // Initialize the global dongle state
    
    dgl.espnow_receive_callback_queue = xQueueCreate(RECEIVE_CALLBACK_QUEUE_SIZE, sizeof(ESPNow_Receive_Callback_Info));
    dgl.espnow_send_callback_queue    = xQueueCreate(SEND_CALLBACK_QUEUE_SIZE, sizeof(ESPNow_Send_Callback_Info));

    DGL_DBG_INFO("Dongle Initialized!");
}

static int packet_size(Packet* pkt)
{
    return sizeof(Packet) + pkt->payload_size;
}

// TODO (YUZENI): Refactor such that the type must be specified on pkg creation
static Packet* create_packet(int packet_size, Packet_Type type)
{
    assert(packet_size >= sizeof(Packet));
    Packet* pkt = malloc(packet_size);
    if (pkt == NULL) DGL_ABORT("create_packet: malloc failed");

    pkt->type = type;
    pkt->payload_size = packet_size - sizeof(Packet);
    
    return pkt;
}

static void destroy_packet(Packet* pkt)
{
    free(pkt);
}

// Returns how many bytes were sent (0 or all bytes)
static int send_packet_to_user_via_serial_port(Packet* pkt)
{
    if (pkt == NULL) return 0;

    // Note: We do not check if the USB Serial/JTAG port is connected,
    // though we could with 'bool usb_serial_jtag_is_connected(void)'

    // The delimiter is combined with the packet because sending both seperately could cause trouble,
    // but its also not great to allocate & free just for that. (TODO: try not to allocate)
    
    int serial_pkt_size = packet_size(pkt) + sizeof(Serial_Delimiter);
    void* serial_pkt = malloc(serial_pkt_size);
    
    if (serial_pkt == NULL) DGL_ABORT("send_packet_to_user_via_serial_port: malloc failed");

    // serial_pkt = delimiter + pkt
    *((Serial_Delimiter*) serial_pkt) = SERIAL_DELIMITER;
    memcpy(serial_pkt + sizeof(Serial_Delimiter), pkt, packet_size(pkt));

    int bytes_written = usb_serial_jtag_write_bytes(serial_pkt, serial_pkt_size, USB_SERIAL_WRITE_MAX_TICKS_TO_WAIT);

    free(serial_pkt);
    
    if (bytes_written != serial_pkt_size) {
        DGL_DBG_INFO("usb_serial_jtag_write_bytes failed");
        return 0;
    }
    
    return bytes_written;
}

// Unused, when SEND_LOGS_AS_SERIAL_PACKETS is 0
[[maybe_unused]] static void send_c2u_log_msg(Log_Msg_Type type, const char* format, ...)
{
    va_list args;
    
    va_start(args, format);
    int msg_len = vsnprintf(NULL, 0, format, args);
    va_end(args);
    
    int pkt_log_msg_size = sizeof(Packet_C2U_Log_Msg) + (msg_len + 1) * sizeof(char);

    Packet_C2U_Log_Msg* pkt_log_msg = (Packet_C2U_Log_Msg*) create_packet(pkt_log_msg_size, PKT_C2U_LOG_MSG);
    pkt_log_msg->log_msg_type = type;
    pkt_log_msg->msg_length = msg_len + 1;
    
    va_start(args, format);
    vsnprintf(pkt_log_msg->msg, msg_len + 1, format, args);
    va_end(args);

    send_packet_to_user_via_serial_port((Packet*) pkt_log_msg);
    destroy_packet((Packet*) pkt_log_msg);
}

static bool is_esp_mac_addr_idexed(uint8_t* mac_addr)
{
    for (int esp_idx = 0; esp_idx < MAX_ESP_DEVICES; ++esp_idx) {
        if (memcmp(&dgl.esp_devices[esp_idx].mac_addr, mac_addr, ESP_NOW_ETH_ALEN) == 0) {
            return true;
        }
    }
    return false;
}

static ESP_IDX get_esp_idx_from_mac_addr(uint8_t* mac_addr)
{
    for (int idx = 0; idx < MAX_ESP_DEVICES; ++idx) {
        if (memcmp(&dgl.esp_devices[idx].mac_addr, mac_addr, ESP_NOW_ETH_ALEN) == 0) {
            return idx;
        }
    }
    DGL_ERROR("No esp indexed/detected with MAC: %X:%X:%X:%X:%X:%X. Send PKT_D2C_ADVERTISE for your esp to be indexed/detected.",
              mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    return INVALID_ESP_IDX;
}

static bool valid_esp_device_at_esp_idx(ESP_IDX esp_idx)
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

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    ESPNow_Send_Callback_Info send_cb;
    if (tx_info == NULL) {
        DGL_ERROR("espnow_send_cb: arg error");
        return;
    }

    memcpy(send_cb.mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    send_cb.status = status;
    if (!xQueueSend(dgl.espnow_send_callback_queue, &send_cb, 0)) {
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
    if (!xQueueSend(dgl.espnow_receive_callback_queue, &recv_cb, 0)) {
        DGL_DBG_INFO("espnow_recv_cb: receive queue fail");
        free(recv_cb.packet_data);
    }
}

static bool send_packet_to_drone_with_esp_now(uint8_t* mac_addr, Packet* pkt)
{
    if (esp_now_send(mac_addr, (uint8_t*) pkt, packet_size(pkt)) != ESP_OK) {
        DGL_ERROR("send_packet_to_drone_with_esp_now: send packet error");
        return false;
    }
    
    return true;
}

static void receive_d2c_drone_advertise(Packet* pkt, uint8_t* mac_addr)
{
    if (!is_esp_mac_addr_idexed(mac_addr))
    {
        bool found_and_inserted_in_empty_slot = false;
        for (int esp_idx = 0; esp_idx < MAX_ESP_DEVICES; ++esp_idx) {
            if (memcmp(&dgl.esp_devices[esp_idx].mac_addr, s_zero_mac, ESP_NOW_ETH_ALEN) == 0) {
                memcpy(&dgl.esp_devices[esp_idx].mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
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
    ESP_IDX esp_idx = get_esp_idx_from_mac_addr(mac_addr);
    if (esp_idx == INVALID_ESP_IDX) return;
    
    dgl.esp_devices[esp_idx].connected_state = ESP_CONNECTED;

    Packet_C2U_Drone_Connected* pkt_connected = (Packet_C2U_Drone_Connected*) create_packet(sizeof(Packet_C2U_Drone_Connected), PKT_C2U_DRONE_CONNECTED);
    if (pkt_connected == NULL) return;
    
    pkt_connected->esp_idx = esp_idx;

    send_packet_to_user_via_serial_port((Packet*) pkt_connected);
    destroy_packet((Packet*) pkt_connected);
}

static void receive_d2c_accept_disconnect(Packet* pkt, uint8_t* mac_addr)
{
    ESP_IDX esp_idx = get_esp_idx_from_mac_addr(mac_addr);
    if (esp_idx == INVALID_ESP_IDX) return;
    
    dgl.esp_devices[esp_idx].connected_state = ESP_DISCONNECTED;

    Packet_C2U_Drone_Disconnected* pkt_disconnected = (Packet_C2U_Drone_Disconnected*) create_packet(sizeof(Packet_C2U_Drone_Disconnected), PKT_C2U_DRONE_DISCONNECTED);
    if (pkt_disconnected == NULL) return;
    
    pkt_disconnected->esp_idx = esp_idx;

    send_packet_to_user_via_serial_port((Packet*) pkt_disconnected);
    destroy_packet((Packet*) pkt_disconnected);
}

static void receive_d2c_imu_data(Packet* pkt, uint8_t* mac_addr)
{
    Packet_D2C_Imu_Data* pkt_d2c_imu_data = (Packet_D2C_Imu_Data*) pkt;
    assert(pkt_d2c_imu_data->pkt.type == PKT_D2C_IMU_DATA);

    ESP_IDX esp_idx = get_esp_idx_from_mac_addr(mac_addr);
    if (esp_idx == INVALID_ESP_IDX) return;

    Packet_C2U_Imu_Data* pkt_c2u_imu_data = (Packet_C2U_Imu_Data*) create_packet(sizeof(Packet_C2U_Imu_Data), PKT_C2U_IMU_DATA);
    if (pkt_c2u_imu_data == NULL) return;

    memcpy(&pkt_c2u_imu_data->imu_data, &pkt_d2c_imu_data->imu_data, sizeof(Payload_Imu_Data));
    pkt_c2u_imu_data->esp_idx = esp_idx;
    
    send_packet_to_user_via_serial_port((Packet*) pkt_c2u_imu_data);
    destroy_packet((Packet*) pkt_c2u_imu_data);
}

static void xtask_try_to_connect_to_esp(void* voidp_esp_idx)
{
    ESP_IDX esp_idx = *((ESP_IDX*) voidp_esp_idx);
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
        
        send_packet_to_drone_with_esp_now(dgl.esp_devices[esp_idx].mac_addr, (Packet*) pkt_connect);
        vTaskDelay(TRY_TO_CONNECT_TO_ESP_SEND_DELAY);
    }

    destroy_packet((Packet*) pkt_connect);
    vTaskDelete(NULL); // Delete this task
}

static void receive_u2c_connect(Packet* pkt)
{
    assert(pkt->type == PKT_U2C_CONNECT);
    Packet_U2C_Connect* pkt_connect = (Packet_U2C_Connect*) pkt;

    ESP_IDX* esp_idx = malloc(sizeof(ESP_IDX));
    if (esp_idx == NULL) DGL_ABORT("receive_u2c_connect: malloc failed");
    
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
    ESP_IDX esp_idx = *((ESP_IDX*) voidp_esp_idx);
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
        
        send_packet_to_drone_with_esp_now(dgl.esp_devices[esp_idx].mac_addr, (Packet*) pkt_disconnect);
        vTaskDelay(TRY_TO_DISCONNECT_FROM_ESP_SEND_DELAY);
    }

    destroy_packet((Packet*) pkt_disconnect);
    vTaskDelete(NULL); // Delete this task
}

static void receive_u2c_disconnect(Packet* pkt)
{
    assert(pkt->type == PKT_U2C_DISCONNECT);
    Packet_U2C_Disconnect* pkt_disconnect = (Packet_U2C_Disconnect*) pkt;

    ESP_IDX* esp_idx = malloc(sizeof(ESP_IDX));
    if (esp_idx == NULL) DGL_ABORT("receive_u2c_disconnect: malloc failed");
    
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

static void receive_u2c_controll_commands(Packet* pkt)
{
    assert(pkt->type == PKT_U2C_CONTROLL_COMMANDS);
    Packet_U2C_Controll_Commands* pkt_u2c_ctrl_cmds = (Packet_U2C_Controll_Commands*) pkt;

    Packet_C2D_Controll_Commands* pkt_c2d_ctrl_cmds = (Packet_C2D_Controll_Commands*) create_packet(sizeof(Packet_C2D_Controll_Commands), PKT_C2D_CONTROLL_COMMANDS);
    memcpy(&pkt_c2d_ctrl_cmds->ctrl_cmds, &pkt_u2c_ctrl_cmds->ctrl_cmds, sizeof(Payload_Controll_Commands));

    if (!valid_esp_device_at_esp_idx(pkt_u2c_ctrl_cmds->esp_idx)) return;

    send_packet_to_drone_with_esp_now(dgl.esp_devices[pkt_u2c_ctrl_cmds->esp_idx].mac_addr, (Packet*) pkt_c2d_ctrl_cmds);
    destroy_packet((Packet*) pkt_c2d_ctrl_cmds);
}

static void receive_u2c_get_detected_drones(Packet* pkt)
{
    // Sends a PKT_C2U_DETECTED_DRONES back.
    
    int detected_drones_length = 0;
    for (ESP_IDX esp_idx = 0; esp_idx < MAX_ESP_DEVICES; ++esp_idx) {
        if (valid_esp_device_at_esp_idx(esp_idx)) ++detected_drones_length;
    }

    int detected_drones_pkt_size = sizeof(Packet_C2U_Detected_Drones) + detected_drones_length * sizeof(ESP_IDX);

    Packet_C2U_Detected_Drones* pkt_detected_drones = (Packet_C2U_Detected_Drones*) create_packet(detected_drones_pkt_size, PKT_C2U_DETECTED_DRONES);
    if (pkt_detected_drones == NULL) return;

    pkt_detected_drones->detected_drones_length = detected_drones_length;

    int detected_drones_idx = 0;
    for (ESP_IDX esp_idx = 0; esp_idx < MAX_ESP_DEVICES; ++esp_idx) {
        if (valid_esp_device_at_esp_idx(esp_idx)) {
            assert(detected_drones_idx < detected_drones_length);
            pkt_detected_drones->detected_drones[detected_drones_idx] = esp_idx;
            ++detected_drones_idx;
        }
    }

    send_packet_to_user_via_serial_port((Packet*) pkt_detected_drones);
    destroy_packet((Packet*) pkt_detected_drones);
    
    /* // For each detected drone the Serial_Packet_C2U_Detected_Drone is sent, then to inform that */
    /* // all detected drones have been sent, Serial_Packet_C2U_Detected_Drones_End is sent. */
    /* for (ESP_IDX esp_idx = 0; esp_idx < MAX_ESP_DEVICES; ++esp_idx) { */
    /*     if (valid_esp_device_at_esp_idx(esp_idx)) { */
    /*         Serial_Packet_C2U_Detected_Drone* spkt_detected_drone = (Serial_Packet_C2U_Detected_Drone*) create_serial_packet(sizeof(Serial_Packet_C2U_Detected_Drone)); */
    /*         if (spkt_detected_drone == NULL) continue; */

    /*         spkt_detected_drone->detected_drone = esp_idx; */

    /*         send_packet_to_user_via_serial_port((Serial_Packet*) spkt_detected_drone, sizeof(Serial_Packet_C2U_Detected_Drone)); */

    /*         free(spkt_detected_drone); */
    /*     } */
    /* } */

    /* Serial_Packet_C2U_Detected_Drones_End* spkt_detected_drones_end = (Serial_Packet_C2U_Detected_Drones_End*) create_serial_packet(sizeof(Serial_Packet_C2U_Detected_Drones_End)); */
    /* if (spkt_detected_drones_end == NULL) return; */

    /* send_packet_to_user_via_serial_port((Serial_Packet*) spkt_detected_drones_end, sizeof(Serial_Packet_C2U_Detected_Drones_End)); */

    /* free(spkt_detected_drones_end); */
}

static Packet* get_next_packet_from_user_via_serial_port()
{
    // First checks for the delimiter then reads the next 'generic' Packet which contains the packet payload size.
    // Then reads the payload and returns the combined actual Packet.

    size_t bytes_available = usb_serial_jtag_get_read_bytes_available();

    // There should be at least this many bytes
    if (bytes_available < sizeof(Serial_Delimiter) + sizeof(Packet)) return NULL;

    Serial_Delimiter delim;
    int bytes_read_delim = usb_serial_jtag_read_bytes(&delim, sizeof(Serial_Delimiter), USB_SERIAL_READ_MAX_TICKS_TO_WAIT);
    assert(bytes_read_delim == sizeof(Serial_Delimiter));

    if (delim != SERIAL_DELIMITER) {
        DGL_ERROR("Failed to read serial packet: Delimiter was not at the expected location");
        // TODO: 'skip to next delimiter'
        return NULL;
    }

    Packet pkt;
    int bytes_read_pkt = usb_serial_jtag_read_bytes(&pkt, sizeof(Packet), USB_SERIAL_READ_MAX_TICKS_TO_WAIT);
    assert(bytes_read_pkt == sizeof(Packet));

    int pkt_size = packet_size(&pkt);
    
    Packet* pkt_with_payload = create_packet(pkt_size, pkt.type); // create packet from pkt with 'correct' size (including payload)
    
    int bytes_read_payload = usb_serial_jtag_read_bytes(pkt_with_payload->payload, pkt_with_payload->payload_size, USB_SERIAL_READ_MAX_TICKS_TO_WAIT);
    
    if (bytes_read_payload != pkt_with_payload->payload_size) {
        DGL_ERROR("Failed to read payload of serial packet: No data available");
        return NULL;
    }

    return pkt_with_payload;
}

void dongle_loop()
{
    while (true)
    {
        // Handle incoming packets from the drone via ESP-Now
        
        ESPNow_Receive_Callback_Info recv_info = { 0 };
        if (xQueueReceive(dgl.espnow_receive_callback_queue, &recv_info, 0))
        {
            Packet* pkt = (Packet*) recv_info.packet_data;
            Packet_Info pkt_info = s_pkt_info_table[pkt->type];

            if (recv_info.packet_data_length == packet_size(pkt))
            {
                if (pkt_info.from_drone_handler != NULL) {
                    pkt_info.from_drone_handler(pkt, recv_info.mac_addr);
                }
                else DGL_ERROR("Received unexpected or unknown packet from the drone");
            }
            else DGL_ERROR("recv_info.packet_data_length != packet_size(pkt)");
        }
        destroy_espnow_receive_callback_info(recv_info);

        // Handle ESP-Now send Callback

        ESPNow_Send_Callback_Info send_info = { 0 };
        if (xQueueReceive(dgl.espnow_send_callback_queue, &send_info, 0))
        {
            if (send_info.status == ESP_NOW_SEND_FAIL) {
                uint8_t* mac_addr = send_info.mac_addr;
                DGL_DBG_INFO("Failed to send packge to ESP with MAC: %X:%X:%X:%X:%X:%X",
                            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
            }
        }

        // Handle incoming packets from the user via USB Serial/JTAG
        
        Packet* pkt = get_next_packet_from_user_via_serial_port();
        if (pkt != NULL)
        {
            DGL_DBG_INFO("Received packet!");
            Packet_Info pkt_info = s_pkt_info_table[pkt->type];
            
            if (pkt_info.from_user_handler != NULL) {
                pkt_info.from_user_handler(pkt);
            }
            else DGL_ERROR("Received unexpected or unknown packet from the user");
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
