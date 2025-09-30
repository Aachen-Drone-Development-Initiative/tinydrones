
#include "../../shared_types/communication_packages.h"
#include "sdkconfig.h"

#include <string.h>
#include <assert.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "driver/usb_serial_jtag.h"

#define MAX_ESP_DEVICES (128)

#define RECEIVE_CALLBACK_QUEUE_SIZE (10)
#define SEND_CALLBACK_QUEUE_SIZE (10)
#define ESPNOW_CHANNEL (1)

#define ESPNOW_WIFI_MODE (WIFI_MODE_AP)
#define ESPNOW_WIFI_IF   (ESP_IF_WIFI_AP)

#define USB_SERIAL_JTAG_BUFFER_SIZE (1024)
#define USB_SERIAL_WRITE_MAX_TICKS_TO_WAIT (20 / portTICK_PERIOD_MS)
#define USB_SERIAL_READ_MAX_TICKS_TO_WAIT  (20 / portTICK_PERIOD_MS)

#define XTASK_DEFAULT_STACK_DEPTH (100)
#define XTASK_DEFAULT_PRIORITY    (10)

#define TRY_TO_CONNECT_TO_ESP_SEND_DELAY  (100 / portTICK_PERIOD_MS)
#define TRY_TO_CONNECT_TO_ESP_TIMEOUT     (1000 / portTICK_PERIOD_MS)
#define TRY_TO_DISCONNECT_FROM_ESP_SEND_DELAY  (100 / portTICK_PERIOD_MS)
#define TRY_TO_DISCONNECT_FROM_ESP_TIMEOUT     (1000 / portTICK_PERIOD_MS)

static const char *DONGLE_TAG = "espnow_dongle";

// Special mac address for broadcasting to all devices in range, not just a single device
static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint8_t s_zero_mac[ESP_NOW_ETH_ALEN]      = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} ESPNow_Send_Callback_Info;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *package_data;
    uint16_t package_data_length;
} ESPNow_Receive_Callback_Info;

void destroy_espnow_receive_callback_info(ESPNow_Receive_Callback_Info recv_info) {
    free(recv_info.package_data);
}

typedef struct {
    void (*from_drone_handler)(ESPNow_Receive_Callback_Info*);
    void (*from_user_handler)(Serial_Package*);
    int pkg_size;
    int serial_pkg_size;
} Package_Info;

static void receive_d2c_drone_advertise(ESPNow_Receive_Callback_Info*);
static void receive_d2c_accept_connection(ESPNow_Receive_Callback_Info*);
static void receive_d2c_accept_disconnect(ESPNow_Receive_Callback_Info*);
static void receive_2u_imu_data(ESPNow_Receive_Callback_Info*);

static void receive_2d_connect(Serial_Package*);
static void receive_2d_disconnect(Serial_Package*);
static void receive_2d_controll_commands(Serial_Package*);
static void receive_u2c_get_indexed_drones(Serial_Package*);

static const Package_Info s_pkg_info_table[PKG_TYPE_SIZE] = {
    [PKG_D2C_ADVERTISE] = {
        .pkg_size = sizeof(Package_D2C_Advertise),
        .from_drone_handler = receive_d2c_drone_advertise,
    },
    [PKG_D2C_ACCEPT_CONNECTION] = {
        .pkg_size = sizeof(Package_D2C_Accept_Connection),
        .from_drone_handler = receive_d2c_accept_connection,
    },
    [PKG_D2C_ACCEPT_DISCONNECT] = {
        .pkg_size = sizeof(Package_D2C_Accept_Disconnect),
        .from_drone_handler = receive_d2c_accept_disconnect,
    },
    [PKG_2U_IMU_DATA] = {
        .pkg_size = sizeof(Package_2U_Imu_Data),
        .serial_pkg_size = sizeof(Serial_Package_2U_Imu_Data),
        .from_drone_handler = receive_2u_imu_data,
    },
    [PKG_2D_CONNECT] = {
        .pkg_size = sizeof(Package_2D_Connect),
        .serial_pkg_size = sizeof(Serial_Package_2D_Connect),
        .from_user_handler = receive_2d_connect,
    },
    [PKG_2D_DISCONNECT] = {
        .pkg_size=sizeof(Package_2D_Disconnect),
        .serial_pkg_size=sizeof(Serial_Package_2D_Disconnect),
        .from_user_handler = receive_2d_disconnect,
    },
    [PKG_2D_CONTROLL_COMMANDS] = {
        .pkg_size=sizeof(Package_2D_Controll_Commands),
        .serial_pkg_size=sizeof(Serial_Package_2D_Controll_Commands),
        .from_user_handler = receive_2d_controll_commands,
    },
    [PKG_U2C_GET_INDEXED_DRONES] = {
        .serial_pkg_size = sizeof(Serial_Package_U2C_Get_Indexed_Drones),
        .from_user_handler = receive_u2c_get_indexed_drones,
    },
    [PKG_C2U_DRONE_CONNECTED] = {
        .serial_pkg_size=sizeof(Serial_Package_C2U_Drone_Connected),
    },
    [PKG_C2U_DRONE_DISCONNECTED] = {
        .serial_pkg_size=sizeof(Serial_Package_C2U_Drone_Disconnected)
    },
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
    ESP_LOGE(DONGLE_TAG, "No esp detected with MAC: %X:%X:%X:%X:%X:%X",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    return INVALID_ESP_IDX;
}

static bool valid_esp_device_at_esp_idx(ESP_IDX esp_idx)
{
    if (!(esp_idx >= 0 && esp_idx < MAX_ESP_DEVICES)) {
        ESP_LOGE(DONGLE_TAG, "ESP-Index '%d' is not inside the valid range of [0, %d]", esp_idx, MAX_ESP_DEVICES);
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
        ESP_LOGE(DONGLE_TAG, "espnow_send_cb: arg error");
        return;
    }

    memcpy(send_cb.mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    send_cb.status = status;
    if (!xQueueSend(dgl.espnow_send_callback_queue, &send_cb, 0)) {
        ESP_LOGW(DONGLE_TAG, "espnow_send_cb: send queue fail");
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int length)
{
    ESPNow_Receive_Callback_Info recv_cb;
    uint8_t* mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || length <= 0) {
        ESP_LOGE(DONGLE_TAG, "espnow_recv_cb: arg error");
        return;
    }

    memcpy(recv_cb.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb.package_data = malloc(length);
    if (recv_cb.package_data == NULL) {
        ESP_LOGE(DONGLE_TAG, "espnow_recv_cb: malloc failed");
        return;
    }
    
    memcpy(recv_cb.package_data, data, length);
    recv_cb.package_data_length = length;
    if (!xQueueSend(dgl.espnow_receive_callback_queue, &recv_cb, 0)) {
        ESP_LOGW(DONGLE_TAG, "espnow_recv_cb: receive queue fail");
        free(recv_cb.package_data);
    }
}

static void init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_LOGI(DONGLE_TAG, "NVS init done");
}

static void init_wifi(uint8_t wifi_channel, bool long_range_mode)
{
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());

    if (long_range_mode) {
        ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
    }

    ESP_LOGI(DONGLE_TAG, "WIFI init done");
}

static void init_espnow()
{
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );
    
    // Adding Boradcast peer to peer list.
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(DONGLE_TAG, "espnow_init: malloc failed");
        abort();
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = 0;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_broadcast_mac, sizeof(s_broadcast_mac));
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);
    
    ESP_LOGI(DONGLE_TAG, "ESPNOW init done");
}

static void init_usb_serial_jtag()
{
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = USB_SERIAL_JTAG_BUFFER_SIZE,
        .tx_buffer_size = USB_SERIAL_JTAG_BUFFER_SIZE,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    ESP_LOGI(DONGLE_TAG, "USB_SERIAL_JTAG init done");
}

static void init_dongle()
{
    init_nvs();
    init_wifi(ESPNOW_CHANNEL, false);
    init_espnow();
    init_usb_serial_jtag();

    // Initialize the global dongle state
    
    dgl.espnow_receive_callback_queue = xQueueCreate(RECEIVE_CALLBACK_QUEUE_SIZE, sizeof(ESPNow_Receive_Callback_Info));
    dgl.espnow_send_callback_queue    = xQueueCreate(SEND_CALLBACK_QUEUE_SIZE, sizeof(ESPNow_Send_Callback_Info));
    
    ESP_LOGI(DONGLE_TAG, "DONGLE Initialized!!!");
}

static Serial_Package* create_serial_package(int serial_packge_size)
{
    assert(serial_packge_size >= sizeof(Serial_Package));
    Serial_Package* spkg = malloc(serial_packge_size);
    if (spkg == NULL) {
        ESP_LOGE(DONGLE_TAG, "create_serial_package: malloc failed");
        return NULL;
    }

    spkg->delimiter = SERIAL_DELIMITER;
    spkg->pkg.type = PKG_INVALID;
    
    return spkg;
}

static int send_serial_package(Serial_Package* spkg, size_t spkg_size)
{
    if (spkg_size == 0) {
        ESP_LOGE(DONGLE_TAG, "send_serial_package: Serial Package has size 0");
    }
    
    int bytes_written = usb_serial_jtag_write_bytes((void*) spkg, spkg_size, USB_SERIAL_WRITE_MAX_TICKS_TO_WAIT);
        
    if (bytes_written != spkg_size) {
        ESP_LOGW(DONGLE_TAG, "usb_serial_jtag_write_bytes failed");
        return 0;
    }
    
    return bytes_written;
}

static Serial_Package* get_next_serial_package()
{
    // NOTE(YUZENI): First reads the next 'generic' Serial_Package which contains the size of the entire package,
    // including the payload. Then reads the playload and return the combined actual Serial_Package.
    
    Serial_Package spkg;
    int bytes_read = usb_serial_jtag_read_bytes(&spkg, sizeof(Serial_Package), USB_SERIAL_READ_MAX_TICKS_TO_WAIT);
    
    if (bytes_read != sizeof(Serial_Package)) {
        ESP_LOGE(DONGLE_TAG, "Failed to read package: No data available");
        return NULL;
    }

    if (spkg.delimiter != SERIAL_DELIMITER) {
        ESP_LOGE(DONGLE_TAG, "Failed to read package: Delimiter was not at the expected location in the serial-data steam");
        return NULL;
    }

    int spkg_size = s_pkg_info_table[spkg.pkg.type].serial_pkg_size;
    Serial_Package* spkg_with_payload = malloc(spkg_size);
    memcpy(spkg_with_payload, &spkg, sizeof(Serial_Package));
    int payload_size = spkg_size - sizeof(Serial_Package);
    
    bytes_read = usb_serial_jtag_read_bytes(spkg_with_payload->payload, payload_size, USB_SERIAL_READ_MAX_TICKS_TO_WAIT);
    
    if (bytes_read != payload_size) {
        ESP_LOGE(DONGLE_TAG, "Failed to read package-payload: No data available");
        return NULL;
    }

    return spkg_with_payload;
}

static bool send_package_to_drone_with_esp_now(uint8_t* mac_addr, Package* pkg)
{
    if (s_pkg_info_table[pkg->type].pkg_size == 0) {
        ESP_LOGE(DONGLE_TAG, "send_package_to_drone_with_esp_now: package has size 0");
        return false;
    }
    
    if (esp_now_send(mac_addr, (uint8_t*) pkg, s_pkg_info_table[pkg->type].pkg_size) != ESP_OK) {
        ESP_LOGE(DONGLE_TAG, "send_package_to_drone_with_esp_now: send package error");
        return false;
    }
    
    return true;
}

static void receive_d2c_drone_advertise(ESPNow_Receive_Callback_Info* recv_info)
{
    if (!is_esp_mac_addr_idexed(recv_info->mac_addr))
    {
        bool found_and_inserted_in_empty_slot = false;
        for (int esp_idx = 0; esp_idx < MAX_ESP_DEVICES; ++esp_idx) {
            if (memcmp(&dgl.esp_devices[esp_idx].mac_addr, s_zero_mac, ESP_NOW_ETH_ALEN) == 0) {
                memcpy(&dgl.esp_devices[esp_idx].mac_addr, &recv_info->mac_addr, ESP_NOW_ETH_ALEN);
                found_and_inserted_in_empty_slot = true;
                break;
            }
        }

        if (!found_and_inserted_in_empty_slot) {
            ESP_LOGE(DONGLE_TAG, "Failed to add detected ESP to the static detected-esps-array of size %d, "
                     "because all slots are full", MAX_ESP_DEVICES);
        }
    }
}

static void receive_d2c_accept_connection(ESPNow_Receive_Callback_Info* recv_info)
{
    ESP_IDX esp_idx = get_esp_idx_from_mac_addr(recv_info->mac_addr);
    if (esp_idx == INVALID_ESP_IDX) return;
                
    dgl.esp_devices[esp_idx].connected_state = ESP_CONNECTED;

    Serial_Package_C2U_Drone_Connected* spkg_connected = (Serial_Package_C2U_Drone_Connected*) create_serial_package(sizeof(Serial_Package_C2U_Drone_Connected));
    if (spkg_connected == NULL) return;
    spkg_connected->spkg.pkg.type = PKG_C2U_DRONE_CONNECTED;

    send_serial_package((Serial_Package*) spkg_connected, sizeof(Serial_Package_C2U_Drone_Connected));

    free(spkg_connected);
}

static void receive_d2c_accept_disconnect(ESPNow_Receive_Callback_Info* recv_info)
{
    ESP_IDX esp_idx = get_esp_idx_from_mac_addr(recv_info->mac_addr);
    if (esp_idx == INVALID_ESP_IDX) return;
                
    dgl.esp_devices[esp_idx].connected_state = ESP_DISCONNECTED;

    Serial_Package_C2U_Drone_Disconnected* spkg_disconnected = (Serial_Package_C2U_Drone_Disconnected*) create_serial_package(sizeof(Serial_Package_C2U_Drone_Disconnected));
    if (spkg_disconnected == NULL) return;
    
    spkg_disconnected->esp_idx = esp_idx;
    spkg_disconnected->spkg.pkg.type = PKG_C2U_DRONE_DISCONNECTED;

    send_serial_package((Serial_Package*) spkg_disconnected, sizeof(Serial_Package_C2U_Drone_Disconnected));

    free(spkg_disconnected);
}

static void receive_2u_imu_data(ESPNow_Receive_Callback_Info* recv_info)
{
    Package_2U_Imu_Data* pkg_imu_data = (Package_2U_Imu_Data*) recv_info->package_data;

    ESP_LOGI(DONGLE_TAG, "Received imu-data package:");
    ESP_LOGI(DONGLE_TAG, "imu_data: %ld, %f, %f, %f, %f, %f, %f", pkg_imu_data->imu_data.time_us,
             pkg_imu_data->imu_data.acc_x, pkg_imu_data->imu_data.acc_y, pkg_imu_data->imu_data.acc_z,
             pkg_imu_data->imu_data.omega_x, pkg_imu_data->imu_data.omega_y, pkg_imu_data->imu_data.omega_z);

    ESP_IDX esp_idx = get_esp_idx_from_mac_addr(recv_info->mac_addr);
    if (esp_idx == INVALID_ESP_IDX) return;

    Serial_Package_2U_Imu_Data* spkg_imu_data = (Serial_Package_2U_Imu_Data*) create_serial_package(sizeof(Serial_Package_2U_Imu_Data));
    if (spkg_imu_data == NULL) return;

    spkg_imu_data->esp_idx = esp_idx;
    memcpy(&spkg_imu_data->spkg.pkg, pkg_imu_data, sizeof(Package_2U_Imu_Data));
    
    send_serial_package((Serial_Package*) spkg_imu_data, sizeof(Serial_Package_2U_Imu_Data));
    free(spkg_imu_data);
}

static void xtask_try_to_connect_to_esp(void* voidp_spkg)
{
    // Copy the package, bacause spkg will be deleted while this task is running
    Serial_Package_2D_Connect* spkg_connect = malloc(sizeof(Serial_Package_2D_Connect));
    if (spkg_connect == NULL) {
        ESP_LOGE(DONGLE_TAG, "xtask_try_to_connect_to_esp: malloc failed");
        return;
    }
    memcpy(spkg_connect, voidp_spkg, sizeof(Serial_Package_2D_Connect));

    ESP_IDX esp_idx = spkg_connect->esp_idx;

    TickType_t tickcount_begin = xTaskGetTickCount();
    
    while (true)
    {
        if (!valid_esp_device_at_esp_idx(esp_idx))                            break;
        if (dgl.esp_devices[esp_idx].connected_state != ESP_TRY_TO_CONNECT)   break;

        TickType_t tickcount_now = xTaskGetTickCount();

        if (tickcount_now - tickcount_begin >= xTastickcnt_beginTRY_TO_CONNECT_TO_ESP_TIMEOUT) {
            ESP_LOGE(DONGLE_TAG, "xtask_try_to_connect_to_esp: Trying to connect to the esp at index '%d' Timed out after '%d'ms",
                     esp_idx, TRY_TO_CONNECT_TO_ESP_TIMEOUT * portTICK_PERIOD_MS);
        }
        
        send_package_to_drone_with_esp_now(dgl.esp_devices[esp_idx].mac_addr, (Package*) pkg_connect);
        vTaskDelay(TRY_TO_CONNECT_TO_ESP_SEND_DELAY);
    }

    free(pkg_connect);
    vTaskDelete(NULL); // Delete this task
}

static void receive_2d_connect(Serial_Package* spkg)
{
    if (dgl.esp_devices[spkg->esp_idx].connected_state == ESP_DISCONNECTED)
    { 
        dgl.esp_devices[spkg->esp_idx].connected_state = ESP_TRY_TO_CONNECT;
        BaseType_t task_create_result = xTaskCreate( xtask_try_to_connect_to_esp,
                                                     "try_to_connect_to_esp",
                                                     XTASK_DEFAULT_STACK_DEPTH,
                                                     (void*) spkg,
                                                     XTASK_DEFAULT_PRIORITY,
                                                     dgl.esp_devices[spkg->esp_idx].task_handle_try_to_connect);
        if (task_create_result != pdPASS) {
            ESP_LOGE(DONGLE_TAG, "receive_2d_connect: Failed to create task");
        }
    }
}

static void xtask_try_to_disconnect_from_esp(void* voidp_spkg)
{
    Serial_Package* spkg = (Serial_Package*) voidp_spkg;

    // Copy the package, bacause spkg will be deleted while this task is running
    Package_2D_Disconnect* pkg_disconnect = malloc(sizeof(Package_2D_Disconnect));
    memcpy(pkg_disconnect, &spkg->pkg, sizeof(Package_2D_Disconnect));

    ESP_IDX esp_idx = spkg->esp_idx;
    
    while (true)
    {
        if (!valid_esp_device_at_esp_idx(esp_idx))                               break;
        if (dgl.esp_devices[esp_idx].connected_state != ESP_TRY_TO_DISCONNECT)   break;
        
        send_package_to_drone_with_esp_now(dgl.esp_devices[esp_idx].mac_addr, (Package*) pkg_disconnect);
        vTaskDelay(TRY_TO_DISCONNECT_FROM_ESP_SEND_DELAY);
    }

    free(pkg_disconnect);
    vTaskDelete(NULL); // Delete this task
}

static void receive_2d_disconnect(Serial_Package* spkg)
{
    if (dgl.esp_devices[spkg->esp_idx].connected_state == ESP_CONNECTED)
    { 
        dgl.esp_devices[spkg->esp_idx].connected_state = ESP_TRY_TO_DISCONNECT;
        BaseType_t task_create_result = xTaskCreate(xtask_try_to_disconnect_from_esp,
                                                    "try_to_disconnect_from_esp",
                                                    XTASK_DEFAULT_STACK_DEPTH,
                                                    (void*) spkg,
                                                    XTASK_DEFAULT_PRIORITY,
                                                    dgl.esp_devices[spkg->esp_idx].task_handle_try_to_disconnect);
        if (task_create_result != pdPASS) {
            ESP_LOGE(DONGLE_TAG, "receive_2d_disconnect: Failed to create task");
        }
    }
}

static void receive_2d_controll_commands(Serial_Package* spkg)
{
    if (!valid_esp_device_at_esp_idx(spkg->esp_idx)) return;

    send_package_to_drone_with_esp_now(dgl.esp_devices[spkg->esp_idx].mac_addr, &spkg->pkg);
}

static void receive_u2c_get_indexed_drones(Serial_Package* spkg)
{
    // For each indexed drone the Serial_Package_C2U_Indexed_Drone is sent, then to inform that
    // all indexed drones have been sent, Serial_Package_C2U_Indexed_Drones_End is sent.
    for (ESP_IDX esp_idx = 0; esp_idx < MAX_ESP_DEVICES; ++esp_idx) {
        if (valid_esp_device_at_esp_idx(esp_idx)) {
            Serial_Package_C2U_Indexed_Drone* spkg_idxed_drone = (Serial_Package_C2U_Indexed_Drone*) create_serial_package(sizeof(Serial_Package_C2U_Indexed_Drone));
            if (spkg_idxed_drones == NULL) continue;

            spkg_idxed_drone->idxed_drone = esp_idx;

            send_serial_package((Serial_Package*) spkg_idxed_drone, sizeof(Serial_Package_C2U_Indexed_Drone));

            free(spkg_idxed_drones);
        }
    }

    Serial_Package_C2U_Indexed_Drones_End* spkg_idxed_drones_end = (Serial_Package_C2U_Indexed_Drone*) create_serial_package(sizeof(Serial_Package_C2U_Indexed_Drone));
    if (spkg_idxed_drones == NULL) return;

    send_serial_package((Serial_Package*) spkg_idxed_drone, sizeof(Serial_Package_C2U_Indexed_Drone));

    free(spkg_idxed_drones_end);
}

void dongle_loop()
{
    while (true)
    {
        // Handle incoming ESP-Now Packages
        
        ESPNow_Receive_Callback_Info recv_info = { 0 };
        if (xQueueReceive(dgl.espnow_receive_callback_queue, &recv_info, 0))
        {
            Package* pkg = (Package*) recv_info.package_data;
            Package_Info pkg_info = s_pkg_info_table[pkg->type];

            if (recv_info.package_data_length == pkg_info.pkg_size)
            {
                if (pkg_info.from_drone_handler != NULL) {
                    pkg_info.from_drone_handler(&recv_info);
                }
                else ESP_LOGE(DONGLE_TAG, "Received unexpected or unknown package from the drone");
            }
            else ESP_LOGE(DONGLE_TAG, "recv_info.package_data_length != pkg_info.pkg_size");
        }
        destroy_espnow_receive_callback_info(recv_info);

        // Handle ESP-Now send Callback

        ESPNow_Send_Callback_Info send_info = { 0 };
        if (xQueueReceive(dgl.espnow_send_callback_queue, &send_info, 0))
        {
            if (send_info.status == ESP_NOW_SEND_FAIL) {
                uint8_t* mac_addr = send_info.mac_addr;
                ESP_LOGI(DONGLE_TAG, "Failed to send packge to ESP with MAC: %X:%X:%X:%X:%X:%X",
                         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
            }
        }

        // Handle incoming Serial Packages
        
        Serial_Package* spkg = get_next_serial_package();
        if (spkg != NULL)
        {
            Package_Info pkg_info = s_pkg_info_table[spkg->pkg.type];
            
            if (valid_esp_device_at_esp_idx(spkg->esp_idx))
            {
                if (pkg_info.from_user_handler != NULL) {
                    pkg_info.from_user_handler(spkg);
                }
                else ESP_LOGE(DONGLE_TAG, "Received unexpected or unknown package from the user");
            }
            else ESP_LOGE(DONGLE_TAG, "There is no valid device at esp_idx '%'", spkg->esp_idx);
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }    
}

void app_main(void)
{
    init_dongle();
    
    dongle_loop();
    
    esp_restart();
}
