
#include "../../shared_types/drone_and_controller_communication_types.h"
#include "dynamic_array.h"

#include <string.h>
#include <assert.h>
#include "esp_log.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "driver/usb_serial_jtag.h"

/* #include "driver/uart.h" */
/* #include "driver/gpio.h" */

#define RECEIVE_CALLBACK_QUEUE_SIZE 10
#define SEND_CALLBACK_QUEUE_SIZE 10
#define ESPNOW_CHANNEL 1

#define ESPNOW_SEND_DELAY_MS 10

#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP

#define USB_SERIAL_JTAG_BUFFER_SIZE (1024)
#define USB_SERIAL_WRITE_MAX_TICKS_TO_WAIT (20 / portTICK_PERIOD_MS)
#define USB_SERIAL_READ_MAX_TICKS_TO_WAIT (20 / portTICK_PERIOD_MS)

static const char *DONGLE_TAG = "espnow_dongle";

// Special mac address for broadcasting to all devices in range, not just a single device
static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* #define IS_BROADCAST_ADDR(addr) (memcmp((addr), s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0) */

typedef struct {
    uint8_t state;                        // Indicate that if has received broadcast ESPNOW data or not.
    uint16_t delay;                       // Delay between sending two ESPNOW data, unit: ms.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   // MAC address of destination device.
    int length;
    uint8_t *buffer;
} ESPNow_Send_Param;

/* typedef struct { */
/*     uint8_t state;                        // Indicate that if has received broadcast ESPNOW data or not. */
/*     uint32_t magic;                       // Magic number which is used to determine which device to send unicast ESPNOW data. */
/*     uint8_t payload[0];                   // Real payload of ESPNOW data. */
/* } __attribute__((packed)) ESPNow_Broadcast_Package; */

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} ESPNow_Send_Callback_Info;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *package_data;
    uint16_t package_data_length;
} ESPNow_Receive_Callback_Info;

void destroy_espnow_receive_callback_info(ESPNow_Receive_Callback_Info recv_info)
{
    free(recv_info.package_data);
}

typedef struct {
    uint8_t src_mac_addr[6]; // Source MAC
    size_t length;
    uint8_t package_data[0];
} ESPNow_Package;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
} ESP_Device;

typedef struct {
    ESP_Device *items;
    size_t count;
    size_t capacity;
} DA_Detected_ESPs;

typedef struct {
    ESP_Device *items;
    size_t count;
    size_t capacity;
} DA_Connected_ESPs;

static struct {
    QueueHandle_t espnow_send_callback_queue;
    QueueHandle_t espnow_receive_callback_queue;

    DA_Detected_ESPs da_detected_esps;
    DA_Connected_ESPs da_connected_esps;
} dgl = { 0 };

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
        abort();
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

    // Initialize the dongle state
    
    dgl.espnow_receive_callback_queue = xQueueCreate(RECEIVE_CALLBACK_QUEUE_SIZE, sizeof(ESPNow_Receive_Callback_Info));
    dgl.espnow_send_callback_queue    = xQueueCreate(SEND_CALLBACK_QUEUE_SIZE, sizeof(ESPNow_Send_Callback_Info));
    
    da_reserve(&dgl.da_detected_esps, 8);
    da_reserve(&dgl.da_connected_esps, 8);

    ESP_LOGI(DONGLE_TAG, "DONGLE Initialized!!!");
}

/* void espnow_data_prepare(ESPNow_Send_Param *send_param) */
/* { */
/*     ESPNow_Data *buf = (ESPNow_Data*)send_param->buffer; */

/*     assert(send_param->length >= sizeof(ESPNow_Data)); */

/*     buf->type = ESPNOW_DATA_BROADCAST; */
/*     buf->state = send_param->state; */
/*     buf->seq_num = 0; //s_example_espnow_seq[buf->type]++; */
/*     buf->magic = send_param->magic; */
/*     /\* /\\* Fill all remaining bytes after the data with random values *\\/ *\/ */
/*     /\* esp_fill_random(buf->payload, send_param->len - sizeof(example_espnow_data_t)); *\/ */
/*     buf->crc = 0; */
/*     buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->length); */

/*     // random payload */
/*     esp_fill_random(buf->payload, send_param->length - sizeof(ESPNow_Data)); */
/* } */


void dongle_loop()
{
    while (true)
    {
        Package_Drone_To_UI drone_to_ui = { 0 };
        
        ESPNow_Receive_Callback_Info recv_info = { 0 };
        if (xQueueReceive(dgl.espnow_receive_callback_queue, &recv_info, 0))
        {
            Package package = *((Package*) recv_info.package_data);
            switch(package.type) {
            case PKG_D2C_ADVERTISE:
                if (recv_info.package_data_length != sizeof(Payload_Accept_Connection) + sizeof(Payload_Imu_Data)) {
                    ESP_LOGE(DONGLE_TAG, "recv_info.package_data_length != sizeof(Payload_Accept_Connection) + sizeof(Payload_Imu_Data)");
                    break;
                }

                bool already_detected = false;
                for (int i = 0; i < dgl.da_detected_esps.count; ++i) {
                    if (memcmp(&dgl.da_detected_esps.items[i].mac_addr, recv_info.mac_addr, ESP_NOW_ETH_ALEN) == 0) {
                        already_detected = true;
                    }
                }

                if (!already_detected) {
                    ESP_Device new_device = { 0 };
                    memcpy(&new_device.mac_addr, &recv_info.mac_addr, ESP_NOW_ETH_ALEN);
                    da_append(&dgl.da_detected_esps, new_device);
                }

                break;
                
            case PKG_D2C_IMU_DATA:
                if (recv_info.package_data_length != sizeof(Package) + sizeof(Payload_Imu_Data)) {
                    ESP_LOGE(DONGLE_TAG, "recv_info.package_data_length != sizeof(Package) + sizeof(Payload_Imu_Data)");
                    break;
                }

                Payload_Imu_Data imu_data = *((Payload_Imu_Data*) package.payload);

                ESP_LOGI(DONGLE_TAG, "Received imu-data package:");
                ESP_LOGI(DONGLE_TAG, "imu_data: %ld, %f, %f, %f, %f, %f, %f",
                         imu_data.time_us, imu_data.acc_x, imu_data.acc_y, imu_data.acc_z, imu_data.omega_x, imu_data.omega_y, imu_data.omega_z);
                
                drone_to_ui.imu.time_us = imu_data.time_us;
                drone_to_ui.imu.acc_x = imu_data.acc_x;
                drone_to_ui.imu.acc_y = imu_data.acc_y;
                drone_to_ui.imu.acc_z = imu_data.acc_z;
                drone_to_ui.imu.omega_x = imu_data.omega_x;
                drone_to_ui.imu.omega_y = imu_data.omega_y;
                drone_to_ui.imu.omega_z = imu_data.omega_z;
                drone_to_ui.imu.contains_data = true;
                
                break;
                
            default:
                ESP_LOGE(DONGLE_TAG, "Received package with wrong or unknown type");
                break;
            }
            
            destroy_espnow_receive_callback_info(recv_info);
        }

        ESPNow_Send_Callback_Info send_info = { 0 };
        if (xQueueReceive(dgl.espnow_send_callback_queue, &send_info, 0))
        {
            if (send_info.status == ESP_NOW_SEND_FAIL) {
                uint8_t* mac_addr = send_info.mac_addr;
                ESP_LOGI(DONGLE_TAG, "Failed to send packge to ESP with MAC: %X:%X:%X:%X:%X:%X",
                         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
            }
        }

        // Communicate with all connected esp devices

        for (int i = 0; i < dgl.da_connected_esps.count; ++i) {
            /* if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->length) != ESP_OK) { */
            /*     ESP_LOGE(DONGLE_TAG, "Send package error"); */
            /* }             */
        }

        // Receive UI-to-Drone Package

        Package_UI_To_Drone ui_to_drone = { 0 };
        int bytes_read =  usb_serial_jtag_read_bytes(&ui_to_drone, sizeof(Package_UI_To_Drone), USB_SERIAL_READ_MAX_TICKS_TO_WAIT);
        if (bytes_read != sizeof(Package_UI_To_Drone))
        {
            if (ui_to_drone.controller_commands.contains_data)
            {
                Payload_Controller_Commands ctrl_cmds = { 
                    .pitch = ui_to_drone.controller_commands.pitch,
                    .yaw = ui_to_drone.controller_commands.yaw,
                    .roll = ui_to_drone.controller_commands.roll,
                    .throttle = ui_to_drone.controller_commands.throttle
                };

                Package package = {
                    .type = PKG_C2D_CONTROLL_COMMANDS,
                    .payload = ,
                };

                memcpy(ctrl_cmds, package.payload, sizeof(Payload_Controller_Commands));

                if (esp_now_send(send_param->dest_mac, &package, send_param->length) != ESP_OK) {
                    ESP_LOGE(DONGLE_TAG, "Send package error");
                }
            }
        }
        else {
            ESP_LOGW(DONGLE_TAG, "usb_serial_jtag_read_bytes failed");
        }

        // Send Drone-to-UI Package

        if (usb_serial_jtag_write_bytes(&drone_to_ui, sizeof(Package_Drone_To_UI), USB_SERIAL_WRITE_MAX_TICKS_TO_WAIT) == 0) {
            ESP_LOGW(DONGLE_TAG, "usb_serial_jtag_write_bytes failed");
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
