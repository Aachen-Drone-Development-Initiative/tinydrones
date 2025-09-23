#include <assert.h>
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "sdkconfig.h"
#include "MPU6050.h"

#include <stdio.h>
#include <string.h>

#define I2C_MASTER_SDA_IO GPIO_NUM_11 // GPIO number for I2C SDA
#define I2C_MASTER_SCL_IO GPIO_NUM_10 // GPIO number for I2C SCL

typedef struct {
    uint8_t src_mac_addr[6]; // Source MAC
    uint8_t payload[ESP_NOW_MAX_DATA_LEN_V2];
    size_t len;
} espnow_packet_t;

void nvs_init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
}

void minimal_wifi_init() {
    // Initialize Wi-Fi in station mode
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    esp_event_loop_create_default();

    ESP_LOGI("wifi", "Initializing Wifi");

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void)
{
    // I2C Setup
    i2c_master_bus_config_t i2c_config = {
        .i2c_port = I2C_NUM_0, // use i2c controller 0
        .sda_io_num = I2C_MASTER_SDA_IO, // use GPIO 21 for SDA
        .scl_io_num = I2C_MASTER_SCL_IO, // use GPIO 22 for SCL
        .glitch_ignore_cnt = 7, // recommended value from documentation
        .intr_priority = 2, // medium priority
        .clk_source = I2C_CLK_SRC_DEFAULT, // default clock source
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t master_bus;
    i2c_new_master_bus(&i2c_config, &master_bus); // initialize I2C master bus

    MPU6050_Handle mpu = mpu6050_init(master_bus, MPU6050_PLUS_MINUS_500_DEG_PER_SECOND, MPU6050_PLUS_MINUS_2G);
    if (mpu.i2c_handle == NULL) return;

    // ESP NOW

    // Initialize NVS, Wi-Fi and ESP-NOW
    nvs_init();
    minimal_wifi_init();
    ESP_ERROR_CHECK(esp_now_init());

    // Register Broadcast MAC Address
    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    memcpy(peerInfo.peer_addr, (uint8_t[]){0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, 6); 
    peerInfo.ifidx = WIFI_IF_STA;
    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));

    espnow_packet_t* packet = malloc(sizeof(espnow_packet_t));
    
    while (true)
    {
        MPU6050_Data mpu_data = mpu6050_get_data(mpu);

        packet->len = sizeof(mpu_data);
        memcpy(packet->payload, &mpu_data, sizeof(mpu_data));
        esp_now_send((uint8_t[]){0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, packet->payload, packet->len);
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
