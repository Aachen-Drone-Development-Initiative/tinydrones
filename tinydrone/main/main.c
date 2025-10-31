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
#include "esp_now_interface.h"

#include <stdio.h>
#include <string.h>

#include "../../shared_types/communication_packets.h"

#define I2C_MASTER_SDA_IO GPIO_NUM_11 // GPIO number for I2C SDA
#define I2C_MASTER_SCL_IO GPIO_NUM_10 // GPIO number for I2C SCL
#define ADVERTISEMENT_INTERVAL_MS 2000

typedef struct {
    bool espnow_controller_connected;
    uint8_t espnow_controller_mac[6];
} Drone_Status;

static Drone_Status drone_status = {
    .espnow_controller_connected = false
};

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

static Packet* create_packet(int packet_size, Packet_Type type)
{
    assert(packet_size >= sizeof(Packet));
    Packet* pkt = malloc(packet_size);
    if (pkt == NULL) return NULL;

    pkt->type = type;
    pkt->payload_size = packet_size - sizeof(Packet);
    
    return pkt;
}

static void destroy_packet(Packet* pkt)
{
    free(pkt);
}

/* static void xtask_consume_espnow_receive_queue(void*) */
/* { */
/*     while (true) */
/*     { */
/*         vTaskDelay(/\**\/); */
/*     } */

/*     vTaskDelete(NULL); // Delete this task */
/* } */

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

    // Initialize NVS, Wi-Fi and ESP-NOW
    nvs_init();
    minimal_wifi_init();
    esp_now_setup();

    // Register Broadcast MAC Address
    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    memcpy(peerInfo.peer_addr, (uint8_t[]){0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, 6); 
    peerInfo.ifidx = WIFI_IF_STA;
    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));

    // Connect to espnow controller
    ESPnow_Controller_Connection connection;
    advertise_drone_espnow(&connection, ADVERTISEMENT_INTERVAL_MS);
    drone_status.espnow_controller_connected = connection.connected;
    memcpy(drone_status.espnow_controller_mac, connection.mac_addr, 6);


    n// Main loop: read MPU6050 data and send via ESP-NOW
    Package_Imu_Data* package = malloc(sizeof(Package_Imu_Data));

    while (true)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* printf("imu_packet_size: %d\n", sizeof(Packet_D2C_Imu_Data)); */
        MPU6050_Data mpu_data = mpu6050_get_data(mpu);

        package->pkg.type = PKG_D2C_IMU_DATA;
        package->time_us = mpu_data.time_us;
        package->omega_x = mpu_data.omega_x;
        package->omega_y = mpu_data.omega_y;
        package->omega_z = mpu_data.omega_z;
        package->acc_x = mpu_data.acc_x;
        package->acc_y = mpu_data.acc_y;
        package->acc_z = mpu_data.acc_z;

        send_package_espnow(drone_status.espnow_controller_mac, (Package *) package);
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    /* while (true) */
    /* { */
    /*     // Handle incoming packets from the controller via ESP-Now */
        
    /*     ESPNow_Receive_Callback_Info recv_info = { 0 }; */
    /*     if (xQueueReceive(dgl.espnow_receive_callback_queue, &recv_info, 0)) */
    /*     { */
    /*         Packet* pkt = (Packet*) recv_info.packet_data; */
    /*         Packet_Info pkt_info = s_pkt_info_table[pkt->type]; */

    /*         if (recv_info.packet_data_length == packet_size(pkt)) */
    /*         { */
    /*             if (pkt_info.from_drone_handler != NULL) { */
    /*                 pkt_info.from_drone_handler(pkt, recv_info.mac_addr); */
    /*             } */
    /*             else DGL_ERROR("Received unexpected or unknown packet from the drone"); */
    /*         } */
    /*         else DGL_ERROR("recv_info.packet_data_length != packet_size(pkt)"); */
    /*     } */
    /*     destroy_espnow_receive_callback_info(recv_info); */

    /*     // Handle ESP-Now send Callback */

    /*     ESPNow_Send_Callback_Info send_info = { 0 }; */
    /*     if (xQueueReceive(dgl.espnow_send_callback_queue, &send_info, 0)) */
    /*     { */
    /*         if (send_info.status == ESP_NOW_SEND_FAIL) { */
    /*             uint8_t* mac_addr = send_info.mac_addr; */
    /*             DGL_DBG_INFO("Failed to send packge to ESP with MAC: %X:%X:%X:%X:%X:%X", */
    /*                         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]); */
    /*         } */
    /*     } */

    /*     // Handle incoming packets from the user via USB Serial/JTAG */
        
    /*     Packet* pkt = get_next_packet_from_user_via_serial_port(); */
    /*     if (pkt != NULL) */
    /*     { */
    /*         DGL_DBG_INFO("Received packet!"); */
    /*         Packet_Info pkt_info = s_pkt_info_table[pkt->type]; */
            
    /*         if (pkt_info.from_user_handler != NULL) { */
    /*             pkt_info.from_user_handler(pkt); */
    /*         } */
    /*         else DGL_ERROR("Received unexpected or unknown packet from the user"); */
    /*     } */
    /*     destroy_packet(pkt); */
        
    /*     vTaskDelay(DONGLE_MAIN_LOOP_DELAY); */
    /* } */
}
