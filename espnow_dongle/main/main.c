#include <assert.h>
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>

#define ESPNOW_CHANNEL 1
#define ESPNOW_UNICAST_SEND_COUNT 0
#define ESPNOW_SEND_DELAY_MS 10
#define ESPNOW_SEND_LENGTH 64

#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP

// Special mac address for broadcasting to all devices in range, not just a single device
static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
    ESPNOW_DATA_MAX,
};

typedef struct {
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                       //Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                       //Delay between sending two ESPNOW data, unit: ms.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
    int length;
    uint8_t *buffer;
} ESPNow_Send_Param;

typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint8_t payload[0];                   //Real payload of ESPNOW data.
} __attribute__((packed)) ESPNow_Data;

typedef enum {
    EVENT_RECEIVE_CALLBACK,
    EVENT_SEND_CALLBACK,
} ESPNow_Event_Type;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} example_espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} ESPNow_Event_Receive_Callback;

typedef struct {
    ESPNow_Event_Type type;
    union {
        
    };
    
} ESPNow_Event;

/* WiFi should start before using ESPNOW */
// wifi_init(ESPNOW_CHANNEL, false);
static void wifi_init(uint8_t wifi_channel, bool long_range_mode)
{
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE));

    if (long_range_mode) {
        ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
    }
}

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    example_espnow_event_t evt;
    example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

    if (tx_info == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }

    evt.id = EXAMPLE_ESPNOW_SEND_CB;
    memcpy(send_cb->mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
    send_cb->status = status;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    
    example_espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;
    uint8_t * des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (IS_BROADCAST_ADDR(des_addr)) {
        /* If added a peer with encryption before, the receive packets may be
         * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
         * Users can check the destination address to distinguish it.
         */
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
    } else {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }

    evt.id = EXAMPLE_ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb->data);
    }
}

void espnow_data_prepare(ESPNow_Send_Param *send_param)
{
    ESPNow_Data *buf = (ESPNow_Data*)send_param->buffer;

    assert(send_param->length >= sizeof(ESPNow_Data));

    buf->type = ESPNOW_DATA_BROADCAST;
    buf->state = send_param->state;
    buf->seq_num = 0; //s_example_espnow_seq[buf->type]++;
    buf->magic = send_param->magic;
    /* /\* Fill all remaining bytes after the data with random values *\/ */
    /* esp_fill_random(buf->payload, send_param->len - sizeof(example_espnow_data_t)); */
    buf->crc = 0;
    buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->length);

    // random payload
    esp_fill_random(buf->payload, send_param->length - sizeof(ESPNow_Data));
}

static ESPNow_Send_Param* espnow_init()
{
    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );
    
    /* Set primary master key. */
    /* ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) ); */

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        printf("Malloc failed");
        abort();
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_broadcast_mac, sizeof(s_broadcast_mac));
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    ESPNow_Send_Param* send_param = malloc(sizeof(ESPNow_Send_Param));
    if (send_param == NULL) {
        printf("Malloc failed");
        abort();
    }
    memset(send_param, 0, sizeof(ESPNow_Send_Param));
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = ESPNOW_UNICAST_SEND_COUNT;
    send_param->delay = ESPNOW_SEND_DELAY_MS;
    send_param->length = ESPNOW_SEND_LENGTH;
    send_param->buffer = malloc(ESPNOW_SEND_LENGTH);
    if (send_param->buffer == NULL) {
        printf("Malloc fail");
        abort();
    }
    memcpy(send_param->dest_mac, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    espnow_data_prepare(send_param);

    return send_param;
}

void app_main(void)
{
    // Initialize esp now
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_init();

    ESPNow_Send_Param *send_param = espnow_init();
    
    while (true)
    {
        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->length) != ESP_OK) {
            printf("Send error");
            abort();
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
