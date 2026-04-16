#include "esp_event.h"
#include "esp_netif.h"
// #include "mosq_broker.h"
// #include "mqtt_client.h"
// #include "nvs_flash.h"
#include <string.h>

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data);

static void mqtt_app_start(struct mosq_broker_config *config);

void wifi_init_sta(void);

esp_err_t wifi_setup(void);
