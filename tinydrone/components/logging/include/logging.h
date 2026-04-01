#pragma once

#include "espnow_messaging.h"
#include "esp_log.h"

#define TINY_TAG "TINY"

#define LOG_ERROR(format, ...)                      \
    do {                                            \
        ESP_LOGE(TINY_TAG, format, ##__VA_ARGS__);  \
    } while (0);

// Also send the error message to the controller
#define CONTR_LOG_ERROR(format, ...)                                    \
    do {                                                                \
        espnow_send_msg_to_controller("ERROR (TINYDRONE): " format "\n", ##__VA_ARGS__); \
        ESP_LOGE(TINY_TAG, format, ##__VA_ARGS__);                      \
    } while (0);


#define TINYDRONE_ENABLE_DEBUG_LOGS 1

#if TINYDRONE_ENABLE_DEBUG_LOGS

#define LOG_DEBUG(format, ...)                      \
    do {                                            \
        ESP_LOGI(TINY_TAG, format, ##__VA_ARGS__);  \
    } while (0);

// Also send the debug message to the controller
#define CONTR_LOG_DEBUG(format, ...)                                    \
    do {                                                                \
        espnow_send_msg_to_controller("DEBUG (TINYDRONE): " format "\n", ##__VA_ARGS__); \
        ESP_LOGI(TINY_TAG, format, ##__VA_ARGS__);                      \
    } while (0);

#else

#define LOG_DEBUG(...)
#define CONTR_LOG_DEBUG(...)

#endif
