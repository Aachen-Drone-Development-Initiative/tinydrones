#pragma once

#include "espnow_messaging.h"
#include "esp_log.h"

#define TINY_TAG "TINY"

#define TINY_LOG_ERROR(format, ...)                                     \
    do {                                                                \
        ESP_LOGE(TINY_TAG, format, ##__VA_ARGS__);                      \
        espnow_send_log_to_controller("ERROR (TINYDRONE): " format "\n", ##__VA_ARGS__); \
    } while (0);

#if TINYDRONE_ENABLE_DEBUG_LOGS
#define TINY_LOG_DEBUG(format, ...)                                     \
    do {                                                                \
        ESP_LOGD(TINY_TAG, format, ##__VA_ARGS__);                      \
        espnow_send_log_to_controller("DEBUG (TINYDRONE): " format "\n", ##__VA_ARGS__); \
    } while (0);
#else
#define TINY_LOG_DEBUG(...)
#endif

#define TINY_ASSERT(condition)                                          \
    do {                                                                \
        if (!(condition)) {                                             \
            TINY_LOG_ERROR("ASSERTION FAILED (TINYDRONE):%s:%d", __FILE__, __LINE__); \
            abort();                                                    \
        }                                                               \
    } while (0);

#define TINY_ABORT(...)                         \
    do {                                        \
        TINY_LOG_ERROR(__VA_ARGS__);            \
        abort();                                \
    } while (0);
