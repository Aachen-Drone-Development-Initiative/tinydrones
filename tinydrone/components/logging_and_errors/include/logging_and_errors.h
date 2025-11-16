#pragma once

#define SEND_LOGS_TO_CONTROLLER (0)
#define ENABLE_DBG_INFO_LOGS (1)

#if SEND_LOGS_TO_CONTROLLER
    #include "controller_communication.h"
    #include "communication_packets.h"

    #define TINY_ERROR(format, ...) ctrl_comm_send_log_msg(LOG_MSG_ERROR, format, ##__VA_ARGS__)
    #if ENABLE_DBG_INFO_LOGS
        #define TINY_DGB_INFO(format, ...) ctrl_comm_send_log_msg(LOG_MSG_DGB_INFO, format, ##__VA_ARGS__)
    #endif
#else
    #define TINY_ERROR(format, ...) ESP_LOGE("TINYDRONE", format, ##__VA_ARGS__)
    #if ENABLE_DBG_INFO_LOGS
        #define TINY_DGB_INFO(format, ...) ESP_LOGI("TINYDRONE", format, ##__VA_ARGS__)
    #endif
#endif

#ifndef TINY_DBG_INFO
    #define TINY_DBG_INFO(...)
#endif

// The error message will never be sent to the controller, regardless of SEND_LOGS_TO_CONTROLLER.
// This is necessary, when something fails that, sending stuff to the controller, depends on.
#define TINY_ERROR_NEVER_SEND(format, ...) ESP_LOGE("TINYDRONE", format, ##__VA_ARGS__)

#define TINY_ABORT(...)                                         \
    do {                                                        \
        ESP_LOGE("TINYDRONE <FATAL ERROR>", __VA_ARGS__);       \
        abort();                                                \
    } while (0);
