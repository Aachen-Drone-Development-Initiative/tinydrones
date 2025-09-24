#pragma once

#include "./drone_and_controller_communication_types.h"

typedef int ESP_IDX;

typedef struct
{
    struct {
        int64_t time_us;
        float omega_x, omega_y, omega_z; // unit: rad/s
        float acc_x, acc_y, acc_z;       // unit: m/s^2
        bool contains_data;
    } imu;
    
} Package_Drone_To_UI;

typedef struct
{
    ESP_IDX drone_idx;
    bool is_armed;

    struct {
        int16_t pitch;
        int16_t yaw;
        int16_t roll;
        int16_t throttle;
        bool contains_data;
    } controller_commands;
    
} Package_UI_To_Drone;
