
// Package: Drone to controller

typedef enum {
    PKG_D2C_ADVERTISE   = 0,
    PKG_D2C_IMU_DATA,
    
    PKG_C2D_ACCEPT_ADVERTISEMENT,
    PKG_C2D_CONTROLL_COMMANDS,
} Package_Type;

typedef struct {
    uint8_t type;
    uint8_t payload[0];
} __attribute__((packed)) Package;

// type == PKG_D2C_REQUEST_CONNECTION
typedef struct { uint8_t empty[0] } __attribute__((packed)) Payload_Request_Connection; // empty payload

// type == PKG_D2C_IMU_DATA
typedef struct {
    int64_t time_us;
    float omega_x, omega_y, omega_z; // unit: rad/s
    float acc_x, acc_y, acc_z;       // unit: m/s^2
} __attribute__((packed)) Payload_Imu_Data;

// type == PKG_C2D_ACCEPT_CONNECTION
typedef struct { uint8_t empty[0] } __attribute__((packed)) Payload_Accept_Connection; // empty payload

// type == PKG_C2D_CONTROLL_COMMANDS
typedef struct {
    int16_t pitch;
    int16_t yaw;
    int16_t roll;
    int16_t throttle;
} __attribute__((packed)) Payload_Controller_Commands;
