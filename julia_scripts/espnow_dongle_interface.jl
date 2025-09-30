module Dongle

@enum Package_Type::UInt8 begin
    PKG_INVALID = 0

    # 2U - (Drone/Controller) to User
    PKG_2U_IMU_DATA = 4

    # 2D - (User/Controller) to Drone
    PKG_2D_CONNECT = 5
    PKG_2D_DISCONNECT = 6
    PKG_2D_CONTROLL_COMMANDS = 7

    # U2C - User to Controller
    PKG_U2C_GET_INDEXED_DRONES = 8

    # C2U - Controller to User
    PKG_C2U_DRONE_CONNECTED = 9
    PKG_C2U_DRONE_DISCONNECTED = 10
    PKG_C2U_INDEXED_DRONES_LIST = 11
end

struct Package
    type::UInt8
end

const Serial_Delimiter = UInt32
const SERIAL_DELIMITER = Serial_Delimiter(0x796E6974) // "tiny" in ascii-codes

const ESP_IDX = Int32
const INVALID_ESP_IDX = -1

struct Serial_Package
    delimiter::Serial_Delimiter
    pkg::Package
end

#############################################
# Packages 2D ((User/Controller) to Drone)
#############################################

# type == PKG_2D_CONNECT
struct Serial_Package_2D_Connect
    spkg::Serial_Package
    esp_idx::ESP_IDX
end

# type == PKG_2D_DISCONNECT
struct Serial_Package_2D_Disconnect
    spkg::Serial_Package
    esp_idx::ESP_IDX
end

@enum Controll_State_Flags::UInt16 begin
    DRONE_ARMED = UInt16(1 << 0)
end

struct Payload_Controll_Commands
    ctrl_state_flags::Controll_State_Flags
    pitch::UInt16
    yaw::UInt16
    roll::UInt16
    throttle::UInt16
end

struct Serial_Package_2D_Controll_Commands
    spkg::Serial_Package
    ctrl_cmds::Payload_Controll_Commands
    esp_idx::ESP_IDX
end

#############################################
# Packages U2C (User to Controller)
#############################################

# type == PKG_U2C_GET_INDEXED_DRONES
struct Serial_Package_U2C_Get_Indexed_Drones
    spkg::Serial_Package
    esp_idx::ESP_IDX
end

#############################################
# Packages C2U (Controller to User)
#############################################

# type == PKG_C2U_DRONE_CONNECTED
struct Serial_Package_C2U_Drone_Connected
    spkg::Serial_Package
    esp_idx::ESP_IDX
end

# type == PKG_C2U_DRONE_DISCONNECTED
struct Serial_Package_C2U_Drone_Disconnected
    spkg::Serial_Package
    esp_idx::ESP_IDX
end

# type == PKG_C2U_INDEXED_DRONE
struct Serial_Package_C2U_Indexed_Drone
    spkg::Serial_Package
    idxed_drone::ESP_IDX
end

# type == PKG_C2U_INDEXED_DRONES_END
struct Serial_Package_C2U_Indexed_Drones_End
    spkg::Serial_Package
end

#############################################
# Packages 2U ((Drone/Controller) to User)
#############################################

struct Payload_Imu_Data
    time_us::Int64
    omega_x::Float32
    omega_y::Float32
    omega_z::Float32
    acc_x::Float32
    acc_y::Float32
    acc_z::Float32
end

# type == PKG_2U_IMU_DATA
struct Serial_Package_2U_Imu_Data
    spkg::Serial_Package
    imu_data::Payload_Imu_Data
    esp_idx::ESP_IDX
end

#############################################
# Controller Communication Functions
#############################################

function send_serial_package(Seria)

"Returns a vector of the assigned indices for all drones in the range of the dongle which were indexed by the dongle."
function detected_drones()::Vector{ESP_IDX}
    
end

end # module Dongle
