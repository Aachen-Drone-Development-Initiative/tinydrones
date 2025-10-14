
module Dongle

include("./serialportlib/serialportlib.jl")
using .Serialportlib
using Base: packedsize

const Serial_Delimiter = UInt32
const SERIAL_DELIMITER = Serial_Delimiter(0x796E6974) # "tiny" in ascii-codes

const ESP_IDX = Int32
const INVALID_ESP_IDX = -1

@enum Package_Type::UInt8 begin
    PKT_INVALID = 0

    PKT_D2C_ADVERTISE = 1
    
    PKT_U2C_CONNECT = 2
    PKT_C2D_CONNECT = 3
    PKT_D2C_ACCEPT_CONNECTION = 4
    PKT_C2U_DRONE_CONNECTED = 5
    
    PKT_U2C_DISCONNECT = 6
    PKT_C2D_DISCONNECT = 7
    PKT_D2C_ACCEPT_DISCONNECT = 8
    PKT_C2U_DRONE_DISCONNECTED = 9
    
    PKT_U2C_CONTROLL_COMMANDS = 10
    PKT_C2D_CONTROLL_COMMANDS = 11

    PKT_U2C_GET_DETECTED_DRONES = 12
    PKT_C2U_DETECTED_DRONES = 13

    PKT_D2C_IMU_DATA = 14
    PKT_C2U_IMU_DATA = 15
    
    PKT_C2U_LOG_MSG = 16
end

abstract type Packet_U2C end
abstract type Packet_C2U end

@kwdef struct Basic_Packet
    type::Package_Type    = PKT_INVALID
    payload_size::Int16   = 0
end

# type == PKT_U2C_CONNECT
struct Packet_U2C_Connect <: Packet_U2C
    pkt::Basic_Packet
    esp_idx::ESP_IDX
end

function Packet_U2C_Connect(esp_idx)
    Packet_U2C_Connect(Basic_Packet(PKT_U2C_CONNECT, sizeof(ESP_IDX)), esp_idx)
end

# type == PKT_C2U_DRONE_CONNECTED
struct Packet_C2U_Drone_Connected <: Packet_C2U
    pkt::Basic_Packet
    esp_idx::ESP_IDX
end

struct Receive_Drone_Connected
    esp_idx::ESP_IDX
end

function read_and_eval_payload(::Val{PKT_C2U_DRONE_CONNECTED}, sp_idx)
    esp_idx = Serialportlib.read_bytes_as_object(ESP_IDX, sp_idx=sp_idx)
    return Receive_Drone_Connected(esp_idx)
end
    
# type == PKT_U2C_DISCONNECT
struct Packet_U2C_Disconnect <: Packet_U2C
    pkt::Basic_Packet
    esp_idx::ESP_IDX
end

function Packet_U2C_Disconnect(esp_idx)
    Packet_U2C_Disconnect(Basic_Packet(PKT_U2C_DISCONNECT, sizeof(ESP_IDX)), esp_idx)
end

# type == PKT_C2U_DRONE_DISCONNECTED
struct Packet_C2U_Drone_Disconnected <: Packet_C2U
    pkt::Basic_Packet
    esp_idx::ESP_IDX
end

struct Receive_Drone_Disconnected
    esp_idx::ESP_IDX
end

function read_and_eval_payload(::Val{PKT_C2U_DRONE_DISCONNECTED}, sp_idx)
    esp_idx = Serialportlib.read_bytes_as_object(ESP_IDX, sp_idx=sp_idx)
    return Receive_Drone_Disconnected(esp_idx)
end
    
@enum Controll_State_Flags::UInt16 begin
    DRONE_ARMED = UInt16(1 << 0)
end

@kwdef struct Payload_Controll_Commands
    ctrl_state_flags::Controll_State_Flags = 0
    pitch::Int16 = 0
    yaw::Int16 = 0
    roll::Int16 = 0
    throttle::Int16 = 0
end
    
# type == PKT_U2C_CONTROLL_COMMANDS
struct Packet_U2C_Controll_Commands <: Packet_U2C
    pkt::Basic_Packet
    ctrl_cmds::Payload_Controll_Commands
    esp_idx::ESP_IDX
end

function Packet_U2C_Controll_Commands(ctrl_cmds::Payload_Controll_Commands, esp_idx)
    Packet_U2C_Connect(Basic_Packet(PKT_U2C_CONTROLL_COMMANDS, packedsize(Packet_U2C_Controll_Commands) - packedsize(Basic_Packet)),
                       ctrl_cmds,
                       esp_idx)
end

# type == PKT_U2C_GET_DETECTED_DRONES
struct Packet_U2C_Get_Detected_Drones <: Packet_U2C
    pkt::Basic_Packet
end

function Packet_U2C_Get_Detected_Drones()
    Packet_U2C_Get_Detected_Drones(Basic_Packet(PKT_U2C_GET_DETECTED_DRONES, 0))
end

# type == PKT_C2U_DETECTED_DRONES
struct Packet_C2U_Detected_Drones <: Packet_C2U
    pkt::Basic_Packet
    detected_drones_length::Int32
    
    # The indices of the detected drones follow right after the packet
    # In C you would write 'ESP_IDX detected_drones[0];' as last member of the struct
end

struct Receive_Detected_Drones{length}
    detected_drones::NTuple{length, ESP_IDX}
end

function read_and_eval_payload(::Val{PKT_C2U_DETECTED_DRONES}, sp_idx)
    detected_drones_length = Serialportlib.read_bytes_as_object_packed(Int32, sp_idx=sp_idx)
    detected_drones = Serialportlib.read_bytes_as_object_packed(NTuple{detected_drones_length, ESP_IDX}, sp_idx=sp_idx)
    return Receive_Detected_Drones{detected_drones_length}(detected_drones)
end    

struct Payload_Imu_Data
    time_us::Int64
    omega_x::Float32
    omega_y::Float32
    omega_z::Float32
    acc_x::Float32
    acc_y::Float32
    acc_z::Float32
end

# type == PKT_C2U_IMU_DATA
struct Packet_C2U_Imu_Data <: Packet_C2U
    pkt::Basic_Packet
    imu_data::Payload_Imu_Data
    esp_idx::ESP_IDX
end

struct Receive_Imu_Data
    esp_idx::ESP_IDX
    imu_data::Payload_Imu_Data
end

function read_and_eval_payload(::Val{PKT_C2U_IMU_DATA}, sp_idx)
    imu_data = Serialportlib.read_bytes_as_object_packed(Payload_Imu_Data, sp_idx=sp_idx)
    esp_idx = Serialportlib.read_bytes_as_object_packed(ESP_IDX, sp_idx=sp_idx)
    return Receive_Imu_Data(esp_idx, imu_data)
end

@enum Log_Msg_Type::UInt8 begin
    LOG_MSG_NONE = 0
    LOG_MSG_DBG_INFO = 1
    LOG_MSG_ERROR = 2
end

# type == PKT_C2U_LOG_MSG
struct Packet_C2U_Log_Msg <: Packet_C2U
    pkt::Basic_Packet
    log_msg_type::Log_Msg_Type
    msg_length::Int32
    
    # The bytes of the msg follow right after the packet
    # In C you would write 'char msg[0];' as last member of the struct
end

struct Receive_Log_Msg
    log_msg_type::Log_Msg_Type
    log_msg::String
end

function read_and_eval_payload(::Val{PKT_C2U_LOG_MSG}, pkt::Basic_Packet, sp_idx)
    log_msg_type = Serialportlib.read_bytes_as_object_packed(Log_Msg_Type, sp_idx=sp_idx)
    msg_length = Serialportlib.read_bytes_as_object_packed(Int32, sp_idx=sp_idx)
    log_msg_cstr = Serialportlib.read_bytes_as_object_packed(NTuple{msg_length, Cchar}, sp_idx=sp_idx)

    cstr_ref = Ref{NTuple{msg_length, Cchar}}(log_msg_cstr)
    
    log_msg = GC.@preserve cstr_ref begin
        cstr_ptr = Base.unsafe_convert(Ptr{NTuple{msg_length, Cchar}}, cstr_ref)
        unsafe_string(base.unsafe_convert(Ptr{Cchar}, cstr_ptr), msg_length)
    end
    
    return Receive_Log_Msg(log_msg_type, log_msg)
end

function open(portname::String; sp_idx=Serialportlib.DEFAULT_SP_IDX)::Bool
    
    Serialportlib.open(portname, 115200, sp_idx=sp_idx) || return false
    
    # Toggling RTS (Ready To Send) to signal to the esp32 that we are ready to start the serial-communication
    Dongle.Serialportlib.set_flag(Dongle.Serialportlib.RTS, false) || return false
    sleep(0.01) # 10 ms
    Dongle.Serialportlib.set_flag(Dongle.Serialportlib.RTS, true) || return false
    
    return true
end

function close(; sp_idx=Serialportlib.DEFAULT_SP_IDX)::Bool
    return Serialportlib.close(sp_idx=sp_idx)
end

function read_and_eval_packet(; sp_idx=Serialportlib.DEFAULT_SP_IDX)

    # First reads and validates the delimiter, then the next 'basic'-packet, and finally the payload
    # using the information about the payload size from the basic-packet.
    # 'read_and_eval_payload()' dispatches on the packet type and returns one of the 'Receive*' structs
    
    available_bytes = Serialportlib.bytes_available_to_read()
    # There should be at least this many bytes
    if (available_bytes < sizeof(SERIAL_DELIMITER) + packedsize(Basic_Packet))
        return nothing
    end

    delim = Serialportlib.read_bytes_as_object_packed(Serial_Delimiter, sp_idx=sp_idx)

    # If the delimiter is not there, try to skip to (behind) the next one
    if (delim != SERIAL_DELIMITER)
        @warn("Delimiter was not at the expected position after reading from the serial port, skipping to the next delimiter.")
        if !Serialportlib.skip_behind_next_delimiter(SERIAL_DELIMITER, sp_idx=sp_idx)
            @warn("Failed to skip behind the next delimiter.")
            return nothing
        end
    end
    
    basic_packet = Serialportlib.read_bytes_as_object_packed(Basic_Packet, sp_idx=sp_idx)

    available_bytes = Serialportlib.bytes_available_to_read()
    if available_bytes < basic_packet.payload_size
        @warn("Not enough bytes available to read the payload of the packet.")
        return nothing
    end

    # return basic_packet
    return read_and_eval_payload(Val(basic_packet.type), sp_idx)
end

function write_packet(pkt::PacketT; sp_idx=Serialportlib.DEFAULT_SP_IDX)::UInt64 where { PacketT <: Packet_U2C }
    
    bytes_written = Serialportlib.write_bytes_from_object_packed(pkt, sp_idx=sp_idx)
    if bytes_written < 0
        @error("Failed to write packet")
    else
        @assert(bytes_written == packedsize(PacketT))
    end
    
    return bytes_written
end

end # module Dongle
