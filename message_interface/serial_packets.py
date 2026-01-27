import ctypes

# Packet Types

PKT_INVALID = 0

# M2C - Master to Controller
PKT_M2C_CONNECT_TO_DRONE = 1
PKT_M2C_DISCONNECT_FROM_DRONE = 2
PKT_M2C_CONTROLL_COMMANDS = 3
PKT_M2C_GET_DETECTED_DRONES = 4

# C2M - Controller to Master
PKT_C2M_DRONE_CONNECTED = 5
PKT_C2M_DRONE_DISCONNECTED = 6
PKT_C2M_DETECTED_DRONES = 7
PKT_C2M_IMU_DATA = 8
PKT_C2M_LOG_MSG = 9

Packet_Type = ctypes.c_uint8

# Packet layout definitions

class Packet(ctypes.LittleEndianStructure):
    _pack_ = 1
    def _eval_dynamic_part(self, bytes_buffer):
        logging.error("This packet is not dynamically sized, yet the payload size exceeds the size of the packet.")
        return None

class Packet_M2C(Packet):
    pass

class Packet_C2M(Packet):
    pass

class Payload(ctypes.LittleEndianStructure):
    _pack_ = 1

class Packet_Header(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [("type", Packet_Type),
                ("payload_size", ctypes.c_int16)]
    # The payload follows right after these fields in memory
    # In C you would write 'uint8_t payload[0];' as last member of the struct

# M2C - Master to Controller
    
# type == PKT_M2C_CONNECT_TO_DRONE
class Packet_M2C_Connect_To_Drone(Packet_M2C):
    _fields_ = [("header", Packet_Header),
                ("esp_idx", Esp_Idx)]

# type == PKT_M2C_DISCONNECT_FROM_DRONE
class Packet_M2C_Disconnect_From_Drone(Packet_M2C):
    _fields_ = [("header", Packet_Header),
                ("esp_idx", Esp_Idx)]

CTRL_STATE_DRONE_ARMED = (1 << 0)

Controll_State_Flags = ctypes.c_uint16

class Payload_Controll_Commands(Payload):
    _fields_ = [("ctrl_state_flags", Controll_State_Flags),
                ("pitch",    ctypes.c_int16),
                ("yaw",      ctypes.c_int16),
                ("roll",     ctypes.c_int16),
                ("throttle", ctypes.c_int16)]

# type == PKT_M2C_CONTROLL_COMMANDS
class Packet_M2C_Controll_Commands(Packet_M2C):
    _fields_ = [("header", Packet_Header),
                ("ctrl_cmds", Payload_Controll_Commands),
                ("esp_idx", Esp_Idx)]

# type == PKT_M2C_GET_DETECTED_DRONES
class Packet_M2C_Get_Detected_Drones(Packet_M2C):
    _fields_ = [("header", Packet_Header)]

# C2M - Controller to Master

# type == PKT_C2M_DRONE_CONNECTED
class Packet_C2M_Drone_Connected(Packet_C2M):
    _fields_ = [("header", Packet_Header),
                ("esp_idx", Esp_Idx)]

# type == PKT_C2M_DRONE_DISCONNECTED
class Packet_C2M_Drone_Disconnected(Packet_C2M):
    _fields_ = [("header", Packet_Header),
                ("esp_idx", Esp_Idx)]

class Payload_Detected_Drone(Payload):
    _fields_ = [("esp_idx", Esp_Idx),
                ("drone_name", ctypes.c_char * MAX_DRONE_NAME_LENGTH)]

# type == PKT_C2M_DETECTED_DRONES
class Packet_C2M_Detected_Drones(Packet_C2M):
    _fields_ = [("header", Packet_Header),
                ("detected_drones_length", ctypes.c_int)]
    
    # The indices of the detected drones follow right after the packet
    # In C you would write 'Esp_Idx detected_drones[0];' as last member of the struct

    def _eval_dynamic_part(self, bytes_buffer):
        detected_drones_size = self.detected_drones_length * ctypes.sizeof(Payload_Detected_Drone)
        if detected_drones_size != len(bytes_buffer):
            logging.error("Size mismatch for PKT_C2M_DETECTED_DRONES")
            return None
        detected_drones = (Payload_Detected_Drone * self.detected_drones_length).from_buffer(bytes_buffer)
        self.detected_drones = list(detected_drones) # attach a list of indices

class Payload_Imu_Data(Payload):
    _fields_ = [("time_us", ctypes.c_int64),
                ("omega_x", ctypes.c_float),
                ("omega_y", ctypes.c_float),
                ("omega_z", ctypes.c_float),
                ("acc_x", ctypes.c_float),
                ("acc_y", ctypes.c_float),
                ("acc_z", ctypes.c_float)]

# type == PKT_C2M_IMU_DATA
class Packet_C2M_Imu_Data(Packet_C2M):
    _fields_ = [("header", Packet_Header),
                ("imu_data", Payload_Imu_Data),
                ("esp_idx", Esp_Idx)]

LOG_MSG_NONE = 0
LOG_MSG_DBG_INFO = 1
LOG_MSG_ERROR = 2

Log_Msg_Type = ctypes.c_uint8

class Payload_Log(Payload):
    _fields_ = [("type", Log_Msg_Type),
                ("msg_length", ctypes.c_int)]

# type == PKT_C2M_LOG_MSG
class Packet_C2M_Log_Msg(Packet_C2M):
    _fields_ = [("header", Packet_Header),
                ("esp_idx", Esp_Idx),
                ("log", Payload_Log)]

    # The bytes of the msg follow right after the packet
    # In C you would write 'char msg[0];' as last member of the struct

    def _eval_dynamic_part(self, bytes_buffer):
        if self.msg_length != len(bytes_buffer):
            logging.error("Size mismatch for PKT_C2M_LOG_MSG")
            return None
        msg = (ctypes.c_char * self.msg_length).from_buffer(bytes_buffer)
        self.msg = msg.raw.decode("ascii")

map_packet_type_to_packet = {
    # M2C - Master to Controller
    PKT_M2C_CONNECT_TO_DRONE: Packet_M2C_Connect_To_Drone,
    PKT_M2C_DISCONNECT_FROM_DRONE: Packet_M2C_Disconnect_From_Drone,
    PKT_M2C_CONTROLL_COMMANDS: Packet_M2C_Controll_Commands,
    PKT_M2C_GET_DETECTED_DRONES: Packet_M2C_Get_Detected_Drones,
    PKT_C2M_DRONE_CONNECTED: Packet_C2M_Drone_Connected,
    # C2M - Controller to Master
    PKT_C2M_DRONE_DISCONNECTED: Packet_C2M_Drone_Disconnected,
    PKT_C2M_DETECTED_DRONES: Packet_C2M_Detected_Drones,
    PKT_C2M_IMU_DATA: Packet_C2M_Imu_Data,
    PKT_C2M_LOG_MSG: Packet_C2M_Log_Msg,
}
