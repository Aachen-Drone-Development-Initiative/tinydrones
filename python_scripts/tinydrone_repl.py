import serial, ctypes, logging

MAX_DRONE_NAME_LENGTH = 16

SERIAL_DELIMITER = b'tiny'

Esp_Idx = ctypes.c_int
INVALID_ESP_IDX = -1

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

class Dongle:
    def __init__(self, portname):
        # timeout=0 -> immediately return when there is no data
        self.serialport = serial.Serial(port=portname, timeout=0)

    def close(self):
        self.serialport.close()

    def read_packet(self):
        
        # First reads and validates the delimiter, then the next packet-header, and finally the payload
        # using the information about the payload size from the packet-header.

        header_size = ctypes.sizeof(Packet_Header)
        
        # There should be at least this many bytes, return as early as possible
        available_bytes = self.serialport.inWaiting()
        if available_bytes < len(SERIAL_DELIMITER) + header_size:
            return None

        delim = self.serialport.read(len(SERIAL_DELIMITER))

        if delim != SERIAL_DELIMITER:
            logging.warning("Delimiter was not at the expected position after reading from the serial-port, skipping behind the next delimiter.")
            if not self._skip_behind_next_delimiter():
                logging.warning("Failed to skip behind the next delimiter.")
                return None

        # Check available bytes again after potential byte skipping
        available_bytes = self.serialport.inWaiting()
        if available_bytes < header_size:
            return None

        header_bytes = bytearray(self.serialport.read(header_size))
        header = Packet_Header.from_buffer(header_bytes)

        # Checking header validity
        if header.payload_size < 0:
            logging.error(f"Detected negative payload_size of '{header.payload_size}'")
            return None

        Packet_T = map_packet_type_to_packet.get(header.type)
        if not Packet_T:
            logging.error(f"Detected unknow or invalid packet-type '{header.type}'")
            return None

        # TODO (yuzeni): At this point we should really block and wait for the bytes of the payload!!!!!!!!!!!!!!!
        
        # Do we have enough bytes for the payload? 
        available_bytes = self.serialport.inWaiting()
        if available_bytes < header.payload_size:
            logging.warning(f"Not enough bytes available to read the payload from the packet of type '{header.type}'")
            return None

        packet_bytes = header_bytes + bytearray(self.serialport.read(header.payload_size))
        assert len(packet_bytes) == header_size + header.payload_size
        
        packet = Packet_T.from_buffer(packet_bytes)
        packet_size = ctypes.sizeof(Packet_T)

        # Read and eval the dynamically sized part of the payload, if it exists
        if header_size + header.payload_size > packet_size:
            packet._eval_dynamic_part(packet_bytes[packet_size:])

        return packet
    
    def read_all_packets(self):
        packet = self.read_packet()
        packets = list()
        while packet is not None:
            packets.append(packet)
            packet = self.read_packet()
        return packets

    def write_packet(self, pkt) -> bool:
        if not isinstance(pkt, Packet_M2C):
            logging.error("The type of the provided packet is not a Packet_M2C subclass.")
            return False

        # Write the delimiter, then actual packet. Flush to send it immediately
        self.serialport.write(SERIAL_DELIMITER)
        bytes_written = self.serialport.write(bytes(pkt))
        self.serialport.flush()

        # Check if the write was successful
        if bytes_written != ctypes.sizeof(type(pkt)):
            logging.error(f"Only wrote '{bytes_written}' bytes, but sizeof(packet)={ctypes.sizeof(type(pkt))}")
            return False
        return True
    
    def _skip_behind_next_delimiter(self) -> bool:
        delim_len = len(SERIAL_DELIMITER)
        delim = bytearray(self.serialport.read(delim_len))
        if len(delim) != delim_len:
            return False
        
        while delim != SERIAL_DELIMITER:
            c = self.serialport.read(1)
            if c:
                delim[:-1] = delim[1:] # shift by one byte to the left
                delim[-1] = c[0]
            else:
                return False
        return True
