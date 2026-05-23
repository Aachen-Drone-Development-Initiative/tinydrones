import struct, logging, random
from types import SimpleNamespace

DRONE_NAME_LENGTH = 12
NAMED_VALUE_NAME_LENGTH = 10
SPKT_MAX_SIZE = 1024 * 4


SPKT_INVALID = 0

SPKT_M2C_CONNECT_TO_DRONE = 64
SPKT_M2C_DISCONNECT_FROM_DRONE = 65
SPKT_M2C_CONTROLL_COMMANDS = 66
SPKT_M2C_GET_DRONE_INFOS = 67
SPKT_M2C_PING_DRONE = 68
SPKT_M2C_RESTART_DRONE = 69
SPKT_M2C_SET_DRONE_PARAMETERS = 70
SPKT_M2C_DRONE_POSE_ESTIMATE = 71

SPKT_C2M_SIMPLE_RESPONSE = 96
SPKT_C2M_DRONE_INFOS = 97
SPKT_C2M_DRONE_LOG_VALUES = 98
SPKT_C2M_MSG = 100


DTYPE_INT32 = 0
DTYPE_INT64 = 1
DTYPE_FLOAT32 = 2


SERIAL_DELIMITER = (0xB62A429F).to_bytes(4, byteorder='little')

SPACKET_TYPE_FORMAT     = 'B' # uint8_t
SERIAL_DELIMITER_FORMAT = 'I' # uint32_t
SPACKET_ID_FORMAT       = 'I' # uint32_t

HEADER_FORMAT = '<' + SERIAL_DELIMITER_FORMAT + SPACKET_TYPE_FORMAT + 'h' + SPACKET_ID_FORMAT
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)


# M2C - Master to Controller


def generate_packet_id() -> int:
    packet_id = 0
    while packet_id == 0:
        packet_id = random.getrandbits(32)
    return packet_id


def pack_header(pkt_type: int, payload_size: int) -> bytes:
    return struct.pack(HEADER_FORMAT,
                       int.from_bytes(SERIAL_DELIMITER, byteorder='little'),
                       pkt_type,
                       payload_size,
                       generate_packet_id())


def pack_m2c_connect_to_drone(drone_name: str) -> bytes:
    payload_format = '<' + str(DRONE_NAME_LENGTH) + 's'
    header = pack_header(SPKT_M2C_CONNECT_TO_DRONE, struct.calcsize(payload_format))
    payload = struct.pack(payload_format, drone_name.encode('ascii'))
    return header + payload


def pack_m2c_disconnect_from_drone(drone_name: str) -> bytes:
    payload_format = '<' + str(DRONE_NAME_LENGTH) + 's'
    header = pack_header(SPKT_M2C_DISCONNECT_FROM_DRONE, struct.calcsize(payload_format))
    payload = struct.pack(payload_format, drone_name.encode('ascii'))
    return header + payload


CTRL_CMD_FLAG_IS_ARMED = 1 << 0

def pack_m2c_control_commands(drone_name: str, pitch: int, yaw: int, roll: int, thrust: int, flags: int) -> bytes:
    payload_format = '<hhhhB' + str(DRONE_NAME_LENGTH) + 's'
    header = pack_header(SPKT_M2C_CONTROLL_COMMANDS, struct.calcsize(payload_format))
    payload = struct.pack(payload_format, pitch, yaw, roll, thrust, flags, drone_name.encode('ascii'))
    return header + payload


def pack_m2c_set_drone_parameters(drone_name: str, parameters: list[tuple[str, int, any]]) -> bytes:
    payload_format_fixed_part = '<' + str(DRONE_NAME_LENGTH) + 'sh'
    payload = struct.pack(payload_format_fixed_part, drone_name.encode('ascii'), len(parameters))
    
    payload_named_value_format = '<B' + str(NAMED_VALUE_NAME_LENGTH) + 'sQ'
    for param in parameters:
        name, dtype, value = param

        if dtype == DTYPE_INT32:
            data_bytes = struct.pack('<i', int(value)) + bytes(4)
        elif dtype == DTYPE_INT64:
            data_bytes = struct.pack('<Q', int(value))
        elif dtype == DTYPE_FLOAT32:
            data_bytes = struct.pack('<f', float(value)) + bytes(4)
        else:
            raise TypeError(f"Unsupported dtype: {dtype}")

        data_u64 = struct.unpack('<Q', data_bytes)[0]
        payload_named_value = struct.pack(payload_named_value_format, dtype, name.encode('ascii'), data_u64)
        payload += payload_named_value
        
    header = pack_header(SPKT_M2C_SET_DRONE_PARAMETERS, len(payload))
    return header + payload


def pack_m2c_get_drone_infos() -> bytes:
    return pack_header(SPKT_M2C_GET_DRONE_INFOS, 0)


def pack_m2c_ping_drone(drone_name: str) -> bytes:
    payload_format = '<' + str(DRONE_NAME_LENGTH) + 's'
    header = pack_header(SPKT_M2C_PING_DRONE, struct.calcsize(payload_format))
    payload = struct.pack(payload_format, drone_name.encode('ascii'))
    return header + payload


def pack_m2c_restart_drone(drone_name: str) -> bytes:
    payload_format = '<' + str(DRONE_NAME_LENGTH) + 's'
    header = pack_header(SPKT_M2C_DRONE_POSE_ESTIMATE, struct.calcsize(payload_format))
    payload = struct.pack(payload_format, drone_name.encode('ascii'))
    return header + payload


def pack_m2c_pose_estimate(drone_name: str, x: float, y: float, z: float, pitch: float, yaw: float, roll: float, time_us: int) -> bytes:
    payload_format = '<ffffffq' + str(DRONE_NAME_LENGTH) + 's'
    header = pack_header(SPKT_M2C_DRONE_POSE_ESTIMATE, struct.calcsize(payload_format))
    payload = struct.pack(payload_format, x, y, z, pitch, yaw, roll, time_us, drone_name.encode('ascii'))
    return header + payload


# C2M - Controller to Master


RESPONSE_SUCCESS = 0
RESPONSE_FAIL = 1

def unpack_c2m_simple_response(data: bytes) -> SimpleNamespace:
    header = unpack_header(data[:HEADER_SIZE])
    payload_format = '<BI' + SPACKET_TYPE_FORMAT
    response, request_pkt_id, request_pkt_type = struct.unpack(payload_format, data[HEADER_SIZE:])
    return SimpleNamespace(
        pkt_type      = SPKT_C2M_SIMPLE_RESPONSE,
        packet_id     = header.packet_id,
        response      = response,
        request_pkt_id    = request_pkt_id,
        request_pkt_type  = request_pkt_type,
    )


def unpack_c2m_drone_log_values(data: bytes) -> SimpleNamespace:
    header = unpack_header(data[:HEADER_SIZE])
    payload_format = '<' + str(DRONE_NAME_LENGTH) + 'sh'
    drone_name, log_value_count = struct.unpack(payload_format, data[HEADER_SIZE:HEADER_SIZE + struct.calcsize(payload_format)])
    named_value_size = struct.calcsize(f'<B{NAMED_VALUE_NAME_LENGTH}sQ')
    offset = HEADER_SIZE + struct.calcsize(payload_format)
    log_values = SimpleNamespace()

    for _ in range(log_value_count):
        dtype, name_bytes, data_u64 = struct.unpack(f'<B{NAMED_VALUE_NAME_LENGTH}sQ', data[offset : offset + named_value_size])
        data_bytes = struct.pack('<Q', data_u64)

        if dtype == DTYPE_INT32:
            value = struct.unpack('<i', data_bytes[:4])[0]
        elif dtype == DTYPE_FLOAT32:
            value = struct.unpack('<f', data_bytes[:4])[0]
        elif dtype == DTYPE_INT64:
            value = struct.unpack('<q', data_bytes)[0]
        else:
            value = data_bytes

        setattr(log_values, name_bytes.decode('ascii').rstrip('\0'), value)
        offset += named_value_size

    return SimpleNamespace(
        pkt_type=SPKT_C2M_DRONE_LOG_VALUES,
        packet_id=header.packet_id,
        drone_name=drone_name.decode('ascii').rstrip('\0'),
        log_values=log_values,
    )


DRONE_INFO_FLAG_IS_CONNETED      = 1 << 0
DRONE_INFO_FLAG_IS_RESPONDING    = 1 << 1

def unpack_c2m_drone_infos(data: bytes) -> SimpleNamespace:
    header = unpack_header(data[:HEADER_SIZE])
    drone_infos_count_format = '<h'
    drone_infos_count_size = struct.calcsize(drone_infos_count_format)
    (drone_infos_count,) = struct.unpack(drone_infos_count_format,
        data[HEADER_SIZE:HEADER_SIZE + drone_infos_count_size])

    drone_info_format = '<B' + str(DRONE_NAME_LENGTH) + 's'
    drone_info_size = struct.calcsize(drone_info_format)
    drone_infos = []
    offset = HEADER_SIZE + drone_infos_count_size
    for _ in range(drone_infos_count):
        flags, drone_name = struct.unpack(drone_info_format,
                                          data[offset : offset + drone_info_size])
        drone_infos.append(SimpleNamespace(
            flags      = flags,
            drone_name = drone_name.decode('ascii').rstrip('\0')
        ))
        offset += drone_info_size
    
    return SimpleNamespace(
        pkt_type    = SPKT_C2M_DRONE_INFOS,
        packet_id   = header.packet_id,
        drone_infos = drone_infos
    )


def unpack_c2m_log(data: bytes) -> SimpleNamespace:
    header = unpack_header(data[:HEADER_SIZE])
    return SimpleNamespace(
        pkt_type = SPKT_C2M_MSG,
        packet_id = header.packet_id,
        msg      = data[HEADER_SIZE : HEADER_SIZE + header.payload_size].decode('ascii')
    )


def unpack_header(data: bytes) -> SimpleNamespace:
    delim, pkt_type, payload_size, packet_id = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])
    return SimpleNamespace(
        delim        = delim,
        pkt_type     = pkt_type,
        payload_size = payload_size,
        packet_id    = packet_id,
    )

class CorruptPacketError(Exception):
    pass

def unpack_any(data: bytes) -> SimpleNamespace:
    handlers = {
        SPKT_C2M_SIMPLE_RESPONSE: unpack_c2m_simple_response,
        SPKT_C2M_DRONE_INFOS: unpack_c2m_drone_infos,
        SPKT_C2M_DRONE_LOG_VALUES: unpack_c2m_drone_log_values,
        SPKT_C2M_MSG: unpack_c2m_log,
    }
    _, pkt_type, _, _ = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])
    handler = handlers.get(pkt_type)
    
    if not handler:
        raise CorruptPacketError(f"No unpack function exists for the packet type '{pkt_type}', it may be corrupted")
    
    return handler(data)
