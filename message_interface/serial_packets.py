import struct, logging
from types import SimpleNamespace

MAX_DRONE_NAME_LENGTH = 12
SPKT_MAX_SIZE = 1024 * 4


SPKT_INVALID = 0

SPKT_M2C_CONNECT_TO_DRONE = 64
SPKT_M2C_DISCONNECT_FROM_DRONE = 65
SPKT_M2C_CONTROLL_COMMANDS = 66
SPKT_M2C_GET_DRONE_INFOS = 67
SPKT_M2C_PING_DRONE = 68
SPKT_M2C_RESTART_DRONE = 69
SPKT_M2C_DRONE_SETTINGS = 70

SPKT_C2M_DRONE_CONNECTION_UPDATE = 96
SPKT_C2M_DRONE_INFOS = 97
SPKT_C2M_DRONE_IMU_DATA = 98
SPKT_C2M_DRONE_TELEMETRY = 99
SPKT_C2M_LOG = 100
SPKT_C2M_PING_DRONE_RESULT = 101


SERIAL_DELIMITER = (0xB62A429F).to_bytes(4, byteorder='little')

SPACKET_TYPE_FORMAT     = 'B' # uint8_t
SERIAL_DELIMITER_FORMAT = 'I' # uint32_t

HEADER_FORMAT = '<' + SERIAL_DELIMITER_FORMAT + SPACKET_TYPE_FORMAT + 'h'
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)


# M2C - Master to Controller


def pack_header(pkt_type: int, payload_size: int) -> bytes:
    return struct.pack(HEADER_FORMAT, int.from_bytes(SERIAL_DELIMITER, byteorder='little'), pkt_type, payload_size)


def pack_m2c_connect_to_drone(drone_name: str) -> bytes:
    payload_format = '<' + str(MAX_DRONE_NAME_LENGTH) + 's'
    header = pack_header(SPKT_M2C_CONNECT_TO_DRONE, struct.calcsize(payload_format))
    payload = struct.pack(payload_format, drone_name.encode('ascii'))
    return header + payload


def pack_m2c_disconnect_from_drone(drone_name: str) -> bytes:
    payload_format = '<' + str(MAX_DRONE_NAME_LENGTH) + 's'
    header = pack_header(SPKT_M2C_DISCONNECT_FROM_DRONE, struct.calcsize(payload_format))
    payload = struct.pack(payload_format, drone_name.encode('ascii'))
    return header + payload


CTRL_CMD_FLAG_IS_ARMED = 1 << 0

def pack_m2c_control_commands(drone_name: str, pitch: int, yaw: int, roll: int, throttle: int, flags: int) -> bytes:
    payload_format = '<hhhhB' + str(MAX_DRONE_NAME_LENGTH) + 's'
    header = pack_header(SPKT_M2C_CONTROLL_COMMANDS, struct.calcsize(payload_format))
    payload = struct.pack(payload_format, pitch, yaw, roll, throttle, flags, drone_name.encode('ascii'))
    return header + payload


def pack_m2c_drone_settings(drone_name: str,
                            roll_p: float, roll_i: float, roll_d: float,
                            pitch_p: float, pitch_i: float, pitch_d: float,
                            yaw_p: float, yaw_i: float, yaw_d: float) -> bytes:
    payload_format = '<fffffffff' + str(MAX_DRONE_NAME_LENGTH) + 's'
    header = pack_header(SPKT_M2C_DRONE_SETTINGS, struct.calcsize(payload_format))
    payload = struct.pack(payload_format,
                          roll_p, roll_i, roll_d,
                          pitch_p, pitch_i, pitch_d,
                          yaw_p, yaw_i, yaw_d,
                          drone_name.encode('ascii'))
    return header + payload


def pack_m2c_get_drone_infos() -> bytes:
    return pack_header(SPKT_M2C_GET_DRONE_INFOS, 0)


def pack_m2c_ping_drone(drone_name: str) -> bytes:
    payload_format = '<' + str(MAX_DRONE_NAME_LENGTH) + 's'
    header = pack_header(SPKT_M2C_PING_DRONE, struct.calcsize(payload_format))
    payload = struct.pack(payload_format, drone_name.encode('ascii'))
    return header + payload


def pack_m2c_restart_drone(drone_name: str) -> bytes:
    payload_format = '<' + str(MAX_DRONE_NAME_LENGTH) + 's'
    header = pack_header(SPKT_M2C_RESTART_DRONE, struct.calcsize(payload_format))
    payload = struct.pack(payload_format, drone_name.encode('ascii'))
    return header + payload


# C2M - Controller to Master


DRONE_CONN_UPDATE_SUCCESSFULLY_CONNECTED      = 0
DRONE_CONN_UPDATE_FAILED_TO_CONNECT           = 1
DRONE_CONN_UPDATE_SUCCESSFULLY_DISCONNECTED   = 2
DRONE_CONN_UPDATE_FAILED_TO_DISCONNECT        = 3

def unpack_c2m_drone_connection_update(data: bytes) -> SimpleNamespace:
    payload_format = '<B' + str(MAX_DRONE_NAME_LENGTH) + 's'
    update, drone_name = struct.unpack(payload_format, data[HEADER_SIZE:])
    return SimpleNamespace(
        pkt_type   = SPKT_C2M_DRONE_CONNECTION_UPDATE,
        update     = update,
        drone_name = drone_name.decode("ascii").rstrip('\0')
    )


def unpack_c2m_drone_imu_data(data: bytes) -> SimpleNamespace:
    payload_format = '<qffffff' + str(MAX_DRONE_NAME_LENGTH) + 's'
    time_us, omega_x, omega_y, omega_z, acc_x, acc_y, acc_z, drone_name = struct.unpack(
        payload_format, data[HEADER_SIZE:])
    drone_name = drone_name.decode('ascii').rstrip('\0')
    return SimpleNamespace(
        pkt_type = SPKT_C2M_DRONE_IMU_DATA,
        time_us = time_us,
        omega_x = omega_x,
        omega_y = omega_y,
        omega_z = omega_z,
        acc_x = acc_x,
        acc_y = acc_y,
        acc_z = acc_z,
        drone_name = drone_name
    )


def unpack_c2m_drone_telemetry(data: bytes) -> SimpleNamespace:
    payload_format = '<qf' + str(MAX_DRONE_NAME_LENGTH) + 's'
    time_us, battery_voltage, drone_name = struct.unpack(
        payload_format, data[HEADER_SIZE:])
    return SimpleNamespace(
        pkt_type        = SPKT_C2M_DRONE_TELEMETRY,
        time_us         = time_us,
        battery_voltage = battery_voltage,
        drone_name      = drone_name.decode('ascii').rstrip('\0')
    )


DRONE_INFO_FLAG_IS_CONNETED      = 1 << 0
DRONE_INFO_FLAG_IS_RESPONDING    = 1 << 1

def unpack_c2m_drone_infos(data: bytes) -> SimpleNamespace:
    drone_infos_count_format = '<h'
    drone_infos_count_size = struct.calcsize(drone_infos_count_format)
    (drone_infos_count,) = struct.unpack(drone_infos_count_format,
        data[HEADER_SIZE:HEADER_SIZE + drone_infos_count_size])

    drone_info_format = '<B' + str(MAX_DRONE_NAME_LENGTH) + 's'
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
        drone_infos = drone_infos
    )


def unpack_c2m_log(data: bytes) -> SimpleNamespace:
    _, _, payload_size = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])
    return SimpleNamespace(
        pkt_type = SPKT_C2M_LOG,
        msg      = data[HEADER_SIZE : HEADER_SIZE + payload_size].decode('ascii')
    )


def unpack_c2m_ping_drone_result(data: bytes) -> SimpleNamespace:
    payload_format = '<B' + str(MAX_DRONE_NAME_LENGTH) + 's'
    drone_responded, drone_name = struct.unpack(payload_format, data[HEADER_SIZE:])
    return SimpleNamespace(
        pkt_type        = SPKT_C2M_PING_DRONE_RESULT,
        drone_responded = drone_responded,
        drone_name      = drone_name.decode("ascii").rstrip('\0')
    )


def unpack_header(data: bytes) -> SimpleNamespace:
    delim, pkt_type, payload_size = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])
    return SimpleNamespace(
        delim        = delim,
        pkt_type     = pkt_type,
        payload_size = payload_size
    )

class CorruptPacketError(Exception):
    pass

def unpack_any(data: bytes) -> SimpleNamespace:
    handlers = {
        SPKT_C2M_DRONE_CONNECTION_UPDATE:    unpack_c2m_drone_connection_update,
        SPKT_C2M_DRONE_INFOS:                unpack_c2m_drone_infos,
        SPKT_C2M_DRONE_IMU_DATA:             unpack_c2m_drone_imu_data,
        SPKT_C2M_DRONE_TELEMETRY:            unpack_c2m_drone_telemetry,
        SPKT_C2M_LOG:                        unpack_c2m_log,
        SPKT_C2M_PING_DRONE_RESULT:          unpack_c2m_ping_drone_result,
    }
    _, pkt_type, _ = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])
    handler = handlers.get(pkt_type)
    
    if not handler:
        raise CorruptPacketError(f"No unpack function exists for the packet type '{pkt_type}', it may be corrupted")
    
    return handler(data)
