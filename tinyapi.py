import serial, time, sys
from message_interface.serial_packets import *

serialport = None
all_packets = [] # A list of all the packets received so far

def close_port():
    if serialport != None:
        serialport.close()


def set_port(portname):
    close_port()
    global serialport
    # 2 Second timeout seems reasonable
    serialport = serial.Serial(port=portname, timeout=2)


def read_packet():
    # First reads and validates the delimiter, then the next packet-header, and finally the payload
    # using the information about the payload size from the packet-header.
        
    delim_size = len(SERIAL_DELIMITER)
    delim = serialport.read(delim_size)

    if delim != SERIAL_DELIMITER:
        print("WARNING: Delimiter was not at the expected position after reading"
              "from the serial-port, skipping behind the next delimiter.")
        # Skipping to the next delimiter or returning None
        delim = serialport.read(delim_size)
        if len(delim) != delim_size:
            raise TimeoutError("Reading from the serialport timed out...")
        while delim != SERIAL_DELIMITER:
            c = serialport.read(1)
            if len(c) != 1:
                raise TimeoutError("Reading from the serialport timed out...")
            delim = delim[1:] + c
        
    header_bytes = delim + serialport.read(HEADER_SIZE - delim_size)
    if len(header_bytes) != HEADER_SIZE:
        raise TimeoutError("Reading from the serialport timed out...")
    
    header = unpack_header(header_bytes)

    # Header sanity check
    if header.payload_size < 0:
        raise CorruptPacketError(f"Payload size is negative: {header.payload_size}")
    if header.payload_size > SPKT_MAX_SIZE - HEADER_SIZE:
        raise CorruptPacketError(f"Payload size is larger than the maximum size: {header.payload_size}")
        
    packet_bytes = header_bytes + serialport.read(header.payload_size)
    if len(packet_bytes) != HEADER_SIZE + header.payload_size:
        raise TimeoutError("Reading from the serialport timed out...")

    packet = unpack_any(packet_bytes)
    all_packets.append(packet)

    # Always print error messages from the controller
    if packet.pkt_type == SPKT_C2M_LOG:
        if packet.msg.startswith("ERROR"):
            print(packet.msg.rstrip("\n\0"))

    return packet

    
def read_all_packets():
    packets = []
    while serialport.inWaiting() > 0:
        try:
            packets.append(read_packet())
        except TimeoutError:
            pass # the loop condition handles this
        except CorruptPacketError:
            pass
            
    return packets

    
def expect_specific_packet(pkt_type: int, timeout_s: float):
    packet = read_packet()
    time_begin_s = time.time()
    while packet.pkt_type != pkt_type:
        if (time.time() - time_begin_s > timeout_s):
            raise TimeoutError(f"Waiting for the packet of type '{pkt_type}' timed out...")
        packet = read_packet()
        
    return packet


def write_packet(data: bytes):
    bytes_written = serialport.write(data)
    serialport.flush()
    if bytes_written != len(data):
        raise RuntimeError(f"Failed to write all bytes: {bytes_written} of {len(data)}")


def list_drones():
    read_all_packets() # exhaust the serialport receive buffer
    write_packet(pack_m2c_get_drone_infos())
    packet = expect_specific_packet(SPKT_C2M_DRONE_INFOS, 3)
    for drone_info in packet.drone_infos:
        is_connected  = drone_info.flags & DRONE_INFO_FLAG_IS_CONNETED
        is_responding = drone_info.flags & DRONE_INFO_FLAG_IS_RESPONDING
        fstr = f"{drone_info.drone_name} -- {'🟢 connected' if is_connected else '🔴 disconnected'}"
        if is_connected:
            fstr += f", {'⚡ responding' if is_responding else '❌ not responding'}"
        print(fstr)


def connect(drone_name: str):
    read_all_packets()
    write_packet(pack_m2c_connect_to_drone(drone_name))
    packet = expect_specific_packet(SPKT_C2M_DRONE_CONNECTION_UPDATE, 3)
    if packet.update == DRONE_CONN_UPDATE_SUCCESSFULLY_CONNECTED:
        print(f"Connected to drone '{packet.drone_name}'")
    elif packet.update == DRONE_CONN_UPDATE_FAILED_TO_CONNECT:
        print(f"Failed to connect to drone '{drone_name}'")
    else:
        raise CorruptPacketError(f"Unexpected update '{packet.update}'")


def disconnect(drone_name: str):
    read_all_packets()
    write_packet(pack_m2c_disconnect_from_drone(drone_name))
    packet = expect_specific_packet(SPKT_C2M_DRONE_CONNECTION_UPDATE, 3)
    if packet.update == DRONE_CONN_UPDATE_SUCCESSFULLY_DISCONNECTED:
        print(f"Disconnected from drone '{packet.drone_name}'")
    elif packet.update == DRONE_CONN_UPDATE_FAILED_TO_DISCONNECT:
        print(f"Failed to disconnect from drone '{drone_name}'")
    else:
        raise CorruptPacketError(f"Unexpected update '{packet.update}'")


def arm(drone_name: str):
    write_packet(pack_m2c_control_commands(drone_name,
                                           pitch=0, yaw=0, roll=0, throttle=0,
                                           flags=CTRL_CMD_FLAG_IS_ARMED))


def disarm(drone_name: str):
    write_packet(pack_m2c_control_commands(drone_name,
                                           pitch=0, yaw=0, roll=0, throttle=0,
                                           flags=0))


def ping(drone_name: str):
    write_packet(pack_m2c_ping_drone(drone_name))
    packet = expect_specific_packet(SPKT_C2M_PING_DRONE_RESULT, 3)
    if packet.drone_responded == True:
        print(f"Drone '{packet.drone_name}' responded")
    else:
        print(f"Drone '{packet.drone_name}' did not respond")


def range_test(drone_name: str):
    try:
        while True:
            write_packet(pack_m2c_ping_drone(drone_name))
            responded = False
            try:
                packet = expect_specific_packet(SPKT_C2M_PING_DRONE_RESULT, 1)
                responded = packet.drone_responded
            except TimeoutError:
                responded = False
            sys.stdout.write("\r\033[K")
            sys.stdout.write(f"{'responded :-)' if responded else 'no response :-('}")
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("stop")
        return


def restart(drone_name: str):
    write_packet(pack_m2c_restart_drone(drone_name))


def monitor():
    try:
        while True:
            if serialport.inWaiting() > 0:
                packet = read_packet()
                if packet.pkt_type == SPKT_C2M_LOG:
                    print(packet.msg.rstrip("\n\0"))
                else:
                    print(packet)
            else:
                time.sleep(0.01) # don't busy loop
    except KeyboardInterrupt:
        print("exit")
        return
