import serial, time, sys
from message_interface.serial_packets import *

serialport = None
all_packets = [] # A list of all the packets received so far


def open_port(portname):
    """opens a port as serialport, a remote/dongle should be connected to it"""
    global serialport
    if serialport != None:
        serialport.close()
    serialport = serial.Serial(port=portname, timeout=2, baudrate=SERIAL_BAUD_RATE) # 2 Second timeout seems reasonable


def close_port():
    """closes the port"""
    if serialport != None:
        serialport.close()


def read_packet():
    """Reads the next packet from the controller, raises TimeoutError if there are not enough bytes.
    If a log packet is received and it starts with `'ERROR'`, it will be printed immediately.
    The packet will always be appended to the `all_packets` array."""
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
    if header.packet_id == 0:
        raise CorruptPacketError("Packet id is 0")
        
    packet_bytes = header_bytes + serialport.read(header.payload_size)
    if len(packet_bytes) != HEADER_SIZE + header.payload_size:
        raise TimeoutError("Reading from the serialport timed out...")

    packet = unpack_any(packet_bytes)
    all_packets.append(packet)

    # Always print error messages from the controller
    if packet.pkt_type == SPKT_C2M_MSG:
        if packet.msg.startswith("ERROR") or packet.msg.startswith("INFO"):
            print(packet.msg.rstrip("\n\0"))

    return packet

    
def read_all_packets():
    """Reads packets until the serialport runs out of bytes."""
    packets = []
    while serialport.inWaiting() > 0:
        try:
            packets.append(read_packet())
        except TimeoutError:
            pass # the loop condition handles this
        except CorruptPacketError:
            pass
            
    return packets

    
def expect_packet_with_type(pkt_type: int, timeout_s: float):
    """Reads packets until a specific packet is read, or the timeout `timeout_s` is reached"""
    time_begin_s = time.time()
    while True:
        packet = read_packet()
        if packet.pkt_type == pkt_type:
            return packet
        if (time.time() - time_begin_s > timeout_s):
            raise TimeoutError(f"Waiting for the packet of type '{pkt_type}' timed out...")


def expect_simple_response(packet_id: int, timeout_s: float):
    """Reads packets until a specific packet is read, or the timeout `timeout_s` is reached"""
    time_begin_s = time.time()
    while True:
        packet = read_packet()
        if packet.pkt_type == SPKT_C2M_SIMPLE_RESPONSE and packet.request_pkt_id == packet_id:
            return packet
        if (time.time() - time_begin_s > timeout_s):
            raise TimeoutError(f"Waiting for a response for the packet with id '{packet_id}' timed out...")
        

def get_packet_id(data: bytes) -> int:
    header = unpack_header(data[:HEADER_SIZE])
    return header.packet_id

        
def write_packet(data: bytes):
    """Sends a packet to the controller. Use 'pack_m2c_*' functions from 'serial_packets.py'
    to create a bytes object representing the specific packet."""
    bytes_written = serialport.write(data)
    serialport.flush()
    if bytes_written != len(data):
        raise RuntimeError(f"Failed to write all bytes: {bytes_written} of {len(data)}")
    

def list_drones():
    """Prints a list of all drones that were detected by the controller and
    whether they are connected and responding."""
    read_all_packets() # exhaust the serialport receive buffer
    write_packet(pack_m2c_get_drone_infos())
    packet = expect_packet_with_type(SPKT_C2M_DRONE_INFOS, 3)
    for drone_info in packet.drone_infos:
        is_connected  = drone_info.flags & DRONE_INFO_FLAG_IS_CONNETED
        is_responding = drone_info.flags & DRONE_INFO_FLAG_IS_RESPONDING
        fstr = f"{drone_info.drone_name} -- {'🟢 connected' if is_connected else '🔴 disconnected'}"
        if is_connected:
            fstr += f", {'⚡ responding' if is_responding else '❌ not responding'}"
        print(fstr)


def connect(drone_name: str):
    """Ask the controller to connect to a specific drone."""
    read_all_packets()
    m2c_packet = pack_m2c_connect_to_drone(drone_name)
    write_packet(m2c_packet)
    c2m_response = expect_simple_response(get_packet_id(m2c_packet), 3)
    if c2m_response.response == RESPONSE_SUCCESS:
        print(f"Connected to drone '{drone_name}'")
    elif c2m_response.response == RESPONSE_FAIL:
        print(f"Failed to connect to drone '{drone_name}'")
    else:
        raise CorruptPacketError(f"Unexpected response '{c2m_response.response}'")


def disconnect(drone_name: str):
    """Ask the controller to disconnect from a specific drone."""
    read_all_packets()
    m2c_packet = pack_m2c_disconnect_from_drone(drone_name)
    write_packet(m2c_packet)
    c2m_response = expect_simple_response(get_packet_id(m2c_packet), 3)
    if c2m_response.response == RESPONSE_SUCCESS:
        print(f"Disconnected from drone '{drone_name}'")
    elif c2m_response.response == RESPONSE_FAIL:
        print(f"Failed to disconnect from drone '{drone_name}'")
    else:
        raise CorruptPacketError(f"Unexpected response '{c2m_response.response}'")


def arm(drone_name: str):
    """Send a controll-commands packet to a drone where just the
    CTRL_CMD_FLAG_IS_ARMED flag is set."""
    write_packet(pack_m2c_control_commands(drone_name,
                                           pitch=0, yaw=0, roll=0, thrust=0,
                                           flags=CTRL_CMD_FLAG_IS_ARMED))


def disarm(drone_name: str):
    """Send a zeroed out controll-commands packet to the drone"""
    write_packet(pack_m2c_control_commands(drone_name,
                                           pitch=0, yaw=0, roll=0, thrust=0,
                                           flags=0))


def ping(drone_name: str):
    """Sends a ping packet to a drone, which promts the drone to acknowledge the ping.
    If the controller receives the acknowledge the ping was successful."""
    m2c_packet = pack_m2c_ping_drone(drone_name)
    write_packet(m2c_packet)
    c2m_response = expect_simple_response(get_packet_id(m2c_packet), 3)
    if c2m_response.response == RESPONSE_SUCCESS:
        print(f"Drone '{drone_name}' responded")
    elif c2m_response.response == RESPONSE_FAIL:
        print(f"Drone '{drone_name}' did not respond")
    else:
        raise CorruptPacketError(f"Unexpected response '{c2m_response.response}'")


def range_test(drone_name: str):
    """Simple 'range-test' continuously prints whether pinging the drone was successful or not.
    Stop with Ctrl-C."""
    try:
        while True:
            m2c_packet = pack_m2c_ping_drone(drone_name)
            write_packet(m2c_packet)
            responded = False
            try:
                c2m_response = expect_simple_response(get_packet_id(m2c_packet), 1)
                responded = (c2m_response.response == RESPONSE_SUCCESS)
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
    """Send a restart packet to a drone. We cannot guarantee that the packet arrived/
    that the drone actually restarts."""
    write_packet(pack_m2c_restart_drone(drone_name))


def set_drone_parameters(drone_name: str, named_params: list[tuple[str, int, any]]):
    """Send named parameters to a drone."""
    m2c_packet = pack_m2c_set_drone_parameters(drone_name, named_params)
    write_packet(m2c_packet)
    c2m_response = expect_simple_response(get_packet_id(m2c_packet), 3)
    if c2m_response.response == RESPONSE_SUCCESS:
        print(f"Updated parameters on drone '{drone_name}'")
    elif c2m_response.response == RESPONSE_FAIL:
        print(f"Failed to update parameters on drone '{drone_name}'")
    else:
        raise CorruptPacketError(f"Unexpected response '{c2m_response.response}'")


def set_drone_parameter(drone_name: str, param_name: str, dtype: int, value: any):
    """Send a single parameter to a drone."""
    set_drone_parameters(drone_name, [(param_name, dtype, value)])


def monitor():
    """Continuously read packets and print them. Stop with Ctrl-C."""
    try:
        while True:
            if serialport.inWaiting() > 0:
                packet = read_packet()
                if packet.pkt_type == SPKT_C2M_MSG:
                    print(packet.msg.rstrip("\n\0"))
                else:
                    print(packet)
            else:
                time.sleep(0.01) # don't busy loop
    except KeyboardInterrupt:
        print("exit")
        return
