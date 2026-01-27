import time
import numpy as np
from multiprocessing import shared_memory, Semaphore
import logging
import ctypes
from espnow_dongle.python_interface import espnow_dongle as espd

# Shared Memory name
name_shared_memory = "tinydrones"
sem = Semaphore(1)

logging.basicConfig(format='%(asctime)s [%(levelname)s] %(message)s',level=logging.INFO)

class SharedHeader(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("version", ctypes.c_uint8),
        ("seq", ctypes.c_uint32),
        ("flags", ctypes.c_uint8),
    ]

class Communcator():
    def __init__(self, logger=None):
        self.name_shared_memory = name_shared_memory
        if logger == None:
            self.logger = logging.getLogger()
        else:
            self.logger = logger
        self.logger.info("Init Communicator")
        self.latest_buffer = None
        self.setup_shared_memory_layout()
    
    def setup_shared_memory_layout(self):
        self.logger.info("Setup shared memory layout")
        HEADER_OFFSET = 0
        # M2C Master to controller
        self.M2C_CONNECT_DRONE_OFFSET = HEADER_OFFSET + ctypes.sizeof(SharedHeader)
        self.M2C_DISCONNECT_DRONE_OFFSET = self.M2C_CONNECT_DRONE_OFFSET + ctypes.sizeof(espd.Packet_M2C_Connect_To_Drone)
        self.M2C_CONTROLL_DRONE_CMD_OFFSET = self.M2C_DISCONNECT_DRONE_OFFSET + ctypes.sizeof(espd.Packet_M2C_Disconnect_From_Drone)
        self.M2C_GET_DETECTED_DRONES_OFFSET = self.M2C_CONTROLL_DRONE_CMD_OFFSET + ctypes.sizeof(espd.Packet_M2C_Controll_Commands)

        # C2M Controller to Master
        self.C2M_DRONE_CONNECTED_OFFSET = self.M2C_GET_DETECTED_DRONES_OFFSET + ctypes.sizeof(espd.Packet_M2C_Get_Detected_Drones)
        self.C2M_DRONE_DISCONNECTED_OFFSET = self.C2M_DRONE_CONNECTED_OFFSET + ctypes.sizeof(espd.Packet_C2M_Drone_Connected)
        self.C2M_DETECTED_DRONES_OFFSET = self.C2M_DRONE_DISCONNECTED_OFFSET + ctypes.sizeof(espd.Packet_C2M_Drone_Disconnected)
        self.C2M_IMU_DATA_OFFSET = self.C2M_DETECTED_DRONES_OFFSET + ctypes.sizeof(espd.Packet_C2M_Detected_Drones)
        self.C2M_LOG_MSG_OFFSET = self.C2M_IMU_DATA_OFFSET + ctypes.sizeof(espd.Packet_C2M_Imu_Data)

        # Size Shared Memory
        self.TOTAL_SIZE_SHARED_MEMORY = self.C2M_LOG_MSG_OFFSET + ctypes.sizeof(espd.Packet_C2M_Log_Msg)

    def create_shared_memory(self):
        # Create shared memory 
        self.logger.info(f" Create Shared Memory (name:{self.name_shared_memory}, size: {self.TOTAL_SIZE_SHARED_MEMORY})")
        self.shm = shared_memory.SharedMemory(name=self.name_shared_memory,create=True, size=self.TOTAL_SIZE_SHARED_MEMORY)
    
    def open_shared_memory(self):
        # Open existing shared memory
        try:
            self.shm = shared_memory.SharedMemory(name=self.name_shared_memory)
            self.logger.info(f"Open Shared Memory (name:{self.name_shared_memory})")
            return True
        
        except:
            self.logger.error(f"Shared Memory {self.name_shared_memory} is not existing.")
            return False
    
    def send_package(self, packages):
        # Send Package to shared memory
        if sem:
            match packages:
                case espd.Packet_M2C_Connect_To_Drone():
                    # Save Connect Drone package
                    self.shm.buf[self.M2C_CONNECT_DRONE_OFFSET:self.M2C_CONNECT_DRONE_OFFSET + 
                                ctypes.sizeof(espd.Packet_M2C_Connect_To_Drone)] = bytes(packages)
                    self.logger.debug(f"Send M2C_Connect_Drone Package {packages.__str__}")
                
                case espd.Packet_M2C_Disconnect_From_Drone():
                    # Save Disconnect Drone packages
                    self.shm.buf[self.M2C_DISCONNECT_DRONE_OFFSET:self.M2C_DISCONNECT_DRONE_OFFSET + 
                                ctypes.sizeof(espd.Packet_M2C_Disconnect_From_Drone)] = bytes(packages)
                    
                    self.logger.debug(f"Send M2C_Disconnect_Drone Package {packages}")
                
                case espd.Packet_M2C_Controll_Commands:
                    # Save Controll Commands packages
                    self.shm.buf[self.M2C_CONTROLL_DRONE_CMD_OFFSET:self.M2C_CONTROLL_DRONE_CMD_OFFSET + 
                                ctypes.sizeof(espd.Packet_M2C_Controll_Commands)] = bytes(packages)
                    self.logger.debug(f"Send M2C_Controll_Drone Package {packages}")
                
                case espd.Packet_M2C_Get_Detected_Drones():
                    # Save Detect Drones packages
                    self.shm.buf[self.M2C_GET_DETECTED_DRONES_OFFSET:self.M2C_GET_DETECTED_DRONES_OFFSET + 
                                ctypes.sizeof(espd.Packet_M2C_Get_Detected_Drones)] = bytes(packages)
                    
                    self.logger.debug(f"Send M2C_Get_Detected_Drone Package {packages}")
                
                case espd.Packet_C2M_Drone_Connected():
                    self.shm.buf[self.C2M_DRONE_CONNECTED_OFFSET:self.C2M_DRONE_CONNECTED_OFFSET +
                                ctypes.sizeof(espd.Packet_C2M_Drone_Connected)] = bytes(packages)
                    self.logger.debug(f"Send C2M_Connected_Drone Package {packages}")
                
                case espd.Packet_C2M_Drone_Disconnected():
                    self.shm.buf[self.C2M_DRONE_DISCONNECTED_OFFSET:self.C2M_DRONE_DISCONNECTED_OFFSET +
                                ctypes.sizeof(espd.Packet_C2M_Drone_Disconnected)] = bytes(packages)
                    self.logger.debug(f"Send C2M_Disconnected_Drone Package {packages}")
                
                case espd.Packet_C2M_Detected_Drones():
                    self.shm.buf[self.C2M_DETECTED_DRONES_OFFSET:self.C2M_DETECTED_DRONES_OFFSET + 
                                ctypes.sizeof(espd.Packet_C2M_Detected_Drones)] = bytes(packages)
                    self.logger.debug(f"Send C2M_Detected_Drones Package {packages}")
                
                case espd.Packet_C2M_Imu_Data():
                    self.shm.buf[self.C2M_IMU_DATA_OFFSET:self.C2M_IMU_DATA_OFFSET +
                                ctypes.sizeof(espd.Packet_C2M_Imu_Data)] = bytes(packages)
                    self.logger.debug(f"Send C2M IMU DATA Package {packages}")
                
                case espd.Packet_C2M_Log_Msg():
                    self.shm.buf[self.C2M_LOG_MSG_OFFSET:self.C2M_LOG_MSG_OFFSET +
                                ctypes.sizeof(espd.Packet_C2M_Log_Msg)] = bytes(packages)
                    self.logger.debug(f"Send M2C_MSG Log Package {packages}")
                
                case SharedHeader():
                    self.shm.buf[0:ctypes.sizeof(SharedHeader)] = bytes(packages)
                    self.logger.debug(f"Send Shared Header Package {packages}")

                case _:
                    self.logger.error(f"Package is not a valid option {packages}")

    def recieve_packages(self):
        # Recieve Packages
        buffer = self.shm.buf

        # No Changes in shared memory skip
        if buffer == self.latest_buffer:
            return
        
        self.logger.debug(f"Recieve Package {buffer}")
        self.latest_buffer = buffer

        self.shared_header = SharedHeader.from_buffer(buffer[0:ctypes.sizeof(SharedHeader)])
        
        self.m2c_connect_drone = espd.PKT_M2C_CONNECT_TO_DRONE.from_bytes(buffer[
            self.M2C_CONNECT_DRONE_OFFSET:self.M2C_CONNECT_DRONE_OFFSET + ctypes.sizeof(espd.Packet_M2C_Connect_To_Drone)
        ])

        self.m2c_disconnect_drone = espd.PKT_M2C_DISCONNECT_FROM_DRONE.from_bytes(buffer[
            self.M2C_DISCONNECT_DRONE_OFFSET:self.M2C_DISCONNECT_DRONE_OFFSET + ctypes.sizeof(espd.Packet_M2C_Disconnect_From_Drone)
        ])

        self.m2c_controll_drone = espd.PKT_M2C_CONTROLL_COMMANDS.from_bytes(buffer[
            self.M2C_CONTROLL_DRONE_CMD_OFFSET:self.M2C_CONTROLL_DRONE_CMD_OFFSET + ctypes.sizeof(espd.Packet_M2C_Controll_Commands)
        ])

        self.m2c_get_detected_drones = espd.PKT_M2C_GET_DETECTED_DRONES.from_bytes(buffer[
            self.M2C_GET_DETECTED_DRONES_OFFSET:self.M2C_GET_DETECTED_DRONES_OFFSET + ctypes.sizeof(espd.Packet_M2C_Get_Detected_Drones)
        ])

        self.c2m_drone_connected = espd.PKT_C2M_DRONE_CONNECTED.from_bytes(buffer[
            self.C2M_DRONE_CONNECTED_OFFSET:self.C2M_DRONE_CONNECTED_OFFSET + ctypes.sizeof(espd.Packet_C2M_Drone_Connected)
        ])

        self.c2m_drone_disconnected = espd.PKT_C2M_DRONE_DISCONNECTED.from_bytes(buffer[
            self.C2M_DRONE_DISCONNECTED_OFFSET:self.C2M_DRONE_DISCONNECTED_OFFSET + ctypes.sizeof(espd.Packet_C2M_Drone_Disconnected)
        ])

        self.c2m_detected_drones = espd.PKT_C2M_DETECTED_DRONES.from_bytes(buffer[
            self.C2M_DETECTED_DRONES_OFFSET:self.C2M_DETECTED_DRONES_OFFSET + ctypes.sizeof(espd.Packet_C2M_Detected_Drones)
        ])

        self.c2m_imu_data = espd.PKT_C2M_IMU_DATA.from_bytes(buffer[
            self.C2M_IMU_DATA_OFFSET:self.C2M_IMU_DATA_OFFSET + ctypes.sizeof(espd.Packet_C2M_Imu_Data)
        ])

        self.c2m_log_msg = espd.PKT_C2M_LOG_MSG.from_bytes(buffer[
            self.C2M_LOG_MSG_OFFSET:self.C2M_LOG_MSG_OFFSET + ctypes.sizeof(espd.Packet_C2M_Log_Msg)
        ])

