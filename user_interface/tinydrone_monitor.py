from user_interface import communicator as cm
from user_interface import font as fnt
import logging
import time,  sys
from datetime import datetime

# Setup Logging
log_level = logging.INFO

# Detect flags
for i in range(0,len(sys.argv)):
    if sys.argv[i] == "--debug":
        log_level = logging.DEBUG
    
# Logger Setup
logger = logging.getLogger()
logger.setLevel(log_level)
file_handler = logging.FileHandler(f"./logs/{str(datetime.now().strftime("%d-%m-%y_%H-%M-%S"))}.log")
formatter = logging.Formatter(
    '%(asctime)s [%(levelname)s] %(message)s'
)
file_handler.setFormatter(formatter)

logger.addHandler(file_handler)

TIMEOUT = 2 # [s]
MAINLOOP_FPS = 20

print(f"{fnt.TINYDRONES_LOGGER}")

class TinyDroneMonitor():
    def __init__(self):
        self.communication = cm.Communcator(logger)
        self.logger = logger
        self.success_shared_memory = self.communication.open_shared_memory()
        self.setup_variables()
        self.latest_shared_header_seq = None
        self.time_timeout = time.time()

    def main_loop(self):
        while True:
            self.get_packages()
            time.sleep(1/MAINLOOP_FPS)
            if (time.time() - self.time_timeout) >= TIMEOUT:
                self.logger.error(f"Shared Memory timeout reached ")
                break
    
    def setup_variables(self):
        # Setup Variables
        self.latest_c2m_drone_connected = None
        self.latest_c2m_drone_disconnected = None
        self.latest_c2m_drone_detected_length = None
        self.latest_c2m_imu_data_time = None
        self.latest_c2m_log_esp_msg = None
        self.latest_m2c_connect_drone = None
        self.latest_m2c_controll_list = []
        self.latest_m2c_disconnect_drone = None

    def get_packages(self):
        self.communication.recieve_packages()

        if self.communication.shared_header.seq != self.latest_shared_header_seq:
            self.time_timeout = time.time()
            #self.logger.info(f"Shared Header {self.communication.shared_header.seq}")
            self.latest_shared_header_seq = self.communication.shared_header.seq

        if self.communication.m2c_connect_drone.esp_idx != self.latest_m2c_connect_drone:
            self.logger.info(f"M2C_connect_drone: {self.communication.m2c_connect_drone.esp_idx}")
            self.latest_m2c_connect_drone = self.communication.m2c_connect_drone.esp_idx
        
        if self.communication.m2c_disconnect_drone.esp_idx != self.latest_m2c_disconnect_drone:
            self.logger.info(f"M2C disconnect drone: {self.communication.m2c_disconnect_drone.esp_idx}")
            self.latest_m2c_disconnect_drone = self.communication.m2c_disconnect_drone.esp_idx
        
        # Check if new inputs
        m2c_controll = self.communication.m2c_controll_drone.ctrl_cmds
        m2c_controll_list = [m2c_controll.throttle, m2c_controll.yaw, m2c_controll.roll,m2c_controll.pitch]
        if m2c_controll_list != self.latest_m2c_controll_list:
            # Optput 
            output = f"M2C Drone Controll: esp_idx: {self.communication.m2c_controll_drone.esp_idx} \n"
            output+= f"state_flags {m2c_controll.ctrl_state_flags} pitch: {m2c_controll.pitch}"
            output += f" roll: {m2c_controll.roll} yaw: {m2c_controll.yaw} throttle: {m2c_controll.throttle}"
            self.logger.info(output)

            self.latest_m2c_controll_list = [m2c_controll.throttle, m2c_controll.yaw, m2c_controll.roll,m2c_controll.pitch]
        """
        if self.communication.c2m_detected_drones.detected_drones != self.latest_c2m_drone_detected:
            # TODO seperation
            self.logger.info(f"C2M Detected Drones: {self.communication.c2m_detected_drones.detected_drones} ")
            self.latest_c2m_drone_detected = self.communication.c2m_detected_drones.detected_drones
        """
        if self.communication.c2m_drone_connected.esp_idx != self.latest_c2m_drone_connected:
            self.logger.info(f"C2M Drone Connected: {self.communication.c2m_drone_connected.esp_idx}")
            self.latest_c2m_drone_connected = self.communication.c2m_drone_connected.esp_idx
        
        if self.communication.c2m_drone_disconnected.esp_idx != self.latest_c2m_drone_disconnected:
            self.logger.info(f"C2M drone Disconnected: {self.communication.c2m_drone_disconnected.esp_idx}")
            self.latest_c2m_drone_disconnected = self.communication.c2m_drone_disconnected.esp_idx
        
        if self.communication.c2m_imu_data.imu_data.time_us != self.latest_c2m_imu_data_time:
            c2m_imu_data = self.communication.c2m_imu_data
            output = f"C2M IMU Data: esp_idx: {c2m_imu_data.esp_idx} data: time_us {c2m_imu_data.imu_data.time_us}\n"
            output += f"omega_x:{c2m_imu_data.imu_data.omega_x} [°/s] omega_y: {c2m_imu_data.imu_data.omega_y} [°/s] omega_z: {c2m_imu_data.imu_data.omega_z} [°/s]\n"
            output += f"acc_x: {c2m_imu_data.imu_data.acc_x} [m/s²] acc_y: {c2m_imu_data.imu_data.acc_y} [m/s²] acc_z: {c2m_imu_data.imu_data.acc_z} [m/s²]"
            self.logger.info(output)
            self.latest_c2m_imu_data_time = self.communication.c2m_imu_data.imu_data.time_us
        """
        if self.communication.c2m_log_msg.msg != self.latest_c2m_log_esp_msg:
            # TODO message transport
            output = f"C2M LOG ESP_IDX:{self.communication.c2m_log_msg.esp_idx} type: {self.communication.c2m_log_msg.log.type}"
            output += f" message_length: {self.communication.c2m_log_msg.log.msg_length}"
            self.logger.info(output)
            self.latest_c2m_log_esp_msg = self.communication.c2m_log_msg.msg
        """
        
if __name__ == "__main__":
    try:
        log = TinyDroneMonitor()
        if log.success_shared_memory:
            log.main_loop()
        else:
            log.logger.error("No Shared Memory there. Stop programm")
    except KeyboardInterrupt:
        pass

    finally:
        if log.communication.shm is not None:
            log.communication.kill_shared_memory()