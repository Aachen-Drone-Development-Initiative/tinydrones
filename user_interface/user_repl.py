from espnow_dongle.python_interface import espnow_dongle
import threading,pygame, time

SERIAL_PORT = "ser"

class Drone():
    def __init__(self):
        self.exit = False
        self.dongle = espnow_dongle.Dongle(SERIAL_PORT)
        self.mainloop_drone = threading.Thread(target=self.mainloop)
        self.mainloop_drone.start()
    
    def mainloop(self):
        while True:
            self.packages = self.dongle.read_all_packets()

    def list_drones(self):
        package = espnow_dongle.PKT_M2C_GET_DETECTED_DRONES
        package.header.type = espnow_dongle.PKT_M2C_GET_DETECTED_DRONES
        package.header.package_size = 0
        self.dongle.write_packet(package)
        for package in self.packages:
            if package == espnow_dongle.Packet_C2M_Detected_Drones():
                try:
                    self.detected_drones = package.detected_drones
                    print(f"Detected Drones are {self.detected_drones}")
                except:
                    print("No Drones Detected")

    def connect_to_drone(self, drone_name=None):
        if drone_name == None:
            print("No name of Drone given")
            return False
    
    def disconnect_from_drone(self):
        print("Disconnect from Drone")
        self.dongle.write_packet()
    
    def controll_drone(self):
        print("Start Controll Drone via Controller")
        self.controll_drone_thread = threading.Thread(target=self.controll_drone_loop)
    
    def controll_drone_loop(self):
        pygame.init()
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()

        if joystick_count == 0:
            print("No joystick connected")
        try:
            self.joystick = pygame.joystick.Joystick(0)  # first controller
            self.joystick.init()
            while True:
                self.get_controller_axis()
                if self.exit:
                    break
                #print(f"throttle: {self.cmd_throttle} yaw: {self.cmd_yaw} pitch: {self.cmd_pitch} roll: {self.cmd_roll}")
                time.sleep(1/30)

        except:
            print("No controller connected")

    def get_controller_axis(self):
        pygame.event.pump()
        if self.controller_mode == 2:
            throttle = self.joystick.get_axis(3)
            pitch = self.joystick.get_axis(1)
            roll = self.joystick.get_axis(0)
            yaw = self.joystick.get_axis(4)
            self.flight_mode = self.joystick.get_axis(5)
            self.arm = self.joystick.get_axis(6)
            
        if abs(pitch) <= 0.01:
            pitch = 0
        if abs(roll) <= 0.01:
            roll = 0
        if abs(yaw) <= 0.01:
            yaw = 0
            
        self.cmd_throttle = throttle
        self.cmd_yaw = yaw
        self.cmd_roll = roll
        self.cmd_pitch = pitch        