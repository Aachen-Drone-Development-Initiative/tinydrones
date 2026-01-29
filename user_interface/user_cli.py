from prompt_toolkit import PromptSession
from prompt_toolkit.completion import WordCompleter
import threading
import shlex
import keyboard
import pygame
import user_interface.font as fnt
from espnow_dongle.python_interface import espnow_dongle as espd
from user_interface import communicator as cm
import os, time

CMD_FPS = 30
CMD_MAX_VALUE = 1000
CONTROLLER_DEAD_ZONE = 0.01

class Cli():
    def __init__(self):
        # Init Cli Base
        self.CLI_COMMANDS = {'start', 'stop', 'status','drone','exit', 'help'}
        
        self.cli_completer = WordCompleter(self.CLI_COMMANDS, ignore_case=True)
        self.cli_session = PromptSession()
        self.prompt_value = "> "
        self.pitch_controll = 0
        self.roll_controll = 0
        self.yaw_controll = 0
        self.throtle_controll = 0
        self.cmd_throttle = 0
        self.cmd_roll = 0
        self.cmd_yaw = 0
        self.cmd_pitch = 0
        self.controller_mode = 2
        self.exit = False
        self.commander_controller_thread = None
        self.commander_keyboard_thread = None
        self.cli_mainloop_thread = threading.Thread(target=self.cli_mainloop)
        self.cli_mainloop_thread.start()

        self.cmd_mainloop_thread = threading.Thread(target=self.cmd_mainloop)
        self.cmd_mainloop_thread.start()
        

    def cli_mainloop(self):
        print(f"{fnt.CLI_HEADER}")
        while True:
            text = self.cli_session.prompt(self.prompt_value, completer=self.cli_completer)
            parts = shlex.split(text)

            if not parts:
                continue

            cmd, *args = parts
            flags, param = self.parse_commands(args)
            match cmd:
                case "start":
                    print("start")

                case "drone":
                    # Drone Trigger
                    if flags == set():
                        print("No Flag set")
                        print(f"{fnt.CLI_DRONE_COMMANDS}")
                        continue
                    
                    print(f"param {param} flags {flags}")
                    
                    if "--connect" in flags:
                        print(f"Connect Drone {param['name']}")
                        self.prompt_value = f"<{param['name']}> "

                    if "--list-all" in flags:
                        print("List all available drones")

                    if "--disconnect" in flags:
                        print("Disconnect from drone")
                        self.prompt_value = "> "
                    
                    if "--commander-keyboard" in flags:
                        print("Start commander")
                        self.commander_keyboard_thread = threading.Thread(target=self.commander_keyboard)
                        self.commander_keyboard_thread.start()
                    
                    if "--commander-controller" in flags:
                        self.commander_controller_thread = threading.Thread(target=self.commander_controller)
                        self.commander_controller_thread.start()

                case "help":
                    # Show help 
                    print(f"{fnt.CLI_HELP}")
                
                case "exit":
                    # Close CLI
                    self.exit = True
                    break
    
    def cmd_mainloop(self):
        self.communication = cm.Communcator()
        self.communication.create_shared_memory()
        i = 0
        while True:
            self.update_packages()
            if self.exit:
                break
            header = cm.SharedHeader()
            header.seq = i
            i += 1
            self.communication.send_package(header)
            time.sleep(1/CMD_FPS)
    
    def update_packages(self):
        m2c_controll = espd.Packet_M2C_Controll_Commands()
        m2c_controll.ctrl_cmds.throttle = int((1+self.cmd_throttle)*CMD_MAX_VALUE/2)
        m2c_controll.ctrl_cmds.pitch = int((1+self.cmd_pitch)*CMD_MAX_VALUE/2)
        m2c_controll.ctrl_cmds.roll = int((1+self.cmd_roll)*CMD_MAX_VALUE/2)
        m2c_controll.ctrl_cmds.yaw = int((1+self.cmd_yaw)*CMD_MAX_VALUE/2)
        self.communication.send_package(m2c_controll)
    
    def parse_commands(self,args):
        params = {}
        flags = set()

        for arg in args:
            if arg.startswith("-"):
                flags.add(arg)
            elif "=" in arg:
                k, v = arg.split("=", 1)
                params[k] = v
            else:
                params["name"] = arg

        return flags, params
    
    def commander_keyboard(self):
        while True:
            os.system('clear')
            print(f"{fnt.CLI_COMMANDER}")
            event = keyboard.read_event()
            if event.event_type == keyboard.KEY_DOWN:
                match event.name:
                    case "esc":
                        os.system('clear')
                        break
                    case "w":
                        self.pitch_controll = 1
                    case "s":
                        self.pitch_controll = -1
                    case "a":
                        self.roll_controll = -1
                    case "d":
                        self.roll_controll = 1
                    case "q":
                        self.yaw_controll = 1
                    case "e":
                        self.yaw_controll = -1
                    case "up":
                        self.throtle_controll = 1
                    case "down":
                        self.throtle_controll = -1
    
    def commander_controller(self):
        print(f"{fnt.CLI_COMMANDER}")
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
                time.sleep(1/CMD_FPS)

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
            
        if abs(pitch) <= CONTROLLER_DEAD_ZONE:
            pitch = 0
        if abs(roll) <= CONTROLLER_DEAD_ZONE:
            roll = 0
        if abs(yaw) <= CONTROLLER_DEAD_ZONE:
            yaw = 0
            
        self.cmd_throttle = throttle
        self.cmd_yaw = yaw
        self.cmd_roll = roll
        self.cmd_pitch = pitch        
    
if __name__ == "__main__":
    interface = Cli()
    interface.cli_mainloop_thread.join()
    interface.cmd_mainloop_thread.join()
    if interface.commander_keyboard_thread is not None:
        interface.commander_keyboard_thread.join()
    
    if interface.commander_controller_thread is not None:
        interface.commander_controller_thread.join()
    
