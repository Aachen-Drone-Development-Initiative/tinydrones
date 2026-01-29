# Font for Tinydrone CLI
import datetime
CLI_VERSION = "1.0.0"
CLI_PUBLISH_DATE = "24.01.2026"
CLI_LOGO = r"""
  _______ _               _____                               _____ _      _____ 
 |__   __(_)             |  __ \                             / ____| |    |_   _|
    | |   _ _ __  _   _  | |  | |_ __ ___  _ __   ___  ___  | |    | |      | |  
    | |  | | '_ \| | | | | |  | | '__/ _ \| '_ \ / _ \/ __| | |    | |      | |  
    | |  | | | | | |_| | | |__| | | | (_) | | | |  __/\__ \ | |____| |____ _| |_ 
    |_|  |_|_| |_|\__, | |_____/|_|  \___/|_| |_|\___||___/  \_____|______|_____|
                   __/ |                                                         
                  |___/                                                          
"""
CLI_VERSION = f"""
Version:{CLI_VERSION} Publish at: {CLI_PUBLISH_DATE}
Started at: {datetime.datetime.now()}

FOR HELP: "help" 
FOR FINISH: "exit"
"""

CLI_HEADER = CLI_LOGO + "\n" + CLI_VERSION


CLI_HELP_LOGO = r"""
  _    _ ______ _      _____  
 | |  | |  ____| |    |  __ \ 
 | |__| | |__  | |    | |__) |
 |  __  |  __| | |    |  ___/ 
 | |  | | |____| |____| |     
 |_|  |_|______|______|_|     
                                                       
"""
CLI_DRONE_COMMANDS = f"""
############## Drone Commands ###############################
drone:
    {'--list-available:':<30}{'List all available Drones (Over ESPNow)':>20}
    {'--connect <Drone_Name>:':<30}{'Connect to Tinydrone <Drone_Name>':>20}
    {'--disconnect:':<30}{'Disconnect from current Drone':>20}
    {'--commander-keyboard:':<30}{'Start Commander to controll the drone over Keyboard':>20}
    {'--commander-controller:':<30}{'Start Commander to controll the drone over Controller':>20}
"""

CLI_HELP_CONTENT = f"""
############## General Commands #############################
exit: \t Exit the CLI
{CLI_DRONE_COMMANDS}
"""


CLI_HELP = CLI_HELP_LOGO + "\n" + CLI_HELP_CONTENT

CLI_COMMANDER_LOGO = r"""
   _____ ____  __  __ __  __          _   _ _____  ______ _____  
  / ____/ __ \|  \/  |  \/  |   /\   | \ | |  __ \|  ____|  __ \ 
 | |   | |  | | \  / | \  / |  /  \  |  \| | |  | | |__  | |__) |
 | |   | |  | | |\/| | |\/| | / /\ \ | . ` | |  | |  __| |  _  / 
 | |___| |__| | |  | | |  | |/ ____ \| |\  | |__| | |____| | \ \ 
  \_____\____/|_|  |_|_|  |_/_/    \_\_| \_|_____/|______|_|  \_\
                                                                 
"""

CLI_COMMANDER_DESCRIPTION = f"""
###### Press Key to control drone ################
{'"up"-key:':<15}{'fly drone up':>5}
{'"down"-key:':<15}{'fly drone down':>5}
{'"w"-key:':<15}{'fly drone to the front':>5}
{'"s"-key:':<15}{'fly drone back':>5}
{'"a"-key':<15}{'fly drone left':>5}
{'"d"-key':<15}{'fly drone right':>5}
{'"q"-key':<15}{'rotate drone to the left':>5}
{'"e"-key':<15}{'rotate drone to the right':>5}
"""

CLI_COMMANDER = CLI_COMMANDER_LOGO + "\n" + CLI_COMMANDER_DESCRIPTION

TINYDRONES_LOGGER = r"""
  _______ _____ _   ___     __  _____  _____   ____  _   _ ______   _      ____   _____  _____ ______ _____  
 |__   __|_   _| \ | \ \   / / |  __ \|  __ \ / __ \| \ | |  ____| | |    / __ \ / ____|/ ____|  ____|  __ \ 
    | |    | | |  \| |\ \_/ /  | |  | | |__) | |  | |  \| | |__    | |   | |  | | |  __| |  __| |__  | |__) |
    | |    | | | . ` | \   /   | |  | |  _  /| |  | | . ` |  __|   | |   | |  | | | |_ | | |_ |  __| |  _  / 
    | |   _| |_| |\  |  | |    | |__| | | \ \| |__| | |\  | |____  | |___| |__| | |__| | |__| | |____| | \ \ 
    |_|  |_____|_| \_|  |_|    |_____/|_|  \_\\____/|_| \_|______| |______\____/ \_____|\_____|______|_|  \_\
                                                                                                             
                                                                                                             
"""