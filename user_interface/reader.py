import user_interface.communicator as cm
import time

writer = cm.Communcator()
ready = writer.open_shared_memory()
try:
    while ready:
        writer.recieve_packages()
        print(f"shared header {writer.shared_header.seq}")
        time.sleep(0.1)
    
except KeyboardInterrupt:
    pass

finally:
    if ready:
        writer.shm.close()
