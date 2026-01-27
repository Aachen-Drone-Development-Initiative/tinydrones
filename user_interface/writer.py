import user_interface.communicator as cm
import time, os
os.chdir("./")
writer = cm.Communcator()
writer.create_shared_memory()
i = 0
try:
    while True:
        header = cm.SharedHeader()
        header.seq = i
        print(i)
        i += 1
        writer.send_package(header)
        time.sleep(0.5)
        
    
except KeyboardInterrupt:
    pass

finally:
    writer.shm.close()
    writer.shm.unlink()
