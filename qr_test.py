from vilib import Vilib
from time import sleep as wait

def StartCamera():
    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.show_fps()
    Vilib.display(local=True, web=True)
    wait(1)
    #Vilib.object_detect_switch(False) # DO NOT enable object detection
    Vilib.qrcode_detect_switch(True) # Enable QR detection

def main():
    StartCamera()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\033[31mERROR: {e}\033[m")
    finally:
        Vilib.camera_close()