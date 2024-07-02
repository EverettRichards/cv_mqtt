#!/usr/bin/env python3
# TEST git change <<
from vilib import Vilib
from time import sleep

def main():
    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.show_fps()
    Vilib.display(local=True, web=True)
    sleep(1)

    # You can use the following two functions to load the model and the corresponding label
    # Vilib.object_detect_set_model(path='/opt/vilib/detect.tflite')
    ###Vilib.object_detect_set_model(path='/home/udel/cv_mqtt/tf_models/ssd-v1.tflite')
    #Vilib.object_detect_set_labels(path='/opt/vilib/coco_labels.txt')
    ###Vilib.object_detect_set_labels(path='/home/udel/cv_mqtt/tf_models/ssd-labels.txt')

    Vilib.object_detect_switch(True)

    while True: # Keep the main program running
        #print(Vilib.detect_obj_parameter)
        print(Vilib.detect_obj_parameter)
        sleep(0.5)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\033[31mERROR: {e}\033[m")
    finally:
        Vilib.camera_close()



    
