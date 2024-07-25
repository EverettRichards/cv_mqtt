# vehicle.py
# TEST CHANGE 4
import paho.mqtt.client as mqtt
import json
from network_config import broker_IP, port_num
import socket
import time
from time import sleep as wait
from vilib import Vilib # Built-in SunFounder computer vision library
from multiprocessing import Process # Concurrency library, since we have 2 infinite loops going on here...
import numpy as np
import os
from colors import *

config = None

client_name = socket.getfqdn()

def deleteLocalConfig():
    try:
        os.remove("config.json")
    except:
        pass

def encodePayload(data):
    data["source"] = client_name
    output = bytearray()
    output.extend(map(ord,json.dumps(data)))
    return output

def decodePayload(string_data):
    return json.loads(string_data)

def publish(client,topic,message):
    client.publish(topic,payload=encodePayload(message),qos=0,retain=False)
    prYellow(f"Emitted message: {message}")

def on_connect(client, userdata, flags, rc):
    prCyan(f"Connected with result code {rc}")
    # Subscribe to view incoming verdicts
    client.subscribe("verdict")
    client.subscribe("msg_B2V")
    client.subscribe("config")
    # Tell the server that this client exists!
    publish(client,"new_client",{"message":"New Client :)"})
    # Publish test data to the server.
    #publish(client,"data_V2B",{"message":"Traffic Cone","confidence":90,"timestamp":time.time()})

def processVerdict(payload):
    prGreen(f"Verdict received. The objects are: " + str(payload["message"]))

def writeConfig(payload):
    global config
    if config != None: return
    config = payload
    conf_file = open("config.json","w")
    conf_file.write(json.dumps(config))
    prCyan(f"Config received: {config}")

def waitForConfig():
    global config
    while config == None:
        try:
            publish(client,"request_config",{"message":"Please send me the config!"})
            conf_file = open("config.json","r")
            config = json.loads(conf_file.read())
        except:
            prRed("Config not received yet. Waiting...")
        wait(0.5)

# The callback function, it will be triggered when receiving messages
def on_message(client, userdata, msg):
    # Turn from byte array to string text
    payload = msg.payload.decode("utf-8")
    # Turn from string text to data structure
    payload = decodePayload(payload)
    # Handle the message
    if msg.topic == "verdict":
        # Receive a verdict from the server. Utilize it.
        processVerdict(payload)
    elif msg.topic == "config":
        writeConfig(payload)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Set the will message, when the Raspberry Pi is powered off, or the network is interrupted abnormally, it will send the will message to other clients
client.will_set('end_client',encodePayload({"message":"Client Expired :("}))

# Create connection, the three parameters are broker address, broker port number, and keep-alive time respectively
client.connect(broker_IP, port_num, keepalive=60)

# Set the network loop blocking, it will not actively end the program before calling disconnect() or the program crash
def network_loop():
    client.loop_forever()

def find_closest_object(dict_of_numbers, number):
    closest_object = None
    closest_distance = None
    for i,v in dict_of_numbers.items():
        if v == None: continue
        if v > 180: v -= 360
        if closest_object == None or abs(v-number) < closest_distance:
            closest_object = i
            closest_distance = abs(v-number)
    return closest_object

def get_distance(obj,car):
    obj_loc = config["object_locations"][obj]
    car_loc = config["vehicle_locations"][car]
    return np.sqrt((obj_loc["x"]-car_loc["x"])**2 + (obj_loc["y"]-car_loc["y"])**2)

# VILIB CODE...
def ComputerVision():
    waitForConfig()
    global config
    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.show_fps()
    Vilib.display(local=True, web=True)
    wait(1)
    Vilib.object_detect_switch(True)
    wait(1)

    # Import from the collective config file
    object_locations = config["object_locations"]
    vehicle_locations = config["vehicle_locations"]

    # The current car's physical location in 2D space
    my_loc = vehicle_locations[client_name]

    angles_to_each_object = {} # Strictly in Degrees

    # Initialize the angles to each object
    for obj in object_locations.keys():
        obj_loc = object_locations[obj]
        theta = np.arctan2(obj_loc["y"]-my_loc["y"],obj_loc["x"]-my_loc["x"]).item()
        angles_to_each_object[obj] = (my_loc["car_angle"]-my_loc["camera_angle"]) - theta * 180 / np.pi

    horizontal_angle_per_pixel = config["horizontal_FOV"] / config["image_width"]
    #vertical_angle_per_pixel = config["vertical_FOV"] / config["image_height"]
    screen_center_x = config["image_width"] / 2
    #screen_center_y = config["image_height"] / 2

    while True:
        object_list = {}
        for obj in object_locations.keys():
            object_list[obj] = None
        detected_objects = Vilib.object_detection_list_parameter.copy()

        # Look through list of objects
        for obj in detected_objects:
            # ATTRIBUTES: class_name, score, bounding_box[] (4 32-bit floats)

            # Calculate on-screen angle between object and robot, using label
            bounds = obj["bounding_box"]
            y1,x1,y2,x2 = bounds # IDK what order these are actually presented in. CALIBRATE!
            y1,y2 = y1*config["image_height"],y2*config["image_height"] # Adjust to pixel size
            x1,x2 = x1*config["image_width"],x2*config["image_width"] # Adjust to pixel size
    
            x_center = (x1+x2)/2

            # Find out how many degrees off-center the detected object is
            delta_x = x_center - screen_center_x
            delta_theta = delta_x * horizontal_angle_per_pixel

            # Determine which of the known objects this object is closest to, in terms of angle
            closest_object = find_closest_object(angles_to_each_object,delta_theta)
            closest_angle = angles_to_each_object[closest_object]
            angle_difference = abs(delta_theta - closest_angle)
            print(f"Object {obj['class_name']} is {angle_difference} degrees off from object {closest_object}")

            # If the angle is within the threshold, and the object is more confident than the last one (if any), update the object list
            if angle_difference < config["angle_threshold"]:
                if object_list[closest_object] == None or object_list[closest_object][1] < obj["score"]:
                    if obj and "class_name" in obj.keys():
                        object_list[closest_object] = [obj["class_name"],float(obj["score"]),float(get_distance(closest_object,client_name))]
                        if config["show_verbose"]:
                            prLightPurple(f"Object {closest_object} detected: {obj['class_name']} with confidence {obj['score']}")

        # Send out the final decision of what the robot sees!
        publish(client,"data_V2B",{"object_list":object_list})
        
        wait(config["submission_interval"])

# object_list looks like: {"object_name":[class_name,confidence,distance], ... (n=#real_objects)}

if __name__ == "__main__":
    try:
        deleteLocalConfig()
        Process(target=network_loop).start()
        wait(0.5)
        #while config == None:
            #publish(client,"request_config",{"message":"Please send me the config!"})
            #wait(0.5)
        ComputerVision()
    except KeyboardInterrupt:
        pass
    #except Exception as e:
        #print(f"\033[31mERROR: {e}\033[m")
    finally:
        Vilib.camera_close()
