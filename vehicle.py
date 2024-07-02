# vehicle.py
# TEST CHANGE
import paho.mqtt.client as mqtt
import json
import socket
import time
from time import sleep as wait
from vilib import Vilib # Built-in SunFounder computer vision library
from multiprocessing import Process # Concurrency library, since we have 2 infinite loops going on here...

broker_IP = "192.168.147.42"
port_Num = 1883

client_name = socket.getfqdn()
print(client_name)

def encodePayload(data):
    data["source"] = client_name
    output = bytearray()
    output.extend(map(ord,json.dumps(data)))
    return output

def decodePayload(string_data):
    return json.loads(string_data)

def publish(client,topic,message):
    client.publish(topic,payload=encodePayload(message),qos=0,retain=False)
    print(f"Emitted message: {message}")

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    # Subscribe to view incoming verdicts
    client.subscribe("verdict")
    client.subscribe("msg_B2V")
    # Tell the server that this client exists!
    publish(client,"new_client",{"message":"New Client :)"})
    # Publish test data to the server.
    #publish(client,"data_V2B",{"message":"Traffic Cone","confidence":90,"timestamp":time.time()})

def processVerdict(payload):
    print(f"Verdict received. The object is: " + payload["message"])

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

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Set the will message, when the Raspberry Pi is powered off, or the network is interrupted abnormally, it will send the will message to other clients
client.will_set('end_client',encodePayload({"message":"Client Expired :("}))

# Create connection, the three parameters are broker address, broker port number, and keep-alive time respectively
client.connect(broker_IP, port_Num, keepalive=60)

# Set the network loop blocking, it will not actively end the program before calling disconnect() or the program crash
def network_loop():
    client.loop_forever()


# VILIB CODE...

def ComputerVision():
    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.show_fps()
    Vilib.display(local=True, web=True)
    Vilib.traffic_detect_switch(True)
    Vilib.image_classify_switch(True)
    wait(1)

    image_name = None
    sign_name = None
    image_confidence = 0
    sign_confidence = 0

    iteration_counter = 0

    while True:
        iteration_counter += 1
        '''
        t = Vilib.traffic_sign_obj_parameter['t']
        if t != 'none':
            x = Vilib.traffic_sign_obj_parameter['x']
            y = Vilib.traffic_sign_obj_parameter['y']
            w = Vilib.traffic_sign_obj_parameter['w']
            h = Vilib.traffic_sign_obj_parameter['h']
            acc = Vilib.traffic_sign_obj_parameter['acc']

            print(f"{t} ({acc}%), coordinate=({x}, {y}), size={w}*{h}")
        
            publish(client,"data_V2B",{"message":t,"confidence":acc,"timestamp":time.time()})
        else:
            print(f'No traffic sign found')
            TOPRINT = "Banana"
            if client_name == "euclid":
                TOPRINT = "Apple"
            publish(client,"data_V2B",{"message":TOPRINT,"confidence":0,"timestamp":time.time()})
        '''
        img_name_temp = Vilib.image_classification_obj_parameter['name']
        img_acc_temp = Vilib.image_classification_obj_parameter['acc']

        if img_acc_temp >= image_confidence:
            image_name = img_name_temp
            image_confidence = img_acc_temp

        sign_type = Vilib.traffic_sign_obj_parameter['t']
        sign_acc = Vilib.traffic_sign_obj_parameter['acc']
        if sign_type != 'none' and sign_acc > sign_confidence:
            sign_name = sign_type
            sign_confidence = sign_acc/100
        #print(f'{name} {acc:.3f}')
        #publish(client,"data_V2B",{"label":name,"confidence":acc,"timestamp":time.time()})

        # Every 10th iteration, submit the strongest decision to the edge server for consideration.
        if (iteration_counter % 10 == 0):
            # If a Sign was detected, choose it. Otherwise, use image classification.
            chosen_name = sign_name if sign_confidence > 0 else image_name
            chosen_confidence = sign_confidence if sign_confidence > 0 else image_confidence

            # Send out the final decision of what the robot sees!
            publish(client,"data_V2B",{"label":chosen_name,"confidence":chosen_confidence,"timestamp":time.time()})

            # Reset the parameters
            image_name = None
            sign_name = None
            image_confidence = 0
            sign_confidence = 0
        
        wait(0.1)


if __name__ == "__main__":
    try:
        Process(target=network_loop).start()
        ComputerVision()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\033[31mERROR: {e}\033[m")
    finally:
        Vilib.camera_close()
