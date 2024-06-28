# vehicle.py
import paho.mqtt.client as mqtt
import json
import socket
import time
from time import sleep as wait
from vilib import Vilib
from multiprocessing import Process # Concurrency library, since we have 2 infinite loops going on here...

broker_IP = "192.168.251.142"
port_Num = 1883

client_name = socket.getfqdn()

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
    publish(client,"data_V2B",{"message":"Traffic Cone","confidence":90,"timestamp":time.time()})

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
    wait(1)

    while True:
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

            publish(client,"data_V2B",{"message":"Nothing","confidence":0,"timestamp":time.time()})
        
        wait(0.5)


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
