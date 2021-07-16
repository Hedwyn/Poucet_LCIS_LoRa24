#!/bin/python3

import paho.mqtt.client as mqtt 
import socket
import pynmea2
import os
import time
import struct
import serial
import getopt, sys
import json

'''---Global Variables---'''
if len(sys.argv) < 2:
    print("Undefined sever name. Please give the server ip adress as argument")
MQTT_BROKER_ADDRESS = sys.argv[1]

MQTT_BROKER_PORT = 1883
MQTT_PUBLISH_TOPIC_LORA = 'poucet/1/LoRa2.4'

LORA_DEVICE_PORT= '/dev/ttyS0' # default UART port on pin 14-15
serialFile = None
mqtt_client = None

lora = None

        

def handleSerialMessage(msg):
    msg = msg.decode('ascii').strip().replace('\n','')

    try:
        # msg = msg.decode('ascii').strip().replace('\n','')
        parsedMsg = json.loads(msg)
    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        print("Error!! " + msg)
        return
    print("publishing")
    mqtt_client.publish(MQTT_PUBLISH_TOPIC_LORA, loraMessage)


'''---MQTT Related Functions---'''
# Function Triggered when the MQTT Client succeeded to connect to the MQTTBroker 
def on_connect(client, userdata, flags, rc):
    print("Connected with result code", str(rc))

# Function Triggered when the MQTT Client succeeded publishing data to the MQTTBroker
def on_publish(client, userdata, result):
    print('Data Published with code ', str(result))

# Initialization Function for the Mqtt Client
def mqttInit():
    global MQTT_BROKER_ADDRESS
    global MQTT_BROKER_PORT
    global mqtt_client

    mqtt_client = mqtt.Client('LoRaClient',)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_publish = on_publish
    mqtt_client.connect(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT, 60)
    mqtt_client.loop_start()

# Main Function Call
if __name__ == "__main__": 
    mqttInit()
    try:
        serialLoRa = serial.Serial(LORA_DEVICE_PORT,115200)
    except (KeyboardInterrupt, SystemExit):
        raise
    except Exception as e:
        print("LoRa ERROR: Could not open Serial Port on "+LORA_DEVICE_PORT)
        print(e)
        emulatorIsAvailable=False
    print("LoRa Com Opened on Serial Port "+LORA_DEVICE_PORT)

    while serialLoRa.isOpen():
        try:
            loraMessage = serialLoRa.readline()
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            print("Error while reading line, disconnecting from port")
            break
        if not loraMessage:
            pass
        else:
            print("Received from LoRa: {}".format(loraMessage))
            handleSerialMessage(loraMessage)
                            
    print("LoRa disconnected")
