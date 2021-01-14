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
MQTT_BROKER_ADDRESS = '127.0.0.1'
MQTT_BROKER_PORT = 1883
MQTT_PUBLISH_TOPIC_LORA = 'poucet/1/LoRa2.4'

LORA_DEVICE_PORT=None
serialFile = None
mqtt_client = None

lora = None

# Script Parameters Handling
def handleArgs():
    global serialFile, LORA_DEVICE_PORT
    # Retrieve all arguments
    fullCmdArguments = sys.argv
    # Retrieve arguments without the script name
    argumentList = fullCmdArguments[1:]

    # Possible Arguments
    unixOptions = "hf:vs"  
    gnuOptions = ["help", "file=", "s0"]  
    try:  
        arguments, values = getopt.getopt(argumentList, unixOptions, gnuOptions)
    except getopt.error as err:  
        # output error, and return with an error code
        print(str(err))
        sys.exit(2)

    # Handle Arguments
    # evaluate given options
    for currentArgument, currentValue in arguments:  
        if currentArgument in ("-v", "--verbose"):
            print ("enabling verbose mode")
        elif currentArgument in ("-h", "--help"):
            print('\nUse: \n \
                \t-f or --file= set the serial port file name for emulation mode, enables emulation mode\n\
                \t   for example: "./serial.port"\n\
                \t   The program will then use the file to seek its serial port name\n\
                \t   No Default value\n')
            exit(0)
        
        elif currentArgument in ("-f", "--file"):
            print (("Serial Port File: {}").format(currentValue))
            serialFile = currentValue
            print("Emulation Mode Activated\n")
        elif currentArgument in ("-s", "--s0"):
            print ("Using serial port /dev/ttyS0")
            LORA_DEVICE_PORT="/dev/ttyS0"
        

def handleSerialMessage(msg):
    try:
        msg = msg.decode('ascii').strip().replace('\n','')
        print(msg)
        parsedMsg = json.loads(msg)
    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        print("Error Reading JSON, corrupted JSON file")
        return
    try:
        if parsedMsg["type"] == "LoRa2.4":
            print("Publishing LoRa2.4 message")
            mqtt_client.publish(MQTT_PUBLISH_TOPIC_LORA,loraMessage)
    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        print("Error Reading JSON, property 'type' missing")

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
    handleArgs()
    mqttInit()
    emulatorIsAvailable = False
    while True:
        if not emulatorIsAvailable:
            if serialFile is not None:
                with open(serialFile,'r') as f:
                    LORA_DEVICE_PORT = f.readline()
                    print("Serial Emulation Port Found: \n\t"+LORA_DEVICE_PORT)
                    emulatorIsAvailable=True
        time.sleep(0.5)
        if LORA_DEVICE_PORT is None:
            LORA_DEVICE_PORT = os.popen('ls -d -1 /dev/serial/by-id/* | grep STM32').read().strip()
        if LORA_DEVICE_PORT == '' :
            print("No STM32 device in /dev/serial/by-id/*")
            emulatorIsAvailable=False
            time.sleep(10)
        else: 
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
                    emulatorIsAvailable = False
                    break
                if not loraMessage:
                    pass
                else:
#                   print("Received from LoRa: {}".format(loraMessage))
                    handleSerialMessage(loraMessage)
                            
            print("LoRa disconnected")
