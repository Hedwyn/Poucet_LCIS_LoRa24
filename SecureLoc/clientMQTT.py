from serial import *
import serial.tools.list_ports
import paho.mqtt.client as mqtt
import time
import sys
import os
import threading

## network parameters
# remote
# HOST = '169.254.1.1'
# local
HOST = '127.0.0.1'
PORT = 1883 # default mqtt port

## serial parameters
# windows
WIN = True
LINUX = not(WIN)
if WIN:
    SERIALPATH = 'COM10'
    SERIAL_ROOT = ''
    SERIAL_PREFIX = 'COM'
else:
    SERIALPATH = '/dev/ttyACM0'
    SERIAL_ROOT = '/dev/'
    SERIAL_PREFIX = 'ttyACM'

SLEEP_TIME = 0.01
WATCHDOG_TIMEOUT = 5

## mqtt field variables init
distance = 0
anchor_id = ''
bot_id = ''
rssi = 0

## mqtt format
ROOT = 'SecureLoc/anchors_data/'
COOPERATIVE_ROOT = 'SecureLoc/cooperative_data/'
TOPIC_SERIAL = 'Serial'
MQTT_CONFIG_FILE = 'MQTT_topics.txt'

## SX1280 topics- all the measured data sent by SX280 on serial part 
## formatted as topic1_val|topic2_val|...topicN_val#
SX1280_TOPICS = ["master_id", "slave_id", "distance", "time_of_flight", "fei", "rssi"]
NB_DATA = len(SX1280_TOPICS)

## correspence between SX1280 topics and SecureLoc MQTT topics
MQTT_TOPICS = {
    "slave_id":"anchor_id",
    "master_id":"bot_id",
    "distance":"distance",
    "time_of_flight":"t1",
    "fei":"t2",
    "rssi":"rssi"}

## serial containers & flags
devices = []
connected_anchors = {}
serial_ports = {}
exit_flag = False


class ClientStub(object):
    def __getattribute__(self,attr):
        def method(*args, **kwargs):
            pass
        return method

    def __setattribute__(self,attr):
        pass


## starting mqtt client
# stubing mqtt if no broker is found
try:
    mqttc = mqtt.Client()
    mqttc.connect(HOST, PORT, 60)
    mqttc.loop_start()
except:
    mqttc = ClientStub()


def on_message(mqttc,obj,msg):
    """handles data resfreshing when a mqtt message is received"""
    labels = msg.topic.split("/")
    serial_message = msg.payload
    anchorID = labels[-2]
    print("Sending serial command to anchor " + anchorID + " :" + serial_message.encode())
    for port_name, ID in connected_anchors.items():
        if ID == anchorID:
            serial_ports[port_name].write(serial_message)


mqttc.on_message = on_message
def getSerialPorts():
    """returns all the serial devices connect as a list"""
    ports = []
    if SERIAL_PREFIX == 'COM':
        # windows system
        files = [port.device for port in list(serial.tools.list_ports.comports())]
    else:
        # linux
        files= os.listdir(SERIAL_ROOT)
    for entry in files:
        if entry.startswith(SERIAL_PREFIX):
            ports.append(entry)

    print('found serial devices: ' + str(ports) )
    return(ports)


def openPort(path = SERIALPATH):
    """open the serial port for the given path"""
    try:
        port = Serial(path, baudrate = 115200)


    except :
        print("No serial device on the given path :" + path)
        sys.exit()

    return(port)


def processLine(line, port):
    """parses the serial line received. If it is a DATA frame, publishes the data on MQTT.
    DEBUG frames will be ignored"""
    if line and line[0] == '*':
        data = line[1:].split("|")
        if len(data) < NB_DATA:
            print('received frame is not compliant with the expected data format')
        if len(data) >= NB_DATA:
            anchor_id = data[0]
            bot_id = data[1]
            port_name = port.name.split("/")[-1]
            if (port_name in connected_anchors) and not(connected_anchors[port_name]):
                connected_anchors[port_name] = anchor_id
                print("subscribing to " + ROOT + anchor_id + "/" + TOPIC_SERIAL )
                mqttc.subscribe(ROOT + anchor_id + "/" + TOPIC_SERIAL)

            # publishing to MQTT
            data_to_publish = {}
            for sx_topic, value in zip(SX1280_TOPICS, data):
                mqtt_topic = MQTT_TOPICS[sx_topic]
                data_to_publish[mqtt_topic] = value

            # getting ids
            anchor_id = data_to_publish["anchor_id"]
            bot_id = data_to_publish["bot_id"]
            for topic in [key for key in data_to_publish if (key != "anchor_id" and key != "bot_id")]:
                value = data_to_publish[topic]
                print(ROOT + str(anchor_id) + "/" + str(bot_id) + "/" + topic, value )
                mqttc.publish(ROOT + str(anchor_id) + "/" + str(bot_id) + "/" + topic, value )


def rebootTeensy():
    """reboot the TeensyDuino.
    Reflashing Teensyduino is required given that no remote soft reset is provided besides the bootloader"""

    print("Resetting Teensy...")
    os.system('/home/pi/Desktop/teensy_loader_cli -mmcu=mk20dx256 -s -v ' + '/home/pi/Desktop/node*.hex')





def waitSerialDevice(path):
    """waits until the given serial device is connected"""
    print("Waiting for serial device...")
    while ( not(os.path.exists(path)) ):
        time.sleep(1)

    print("Serial device found")


def readSerial(port):
    """reads the given serial port line by line"""
    global exit_flag
    print('Reading serial port ' + port.name)
    exit_flag = False
    try:
        while True:       
            if (port.in_waiting > 0 ):
                line = port.readline().decode('utf-8').strip()
                print(line)
                try:
                    processLine(line, port)
                except:
                    print("process line failed")

    except KeyboardInterrupt:
        print("Ctrl + C received, quitting")
        exit_flag = True

    except:
        print('An error occured, did you unplug the device ?')

    print('Stopped reading ' + port.name)
    port.close()

def handleSerialDevice(path = SERIALPATH):
    """opens the serial port and starts reading it"""
    port = openPort(path)
    serial_ports[port.name.split("/")[-1]] = port
    readSerial(port)


# Program

# ports = getSerialPorts()
# path = SERIALPATH

# def serialPool():
#     """checks for new serial devices connecting, opens and read device when detected"""
#     global exit_flag
#     # devices = getSerialPorts()
#     devices = [SERIALPATH]
#     threads_pool = []
#     # checking if any serial device is connected
#     if len(devices) > 0:
#         for device in devices:
#             # creating a thread reading serial port
#             threads_pool.append(threading.Thread(target = handleSerialDevice, args = (SERIAL_ROOT + device,), daemon = True ))
#             connected_anchors[device] = None
#     else:
#         # waiting for any device to connect;
#         waitSerialDevice(SERIALPATH)

#     # starting threads
#     for thread in threads_pool:
#         thread.start()

#     while not(exit_flag):
#         try:
#             time.sleep(0.5)
#         except KeyboardInterrupt:
#             print(" received, quitting program...")
#             exit_flag = True
#             for thread in threads_pool:
#                 thread.join()
#             sys.exit()





# serialPool()
handleSerialDevice()