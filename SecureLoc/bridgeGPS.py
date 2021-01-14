import paho.mqtt.client as mqtt
import json

LOCALHOST = '127.0.0.1'
# HOST = 'lab.iut-blagnac.fr'
HOST = '192.168.77.205'
# HOST = 'test.mosquitto.org'
PORT = 1883
POUCET_TOPIC = 'poucet/1/gnss'
# POUCET_TOPIC = 'digitransit'
WEB_TOPIC = 'sensors/drone01/altitude'

web_client = None

def initMQTT_web():
    global web_client
    web_client = mqtt.Client()
    web_client.connect(LOCALHOST, PORT, 60)
    web_client.loop_start()

def initMQTT_poucet():
    client = mqtt.Client()
    client.connect(HOST, PORT, 60)
    client.subscribe(POUCET_TOPIC)
    client.on_message = on_message_poucet
    client.loop_forever()


def on_message_poucet(mqttc,obj,msg):
    print("Message received on Poucet")
    global web_client
    # decoding payload
    json_log = json.loads(msg.payload)
    # print(json_log["vehicle"])
    lat, lon = json_log["lat"], json_log["lon"]
    # lat = json_log["vehicle"]["location"]["latitude"]
    # lon = json_log["vehicle"]["location"]["longitude"]
    # converting to tuple
    print(lat, lon)
    latlon =  '[%s, %s,"Agent 1"]' % (lat, lon)
    web_client.publish(WEB_TOPIC, latlon)


if __name__ == "__main__":
    print("Initializing Web client...")
    initMQTT_web()
    print("Initializing poucet client...")
    initMQTT_poucet()