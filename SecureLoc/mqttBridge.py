import paho.mqtt.client as mqtt
from convert_gps_coord import convert_coordinates_to_gps

HOST = '127.0.0.1'
PORT = 1883 # default mqtt port

MQTT_TOPIC = 'SecureLoc/IPS/01'
WEB_TOPIC = 'sensors/drone01/altitude'

ANCHOR1_COORD = (0, 0, 0)
ANCHOR2_COORD = (10, 0, 0)
ANCHOR1_LATLON = 43.649849877192985,1.3745697631578948
ANCHOR2_LATLON = 43.64979264035088,1.374912307017544

def initMQTT():
    client = mqtt.Client()
    client.connect(HOST, PORT, 60)
    client.subscribe(MQTT_TOPIC)
    client.on_message = on_message
    client.loop_forever()

def on_message(mqttc,obj,msg):
    # decoding payload
    payload = msg.payload.decode('utf-8')
    # converting to tuple
    pos = tuple(map(float, payload[1:-1].split(",")))
    lat, lon =convert_coordinates_to_gps(pos, ANCHOR1_LATLON, ANCHOR2_LATLON, ANCHOR1_COORD, ANCHOR2_COORD )
    print(lat, lon)
    latlon =  '[%s, %s,"Agent 1"]' % (lat, lon)
    mqttc.publish(WEB_TOPIC, latlon)


initMQTT()


