import paho.mqtt.client as mqtt
import time

c = mqtt.Client("test")
c.connect("169.254.152.228")
time.sleep(10)