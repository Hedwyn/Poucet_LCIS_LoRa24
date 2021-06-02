import pynmea2
import serial
import numpy as np

port = 'COM5'
lonbuf = []
latbuf = []
length = 50
with serial.Serial(port, 115200) as s:
    while True:
        msg = s.readline().decode('utf-8')
        # print(msg)
        if msg.startswith("$GNGGA"):
            coord = pynmea2.parse(msg)
            # print(coord.longitude, coord.latitude)
            latbuf.append(coord.latitude)
            lonbuf.append(coord.longitude)

            if len(latbuf) == length:
                del latbuf[0]
                del lonbuf[0]

            lat = np.mean(latbuf)
            lon = np.mean(lonbuf)
            print(lat, lon)