START "Broker" "C:\Program Files\mosquitto\mosquitto.exe" -c "C:\Program Files\mosquitto\mosquitto.conf" -v 
SLEEP 2
START "Serial Master" "C:\Users\pestourb\AppData\Local\Programs\Python\Python36\python.exe" "C:\Users\pestourb\Documents\GitHub\Poucet_LCIS_LoRa24\SecureLoc\clientMQTT.py"
START "Flask Client" "C:\Users\pestourb\AppData\Local\Programs\Python\Python36\python.exe" "C:\Users\pestourb\Documents\Poucet\Code_LoRa\ClefUSB\Projet-Poucet\Dev-Python\LCIS\poucet-web\flask_test.py"

@REM START "SecureLoc" "C:\Users\pestourb\AppData\Local\Programs\Python\Python36\python.exe" "C:\Users\pestourb\Documents\GitHub\SecureLoc\run.py"
PAUSE