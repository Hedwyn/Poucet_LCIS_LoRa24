import json
from serial import Serial
import threading
from threading import Thread
import sys
import time
import random

PORT1 = 'COM4'
PORT2 = 'COM2'
LOGS_DIR = "Logs/"

log = None
TOTAL_RANGINGS = 400
topics = ["distance", "fei", "skew", "rssi", "snr", "calibration", "frequency", "sf", "bw", "preamble-length", "duration"]


def parse_json(line):
    values = line.split("|")
    dic = {}
    for topic, val in zip(topics, values):
        dic[topic] = val
    return(dic)


def read_serial(com_port, f = None):
    global log
    sample_ctr = 0
    try:
        port = Serial(com_port, baudrate=  115200)

    except:
        print("Port cannot be found. Make sure to plug the device")
        sys.exit()
    
    # sending start command
    cmd = 'm' if (com_port == PORT1) else 's'
    port.write(cmd.encode())
    exit_flag = False
    while not(exit_flag) and (sample_ctr < TOTAL_RANGINGS):
        time.sleep(0.01)
        if port.in_waiting > 0:
            try:
                line = port.readline().decode('utf-8').strip()
                print(line)
            except:
                print("Ctrl + C received, exitting")
                exit_flag = True
                break
            if line[0] == '{' or (line[0] == '*' and len(line) >= len(topics) ):
                sample_ctr += 1
                if line[0] == '*':
                    sample = parse_json(line[1:])
                    print(sample)
                else:
                    sample = json.loads(line)
                if not f:
                    # auto-detecting parameters
                    if not log:                  
                        sf = sample["sf"]
                        bw = sample["bw"]
                        random_idx = random.randint(0, 99)
                        log_name = LOGS_DIR + PORT1 + "_" + PORT2 +  "_SF" + str(sf) + "_BW" + str(bw) + "_" + str(random_idx) + ".json"
                        log = open(log_name, "w+")
                    f = log
                sample["chip"] = port.name
                json.dump(sample, f)
                f.write("\n")
    print("Measurements complete for port " + port.name)


# read_serial(PORT1)
if __name__ == "__main__":  
    if len(sys.argv) == 2:
        filename = sys.argv[1]
        with open(LOGS_DIR + filename + ".json", 'w+') as f:
            print("yo")
            # read_serial(PORT1, f)
            t1 = Thread(target = read_serial, args = (PORT1, f,), daemon = True)
            t2 = Thread(target = read_serial, args = (PORT2, f,), daemon = True)
            t1.start()
            t2.start()
            while (threading.active_count() > 1):
                time.sleep(1)
            t1.join()
            t2.join()
    
    else:
        t1 = Thread(target = read_serial, args = (PORT1,), daemon = True)
        t2 = Thread(target = read_serial, args = (PORT2,), daemon = True)      
        t1.start()
        t2.start()
        while (threading.active_count() > 1):
            time.sleep(1)
        t1.join()
        t2.join()       
                    
                


    

    
     