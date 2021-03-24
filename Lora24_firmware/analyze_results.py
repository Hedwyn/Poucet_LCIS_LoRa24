import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import json
import random
import os
import sys

devices = {}
LOGS_DIR = 'LOGS/'

def parse_log(log):
    sample = None
    with open(LOGS_DIR + log) as f:
        for line in f:
            try:
                sample = json.loads(line)
            except:
                pass
            if sample:
                device = sample["chip"]
                if not(device in devices):
                    devices[device] = []
                devices[sample["chip"]].append(sample)

def compute_results_per_topic(data = "distance", topic = "frequency"):
    topic_vals = {}
    for device in devices:
        topic_vals[device] = {}
        for sample in devices[device]:
            try:
                topic_val = sample[topic]
                if not(topic_val in topic_vals[device]):
                    topic_vals[device][topic_val] = []
                    topic_vals[device][topic_val].append(float(sample[data]))
            except:
                pass

    return(topic_vals)


def moving_average(data):
    window_size = 20
    output = []
    for idx, val in enumerate(data[window_size:]):
        output.append(np.mean(data[idx + 1 - window_size: idx + 1]))
    
    return(output)

def show_distances():
    for device in devices:
        fig = plt.figure() 
        axe = fig.add_subplot(111)

def compute_skew_error():
    distances  = compute_results_per_topic("distance", "frequency")
    skew = compute_results_per_topic("skew", "frequency")
    [chip1, chip2]  = devices.keys()
    results = {}
    sf = int(devices[chip1][0]["sf"])
    symbol_time = 0.0001576 * pow(2, sf - 5)

    for channel in distances[chip1]:
        d1 = np.mean(distances[chip1][channel])
        d2 = np.mean(distances[chip2][channel])
        sigma_antenna_path = (d1 + d2) / 2
        skew_error1 = d1 - sigma_antenna_path 
        skew_error2 = d2 - sigma_antenna_path 
        print("*** Frequency ***" + str(channel))
        print("Sigma antenna path: " + str(sigma_antenna_path))
        
        sk1 = np.mean(skew[chip1][channel])
        sk2 = np.mean(skew[chip2][channel])
        expected_skew_error1 = (sk2 * 2 * symbol_time * 3E2) / 2
        expected_skew_error2 = (sk1 * 2 * symbol_time * 3E2) / 2
        print("Estimated: " + str( (expected_skew_error1, expected_skew_error2) ))
        print("Observed: " + str( (skew_error1, skew_error2) ))

        results[channel] = (expected_skew_error1, skew_error1, expected_skew_error2, skew_error2, sigma_antenna_path)
    return(results)

def display_skew_error(f = ""):
    results = compute_skew_error()
    fig = plt.figure() 
    axe = fig.add_subplot(111)
    freq = []
    data = [[], [], [], [], []] 
    colors = ["#c6c131","#98bee5", "#878a2b","#314fc6", "black"]
    if f != "":
        f = f.split("_")
        chip1 = f[0]
        chip2 = f[1]
        sf =  f[2]
        bw = f[3]
    else:
        title = ""
    title = "Clock drift-induced distance error on " + chip1 + " and " + chip2 + ", " + sf + ", BW " + bw + " KHz"
    for channel in results:
        center_freq = float(channel)
        freq.append(center_freq)
        for i,val in enumerate(results[channel]):
            data[i].append(val)

    labels = [
        "Distance error estimated from FEI " + chip1,
        "Distance error observed " + chip1,
        "Distance error estimated from FEI " + chip2,
        "Distance error observed " + chip2, 
        "Sum of antennas path delays"
        ]
    markers = ['.', '.', '.', '.', '^']
    for i, d in enumerate(data):
        axe.scatter(freq, d, marker = markers[i], color = colors[i], label = labels[i], s = 10)

    axe.set_title(title)
    axe.set_xlabel("Distance error (m)")
    axe.set_ylabel("Channel center frequency (Hz)")
    plt.legend()
    plt.grid()



def plot_results(data = "distance", topic = "frequency"):
    colors = ["black", "red"]
    color_idx = 0
    fig = plt.figure() 
    axe = fig.add_subplot(111)
    channels = compute_results_per_topic(data, topic)
    random.seed()
    for j, device in enumerate(channels):
        for idx, c in enumerate(channels[device]):
            data = channels[device][c]
            length = len(data)
            try:
                center_freq = float(c)
            except:
                pass
            std = np.std(data)
            mean = np.mean(data)
            x = [i * (1600000 / length) + center_freq for i in range(length)]

            col = random.choice([c for c in mcolors.CSS4_COLORS])
            axe.scatter(x, data, marker = ".", s = 1)
            if idx == 0:
                axe.scatter([center_freq], [std], c = colors[j], marker = ".", s = 9, label = "Standard deviation")
                axe.scatter([center_freq], [mean], c = colors[j], marker = "^", s = 9, label = "Mean")
                axe.set_title(label = "Distances measured over 40 channels, ground truth = 20 cm, SF9")
            else:
                axe.scatter([center_freq], [std], c = colors[j], marker = ".", s = 9)
                axe.scatter([center_freq], [mean], c = colors[j], marker = "^", s = 9)
            axe.set_xlabel("Frequency")
            axe.set_ylabel("Distances")
            plt.legend()
            
            color_idx += 1
            print("Channel " + c + " MEAN: " + str(mean) + " STD: "  + str(std) + " Length " + str(length))
    
    plt.show()



if __name__ =="__main__":
    if len(sys.argv) > 1:
        if sys.argv[1] == "-delay_error":
            keyword = sys.argv[2] if len(sys.argv) > 2 else ""
            for f in os.listdir(LOGS_DIR):
                if keyword in f:
                    print(f)
                if f.endswith(".json") and keyword in f: 
                    devices = {}

                    try:                 
                        parse_log(f)
                        # plot_results("rssi", "frequency")
                        display_skew_error(f)
                    except:
                        print("failed to open" + f)
                        pass
        elif sys.argv[1] == "-distances":
            keyword = sys.argv[2] if len(sys.argv) > 2 else ""
            for f in os.listdir(LOGS_DIR):
                if keyword in f:
                    print(f)
                if f.endswith(".json") and keyword in f: 
                    devices = {}
                    try:                 
                        parse_log(f)
                        plot_results("distance", "frequency")
                    except:
                        print("failed to open" + f)
                        pass
        elif sys.argv[1] == "-calibration":
            keyword = sys.argv[2] if len(sys.argv) > 2 else ""
            for f in os.listdir(LOGS_DIR):
                if keyword in f:
                    print(f)
                if f.endswith(".json") and keyword in f: 
                    devices = {}
                    try:                 
                        parse_log(f)
                        plot_results("distance", "calibration")
                    except:
                        print("failed to open" + f)
                        pass

    plt.show()

        