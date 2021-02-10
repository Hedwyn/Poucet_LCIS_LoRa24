import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import json
import random

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

def compute_results_per_topic(topic):
    topic_vals = {}
    for device in devices:
        topic_vals[device] = {}
        for sample in devices[device]:
            topic_val = sample[topic]
            if not(topic_val in topic_vals[device]):
                topic_vals[device][topic_val] = []
            topic_vals[device][topic_val].append(float(sample["distance"]))

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


def plot_results():
    colors = ["blue", "green"]
    color_idx = 0
    fig = plt.figure() 
    axe = fig.add_subplot(111)
    channels = compute_results_per_topic("frequency")
    random.seed()
    for j, device in enumerate(channels):
        for idx, c in enumerate(channels[device]):
            distances = channels[device][c]
            length = len(distances)
            try:
                center_freq = float(c)
            except:
                pass
            std = np.std(distances)
            mean = np.mean(distances)
            x = [i * (1600000 / length) + center_freq for i in range(length)]

            col = random.choice([c for c in mcolors.CSS4_COLORS])
            axe.scatter(x, distances, marker = "o", s = 1)
            if idx == 0:
                axe.scatter([center_freq], [std], c = colors[j], marker = "*", s = 40, label = "Standard deviation")
                axe.scatter([center_freq], [mean], c = colors[j], marker = "^", s = 40, label = "Mean")
                axe.set_title(label = "Distances measured over 40 channels, ground truth = 20 cm, SF9")
            else:
                axe.scatter([center_freq], [std], c = colors[j], marker = "*", s = 40)
                axe.scatter([center_freq], [mean], c = colors[j], marker = "^", s = 40)
            axe.set_xlabel("Frequency")
            axe.set_ylabel("Distances")
            plt.legend()
            
            color_idx += 1
            print("Channel " + c + " MEAN: " + str(mean) + " STD: "  + str(std) + " Length " + str(length))
    
    plt.show()




parse_log("COM1_COM2_SF7.json")
plot_results()

        