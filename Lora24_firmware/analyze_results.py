import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import json
import random
import os
import sys

plt.style.use('C:/Users/pestourb/Documents/JPMC/Stylefiles/classic.mplstyle')

mpl.rcParams['lines.linewidth'] = 1

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


def linear_plot():
    """Plots the results in chronological order"""
    figs = {}
    topics = ["rssi", "distance","tx_power"]
    maxes = {"rssi": 0, "distance": 100,"tx_power": 13}

    for t in topics:
        f = plt.figure()
        figs[t] = f,f.add_subplot(111)
    nodes = ["COM3", "COM4"]
   
    for node in nodes:
        data = {} 
        for sample in devices[node]:
            for entry in sample:
                if entry in topics:
                    if not(entry in data):
                        data[entry] = []
                    try:
                        val = float(sample[entry])
                        if val < maxes[entry]:
                            data[entry].append(val)
                    except:
                        pass


        for topic in topics:
            f, a = figs[topic]
            a.plot(data[topic], label = topic + " node " + node)
            a.set_title(topic)
    plt.legend()
    plt.grid()

def plot_default_correction_values(fl):
    plots = {}
    ## extraction
    with open(fl) as f:
        for line in f:
            sample = json.loads(line) 
            entry = "SF" + str(sample["SF"]) + ", " + "BW " + str(sample["BW"]) + " KHz"
            if not entry in plots:
                plots[entry] = []
            plots[entry].append(float(sample["distance"]))

    ## plot
    figs = [plt.figure() for i in range(6)]
    axes = []
    for idx, fig in enumerate(figs):
        fig.suptitle("SF" + str(5 + idx), fontsize = 18)
        bw400 = fig.add_subplot(221)
        bw800 = fig.add_subplot(222)
        bw1600 = fig.add_subplot(223)
        axes.append([bw400, bw800, bw1600])

    for idx, entry in enumerate(plots):
        sf_idx = idx // 3;
        bw_idx = idx % 3;
        rssi_axe = [i - 150 for i in range(160)]
        axes[sf_idx][bw_idx].plot(rssi_axe, plots[entry], label = entry, color = "#2fa19e")
        axes[sf_idx][bw_idx].set_xlabel("Ranging RSSI (dBm)", fontsize = 12)
        axes[sf_idx][bw_idx].set_ylabel("Distance correction (m)", fontsize = 12)
        axes[sf_idx][bw_idx].legend(loc = 'lower right')
        axes[sf_idx][bw_idx].grid()
        axes[sf_idx][bw_idx].set_xticks(np.arange(-150, 0, 30))    

        fig.tight_layout(pad=0.4, w_pad=0.5, h_pad=1.0)
    # mng = plt.get_current_fig_manager()
    # mng.full_screen_toggle()
    for fig in figs:
        # fig.set_size_inches((9, 16), forward=False)
        fig.tight_layout()
        fig.savefig('Logs/CorrectionValues/' + fig._suptitle.get_text() + '.jpeg', dpi = 1200)

def rssi_calibration(fl):
    rssi = {}
    bins = [-20 + x * -10 for x in range(10)]
    dist = []
    for j in range (10):
        dist.append([])
  
    with open(fl) as f:
        chip = 'COM4'
        for line in f:
            sample = json.loads(line) 
            if sample["chip"] == chip:
                i = 0
                while float(sample["rssi"]) < bins[i]:
                    i += 1
                dist[i].append(float(sample["distance"]))
    y = []
    for d in dist:
        if d:
            y.append(np.mean(d))
        else:
            y.append(0)
    
    fig = plt.figure()
    axe = fig.add_subplot(111)
    print(bins)
    print(y)
    # axe.plot(bins, dist)
    
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
        elif sys.argv[1] == "-chronological":
            parse_log(sys.argv[2])
            linear_plot()
        elif sys.argv[1] == "-rssi_calibration":
            # parse_log(sys.argv[2])
            # plot_results("distance", "rssi")
            rssi_calibration('Logs/' + sys.argv[2])
       
        elif sys.argv[1] == "-dump_correction_values":
            f = "Logs\CorrectionValues\CorrectionValues.json"
            plot_default_correction_values(f)
    # plt.show()

        