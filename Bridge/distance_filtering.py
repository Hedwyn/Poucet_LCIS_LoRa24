import numpy as np
import json
import random

ACC_LENGTH= 20
BASE_FREQ = 2400000000
CHANNEL_WIDTH = 2000000
ACCUMULATOR_LEN = 4

SNR_THRESHOLDS = {
    'sf5':7,
    'sf6':5,
    'sf7':2,
    'sf8':0,
    'sf9':-5,
    'sf10':-10
}

class Frame:
    def __init__(self, snr, rssi, dist):
        self.snr = snr
        self.rssi = rssi
        self.dist = dist

class ChannelPerformances:
    def __init__(self, snr, rssi, dist, std):
        self.snr = snr
        self.rssi = rssi
        self.dist = dist
        self.std = std

class Filter:
    def __init__(self):
        self.channels_data = {}
        self.prev_freq = None
        self.accumulator = []

    def compute_median(self):
        snr = np.median([f.snr for f in self.accumulator])  
        rssi = np.median([f.rssi for f in self.accumulator])   
        dist = np.median([f.dist for f in self.accumulator])  
        std = np.std([f.dist for f in self.accumulator])
        return(ChannelPerformances(snr, rssi, dist, std)) 

    def feed(self, log):
        data = json.loads(log)
        freq = str(data['frequency'])
        sf = data['sf']
        snr = data['snr']
        if freq != self.prev_freq:
            if self.accumulator:
                # computing previous freq result
                perf = self.compute_median() 
                median_dist = perf.dist
                self.channels_data[freq] = median_dist
                print("Distance for frequency " + str(median_dist))
                # emptying self.accumulator
                self.accumulator.clear()
        if snr > SNR_THRESHOLDS[sf]:
            self.accumulator.append(Frame(data['snr'], data['rssi'], data['corrected_distance']))
        self.prev_freq = freq

        def select_channel(self):
            min_std = 0
            min_idx = 0
            for idx, freq in enumerate(self.channels_data):
                if self.channels_data[freq].std < min_std:
                    min_dist = self.channels_data[freq].std
                    min_idx = idx
            return(min_dist)

        


def test_module():
    model = Filter()
    NB_FRAMES = 40
    test_logs = []
    print("*** Test random logs: ***")
    freq = BASE_FREQ + CHANNEL_WIDTH * random.randint(1, 40)   
    sf = 'sf' + str(random.randint(5, 10))

    for i in range(NB_FRAMES):
        channel_idx = random.randint(1, 40)  
        if i % ACCUMULATOR_LEN == 0:
            # changing frequency
            freq = BASE_FREQ + CHANNEL_WIDTH * channel_idx   
        dist = random.uniform(0, 10)
        rssi = random.uniform(-50, -70)
        snr = random.uniform(10, 18)
        dic = {"corrected_distance":dist, "snr":snr, "rssi":rssi, "frequency":freq, "sf":sf}
        log = json.dumps(dic)
        test_logs.append(log)
        print(log)

    # starting test
    for log in test_logs:
        model.feed(log)




if __name__ =="__main__":
    test_module()

        

