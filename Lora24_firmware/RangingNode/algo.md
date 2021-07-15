**On a rotation of N pre-selected channels (N <= 40)**
Init each channel with a probability of 1 / N
**for channel in channels**
    accumulate(K rangings) in accumlator
    compute(variance(accumulator))
    compute(snr(accumulator))
    compute(rssi(accumulator))
    update_probability(channel)

