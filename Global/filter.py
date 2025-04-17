import numpy as np

class Filter:
    def __init__(self, cutoff_frequency, sampling_rate):
        self.cutoff_frequency = cutoff_frequency
        self.sampling_rate = sampling_rate

    def low_pass_filter(self, signal):
        # Calculate the normalized cutoff frequency
        nyquist_rate = self.sampling_rate / 2
        normalized_cutoff = self.cutoff_frequency / nyquist_rate

        # Design a simple low-pass filter using a Hamming window
        num_taps = 101  # Number of filter coefficients
        taps = np.hamming(num_taps)
        sinc_filter = np.sinc(2 * normalized_cutoff * (np.arange(num_taps) - (num_taps - 1) / 2))
        low_pass = taps * sinc_filter
        low_pass /= np.sum(low_pass)  # Normalize the filter coefficients

        # Apply the filter to the signal using convolution
        filtered_signal = np.convolve(signal, low_pass, mode='same')
        return filtered_signal