import numpy as np

class LowPassFilter:
    def __init__(self, cutoff_freq):
        self.cutoff_freq = cutoff_freq
        self.prev_output = 0
    
    def filter(self, input_signal, time_step):
        alpha = time_step / (time_step + (1 / (2 * np.pi * self.cutoff_freq)))
        output_signal = alpha * input_signal + (1 - alpha) * self.prev_output
        self.prev_output = output_signal
        return output_signal
    
    def set_previous_output(self, output):
        self.prev_output = output
