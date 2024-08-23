import numpy as np

import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

class UKFEstimator:
    def __init__(self, dt, initial_state, process_noise, measurement_noise, motion_model, measurement_model):
        self.dt = dt
        self.motion_model = motion_model
        self.measurement_model = measurement_model
        self.sigma_points = MerweScaledSigmaPoints(n=len(initial_state), alpha=0.1, beta=2., kappa=0)
        self.ukf = UKF(dim_x=len(initial_state), dim_z=3, dt=self.dt, fx=self.motion_model, hx=self.measurement_model, points=self.sigma_points)
        
        # Initialer Zustand
        self.ukf.x = np.array(initial_state)
        
        # Initiale Kovarianzmatrix
        self.ukf.P *= 0.1
        
        # Prozessrauschen Q
        self.ukf.Q = process_noise
        
        # Messrauschen R
        self.ukf.R = measurement_noise
    
    def predict(self, dt):
        self.ukf.predict(dt=dt)
    
    def update(self, measurements):
        self.ukf.update(measurements)
    
    def get_state(self):
        return self.ukf.x

    def set_state(self, state):
        self.ukf.x = np.array(state)


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
