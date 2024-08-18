import numpy as np



class PIDController:

    def __init__(self, kp, ki, kd, p_max = np.inf, p_min = -np.inf, p_minmax = np.inf, pid_max = np.inf, pid_min = -np.inf, pid_minmax = np.inf,
                 i_max = np.inf, i_min = -np.inf, i_minmax = np.inf, d_max = np.inf, d_min = -np.inf, d_minmax = np.inf):
        
        self.P = 0
        self.I = 0
        self.D = 0
        self.PID = 0
        
        self.previous_error = 0 
        self.integral = 0
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.p_max = p_max
        self.p_min = p_min
        self.p_minmax = p_minmax
        
        self.i_max = i_max
        self.i_min = i_min
        self.i_minmax = i_minmax
        
        self.d_max = d_max
        self.d_min = d_min
        self.d_minmax = d_minmax
        
        self.pid_max = pid_max
        self.pid_min = pid_min
        self.pid_minmax = pid_minmax
        
        
    def update(self, setpoint, measurement, time_step):
        error = setpoint - measurement
        self.integral += error * time_step
        derivative = (error - self.previous_error) / time_step
        self.previous_error = error
        
        self.P = self.kp * error
        self.I = self.ki * self.integral
        self.D = self.kd * derivative
        
        
        if (self.P < self.p_min) or (self.P < -self.p_minmax):#-self.imax:
            if self.p_min >= -self.p_minmax:
                self.P = self.p_min
            else:
                self.P = -self.p_minmax
        elif (self.P > self.p_max) or (self.P > self.p_minmax):
            if self.p_max <= self.p_minmax:
                self.P = self.p_max
            else:
                self.P = self.p_minmax
        
        if self.ki != 0:
            if (self.I < self.i_min) or (self.I < -self.i_minmax):#-self.imax:
                if self.i_min >= -self.i_minmax:
                    self.I = self.i_min
                    self.integral = self.i_min / self.ki
                else:
                    self.I = -self.i_minmax
                    self.integral = -self.i_minmax / self.ki
            elif (self.I > self.i_max) or (self.I > self.i_minmax):
                if self.i_max <= self.i_minmax:
                    self.I = self.i_max
                    self.integral = self.i_max / self.ki
                else:
                    self.I = self.i_minmax
                    self.integral = self.i_minmax / self.ki
        
        
        if (self.D < self.d_min) or (self.D < -self.d_minmax):#-self.imax:
            if self.d_min >= -self.d_minmax:
                self.D = self.d_min
            else:
                self.D = -self.d_minmax
        elif (self.D > self.d_max) or (self.D > self.d_minmax):
            if self.d_max <= self.d_minmax:
                self.D = self.d_max
            else:
                self.D = self.d_minmax
                        
                
        self.PID = self.P + self.I + self.D
        
        
        if (self.PID < self.pid_min) or (self.PID < -self.pid_minmax):#-self.imax:
            if self.pid_min >= -self.pid_minmax:
                self.PID = self.pid_min
            else:
                self.PID = -self.pid_minmax
        elif (self.PID > self.pid_max) or (self.PID > self.pid_minmax):
            if self.PID_max <= self.pid_minmax:
                self.PID = self.pid_max
            else:
                self.PID = self.pid_minmax
        
        return self.PID
    
    def set_previous_error(self,error):
        self.previous_error = error
        
    def set_integral(self, integral):
        self.integral = integral
