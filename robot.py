import time
import numpy as np
import random
import math
import pigpio

from filter import LowPassFilter
from filter import UKFEstimator

from ADCDifferentialPi import ADCDifferentialPi
from INA219 import INA219 
import motoron
import adafruit_icm20x
import board





class DifferentialDriveRobot:
    def __init__ (self, param_file = 'parameters.txt', mesurment_noise_mean = 0, mesurment_noise_standard_deviation = 1, system_noise_mean = 0, system_noise_standard_deviation = 1, 
                  init_robot_x = 0, init_robot_y = 0, init_robot_angle = 0, motion_model=None, measurement_model=None, process_noise=None, measurement_noise=None, dt=0.081):
        self.left_wheel_velocities = []  # Liste zur Speicherung der gemessenen Geschwindigkeiten
        self.right_wheel_velocities = []  # Liste zur Speicherung der gemessenen Geschwindigkeiten
        self.left_wheel_velocity_targets = []  # Liste zur Speicherung der Zielgeschwindigkeiten
        self.right_wheel_velocity_targets = []  # Liste zur Speicherung der Zielgeschwindigkeiten

        self.times = []  # Liste zur Speicherung der Zeitpunkte
        
        self.robot_radius = (114.1 / 2) / 1000 
        self.wheel_distance = 89 /1000  # Distance between wheels m
        self.wheel_diameter = 40 / 1000
        self.wheel_circumference =self.wheel_diameter * 3.141592653589793
        
        # Sensor angles (in radians)
        self.sensor_angles = [0, math.pi / 4, math.pi / 2, -math.pi / 4, -math.pi / 2]
        self.sensor_range = 40  # Max sensor range cm

        self.mesurment_noise_mean = mesurment_noise_mean
        self.mesurment_noise_standard_deviation = mesurment_noise_standard_deviation

        self.system_noise_mean = system_noise_mean
        self.system_noise_standard_deviation = system_noise_standard_deviation

        self.robot_x = init_robot_x
        self.robot_y = init_robot_y
        self.robot_angle = init_robot_angle
        self.robot_v = 0
        
        self.left_wheel_velocity = 0
        self.right_wheel_velocity = 0
        self.last_left_time = time.monotonic()
        self.last_right_time = time.monotonic()
        self.velocity_timeout = 0.17  # Sekundenschwelle, nach der die Geschwindigkeit auf 0 gesetzt wird


        self.lpf_sensors = [LowPassFilter(cutoff_freq=3) for _ in self.sensor_angles]
        self.lpf_speed = LowPassFilter(1)
        self.lpf_speed_right = LowPassFilter(1)


        self.pin_a_left = 17
        self.pin_b_left = 27
        self.pin_a_right = 21
        self.pin_b_right = 20
        
        self.ppr = 150 # Pulses Per Revolution ############
        self.counter_left = 0
        self.counter_right = 0
        self.last_time = time.time()

        self.pi = pigpio.pi()
        self.pi.set_mode(self.pin_a_left, pigpio.INPUT)
        self.pi.set_mode(self.pin_b_left, pigpio.INPUT)
        self.pi.set_mode(self.pin_a_right, pigpio.INPUT)
        self.pi.set_mode(self.pin_b_right, pigpio.INPUT)
        
        self.pi.set_pull_up_down(self.pin_a_left, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.pin_b_left, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.pin_a_right, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.pin_b_right, pigpio.PUD_UP)
           
        time.sleep(0.2)

        self.encoder_mode = 0
        # Interrupt on A pin
        if self.encoder_mode == 0:
            self.pi.callback(self.pin_a_left, pigpio.RISING_EDGE, self._update_velocity_left)
            self.pi.callback(self.pin_a_right, pigpio.RISING_EDGE, self._update_velocity_right)
        elif self.encoder_mode == 1:
            self.pi.callback(self.pin_a_left, pigpio.RISING_EDGE, self._update_count_left)
            self.pi.callback(self.pin_a_right, pigpio.RISING_EDGE, self._update_count_right)
        
        
        self.adc = ADCDifferentialPi(0x6E, 0x6F, 14)
        
        
        self.mc_left = motoron.MotoronI2C()
        self.mc_right = motoron.MotoronI2C(address=17)

        self.mc_left.reinitialize()  
        self.mc_left.disable_crc()
        self.mc_left.clear_reset_flag()

        self.mc_right.reinitialize()  
        self.mc_right.disable_crc()
        self.mc_right.clear_reset_flag()

        self.mc_left.set_max_acceleration(1, 500)
        self.mc_left.set_max_deceleration(1, 500)
        self.mc_left.set_starting_speed(1,10)

        self.mc_right.set_max_acceleration(1, 500)
        self.mc_right.set_max_deceleration(1, 500)
        self.mc_right.set_starting_speed(1,10)
        
        self.mc_left.set_braking(1,00)
        self.mc_right.set_braking(1,00)


        self.parameters = self.load_parameters(param_file)
        
        
        self.i2c = board.I2C()
        self.icm = adafruit_icm20x.ICM20948(self.i2c)
        self.gyro_w_bias = 0.00005
        
        
        # Initialisierung von UKF spezifischen Variablen
        initial_state = [init_robot_x, init_robot_y, init_robot_angle, 0, 0, 0, 0]
        process_noise = np.diag([0.0001, 0.0001, 0.001, 0.001, 0.01, 0.01, 0.001]) 
        measurement_noise = np.diag([0.1, 0.1, 0.015]) 
            
        if motion_model is None:
            motion_model = self.ukf_motion_model
        
        if measurement_model is None:
            measurement_model = self.ukf_measurement_model
            
        # UKF Initialisierung
        self.ukf_estimator = UKFEstimator(dt=dt,
                                          initial_state=[0,0,0,0,0,0,0], 
                                          process_noise=process_noise,
                                          measurement_noise=measurement_noise,
                                          motion_model=motion_model,
                                          measurement_model=measurement_model)
        
        self.k_robot_x = 0
        self.k_robot_y = 0
        self.k_robot_angle = 0
        self.k_robot_v = 0
        
        #self.akku = INA219()
        

    def convert_voltage_to_distance(self, voltage):
        a, b, c, d, e = self.parameters
        return (a * voltage**2 + b * voltage + c) / (d * voltage + e)
        
    def load_parameters(self, filename):
        with open(filename, 'r') as file:
            parameters = [float(line.strip()) for line in file]
        return parameters
        

    def get_robot_radius(self):
        return self.robot_radius

  
    def get_wheel_distance(self):
        return self.wheel_distance

  
    def get_sensor_angles(self):
        return self.sensor_angles

  
    def get_sensor_range(self):
        return self.sensor_range

  
    def get_left_wheel_velocity(self):
        return self.left_wheel_velocity

  
    def get_right_wheel_velocity(self):
        return self.right_wheel_velocity

  
    def get_sensor_readings(self):   
        sensor_readings = []
        for i in range(5):
            voltage = self.adc.read_voltage(i + 1)
            distance = self.convert_voltage_to_distance(voltage)
            sensor_readings.append(distance)
        return sensor_readings

  
    def get_formatted_sensor_readings(self, sensor_readings, decimal_places=2):
        #sensor_readings = self.get_sensor_readings()
        formatted_readings = [f"{value:.4f}".rjust(6) for value in sensor_readings]
        return formatted_readings    

  
    def update_robot(self, x, y, theta, left_wheel_velocity, right_wheel_velocity, gyro_w, time_step):
        """Update the robot's position and orientation based on wheel velocities."""
        v = (left_wheel_velocity + right_wheel_velocity) / 2
        omega = gyro_w
        new_x = x + v * math.cos(theta) * time_step
        new_y = y + v * math.sin(theta) * time_step
        new_theta = theta + omega * time_step
        
        x, y, theta = new_x, new_y, new_theta

        return x, y, theta, v
    
   

  
    def get_sensor_readings_with_noise(self, sensor_distances):
        sensor_readings = self.get_sensor_readings(sensor_distances)
        sensor_readings_with_noise = []
        for reading in sensor_readings:
            sensor_readings_with_noise.append(reading + np.random.normal(self.mesurment_noise_mean,self.mesurment_noise_standard_deviation))

        return sensor_readings_with_noise    

  
    def update_robot_with_noise(self, x, y, theta, left_wheel_velocity, right_wheel_velocity, time_step):
        
        left_wheel_velocity = left_wheel_velocity + np.random.normal(self.system_noise_mean,self.system_noise_standard_deviation)
        right_wheel_velocity = right_wheel_velocity + np.random.normal(self.system_noise_mean,self.system_noise_standard_deviation)
        
        x, y, theta = self.update_robot(x, y, theta, left_wheel_velocity, right_wheel_velocity, time_step)
        
        return x, y, theta

  
    def state_estimate(self, left_wheel_velocity, right_wheel_velocity, gyro_w, current_time, time_step): ######################################################################

        if self.encoder_mode == 0:
        # Geschwindigkeit auf 0 setzen, falls das Timeout überschritten wurde
            if time.monotonic() - self.last_left_time > self.velocity_timeout:
                self.left_wheel_velocity = 0
    
            if time.monotonic() - self.last_right_time > self.velocity_timeout:
                self.right_wheel_velocity = 0
        elif self.encoder_mode == 1:
            # Calculate RPS (Revolutions Per Second)
            rps_left = (self.counter_left / self.ppr) / time_step 
            rps_right = (self.counter_right / self.ppr) / time_step 
    
            # Reset counter and time
            self.counter_left = 0
            self.counter_right = 0

            left_wheel_velocity = (self.wheel_circumference * rps_left)   # in meters per second
            right_wheel_velocity = (self.wheel_circumference * rps_right) # in meters per second
            
            self.left_wheel_velocity = self.lpf_speed.filter(left_wheel_velocity, time_step)
            self.right_wheel_velocity = self.lpf_speed_right.filter(right_wheel_velocity, time_step)
            
        # Positions-Update basierend auf der aktuellen Geschwindigkeit
        
        

        # UKF Vorhersage
        self.ukf_estimator.predict(time_step)

        # Sensorwerte als Messungen
        z = np.array([self.left_wheel_velocity, self.right_wheel_velocity, gyro_w - self.gyro_w_bias])

        # UKF Update
        self.ukf_estimator.update(z)

        # Aktualisierung der Roboterzustände
        self.k_robot_x, self.k_robot_y, self.k_robot_angle, _, _, _, self.k_robot_v = self.ukf_estimator.get_state()

        
        self.robot_x, self.robot_y, self.robot_angle, self.robot_v = self.update_robot(self.robot_x, self.robot_y, self.robot_angle, self.left_wheel_velocity,
                                                                         self.right_wheel_velocity, gyro_w - self.gyro_w_bias, time_step)

        self.left_wheel_velocities.append(self.left_wheel_velocity)
        self.right_wheel_velocities.append(self.right_wheel_velocity)
        self.times.append(current_time)
        self.left_wheel_velocity_targets.append(left_wheel_velocity)
        self.right_wheel_velocity_targets.append(right_wheel_velocity)
  
    def get_position_and_angle(self):
        return self.robot_x, self.robot_y, self.robot_angle, self.robot_v

  
    def set_position_and_angle(self, x, y, theta):
        self.robot_x, self.robot_y, self.robot_angle = x, y, theta


    def get_ukf_position_and_angle(self):
        return self.k_robot_x, self.k_robot_y, self.k_robot_angle, self.k_robot_v


    def set_ukf_position_and_angle(self, x, y, theta):
        self.k_robot_x, self.k_robot_y, self.k_robot_angle, self.k_robot_v = x, y, theta, 0
        self.ukf_estimator.set_state([x, y, theta, 0, 0, 0, 0])
  
  
    def filter_sensor_readings(self, sensor_readings, time_step):
        filtered_readings = []
        for i, reading in enumerate(sensor_readings):
            filtered_reading = self.lpf_sensors[i].filter(reading, time_step)
            filtered_readings.append(filtered_reading)
        
        return filtered_readings 

  
    def _update_count_left(self, gpio, level, tick):
        #a_state = GPIO.input(self.pin_a_left)
        #b_state = GPIO.input(self.pin_b_left)
        if True:#b_state:
            self.counter_left += 1
        else:
            self.counter_left -= 1

  
    def _update_count_right(self, gpio, level, tick):
        #a_state = GPIO.input(self.pin_a_right)
        #b_state = GPIO.input(self.pin_b_right)
        if True:#b_state:
            self.counter_right += 1
        else:
            self.counter_right -= 1

  
    def _update_velocity_left(self, gpio, level, tick):
        current_time = time.monotonic()
        time_step = current_time - self.last_left_time
        if time_step > 0:
            # Bestimmen der Richtung
            direction = 1 
            left_wheel_velocity = self.wheel_circumference / (self.ppr * time_step)
            self.left_wheel_velocity = self.lpf_speed.filter(left_wheel_velocity, time_step)
            
        self.last_left_time = current_time
        

  
    def _update_velocity_right(self, gpio, level, tick):
        current_time = time.monotonic()
        time_step = current_time - self.last_right_time
        if time_step > 0:
            # Bestimmen der Richtung
            direction = 1 
            right_wheel_velocity = self.wheel_circumference / (self.ppr * time_step)
            self.right_wheel_velocity = self.lpf_speed_right.filter(right_wheel_velocity, time_step)
            
        self.last_right_time = current_time


    def set_right_motor(self,speed):
        self.mc_right.set_speed(1, speed)
        
        
    def set_left_motor(self,speed):
        self.mc_left.set_speed(1, speed)
        
        
    def get_imu_readings(self):
        return self.icm.gyro


    def ukf_motion_model(self, state, dt):
        x, y, theta, omega, vl, vr, v = state
        
        dx = v * math.cos(theta) * dt
        dy = v * math.sin(theta) * dt
        dtheta = omega * dt
        v = (vl + vr) / 2
        #print(state)
        return np.array([x + dx, y + dy, theta + dtheta, omega, vl, vr, v])
    
    
    def ukf_measurement_model(self, state):
        x, y, theta, omega, vl, vr, v = state
        #print(state)
        return np.array([vl, vr, omega])
    
    def get_power_percentage(self):
        bus_voltage = self.akku.getBusVoltage_V()             # voltage on V- (load side)
        p = (bus_voltage - 9)/3.6*100
        if(p > 100):p = 100
        if(p < 0):p = 0
        return p
        
        
