import numpy as np
import random
import math
from filter import LowPassFilter
from ADCDifferentialPi import ADCDifferentialPi
import RPi.GPIO as GPIO




class DifferentialDriveRobot:
    def __init__ (self, mesurment_noise_mean = 0, mesurment_noise_standard_deviation = 1, system_noise_mean = 0, system_noise_standard_deviation = 1, init_robot_x = 0, init_robot_y = 0, init_robot_angle = 0):
        self.left_wheel_velocities = []  # Liste zur Speicherung der gemessenen Geschwindigkeiten
        self.right_wheel_velocities = []  # Liste zur Speicherung der gemessenen Geschwindigkeiten
        self.left_wheel_velocity_targets = []  # Liste zur Speicherung der Zielgeschwindigkeiten
        self.right_wheel_velocity_targets = []  # Liste zur Speicherung der Zielgeschwindigkeiten

        self.times = []  # Liste zur Speicherung der Zeitpunkte
        
        self.robot_radius = 20
        self.wheel_distance = 89 /1000  # Distance between wheels
        self.wheel_diameter = 40 / 1000
        self.wheel_circumference =self.wheel_diameter * 3.141592653589793
        
        # Sensor angles (in radians)
        self.sensor_angles = [0, math.pi / 4, math.pi / 2, -math.pi / 4, -math.pi / 2]
        self.sensor_range = 200  # Max sensor range

        self.mesurment_noise_mean = mesurment_noise_mean
        self.mesurment_noise_standard_deviation = mesurment_noise_standard_deviation

        self.system_noise_mean = system_noise_mean
        self.system_noise_standard_deviation = system_noise_standard_deviation

        self.robot_x = init_robot_x
        self.robot_y = init_robot_y
        self.robot_angle = init_robot_angle
        
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
        
        self.ppr = 140 # Pulses Per Revolution ############
        self.counter_left = 0
        self.counter_right = 0
        self.last_time = time.time()
        
        GPIO.cleanup()

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a_left, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b_left, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_a_right, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b_right, GPIO.IN, pull_up_down=GPIO.PUD_UP)
       
        time.sleep(0.2)

        self.encoder_mode = 0
        # Interrupt on A pin
        if self.encoder_mode == 0:
            GPIO.add_event_detect(self.pin_a_left, GPIO.RISING, callback=self._update_velocity_left)
            GPIO.add_event_detect(self.pin_a_right, GPIO.RISING, callback=self._update_velocity_right)
        elif self.encoder_mode == 1:
            GPIO.add_event_detect(self.pin_a_left, GPIO.RISING, callback=self._update_count_left)
            GPIO.add_event_detect(self.pin_a_right, GPIO.RISING, callback=self._update_count_right)
            
        self.adc = ADCDifferentialPi(0x68, 0x69, 14)
        

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
            sensor_readings.append(self.adc.read_voltage(i+1))
        
        return sensor_readings

  
    def get_formatted_sensor_readings(self, decimal_places=2):
        sensor_readings = self.get_sensor_readings()
        formatted_readings = [f"{value:.4f}".rjust(6) for value in sensor_readings]
        return formatted_readings    

  
    def update_robot(self, x, y, theta, left_wheel_velocity, right_wheel_velocity, time_step):
        """Update the robot's position and orientation based on wheel velocities."""
        v = (left_wheel_velocity + right_wheel_velocity) / 2
        omega = (right_wheel_velocity - left_wheel_velocity) / self.wheel_distance
        new_x = x + v * math.cos(theta) * time_step
        new_y = y + v * math.sin(theta) * time_step
        new_theta = theta + omega * time_step
        
        x, y, theta = new_x, new_y, new_theta

        return x, y, theta

  
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

  
    def state_estimate(self, left_wheel_velocity, right_wheel_velocity): ######################################################################
        current_time = time.monotonic()
        time_step = current_time - self.last_time
        self.last_time = current_time

        if self.encoder_mode == 0:
        # Geschwindigkeit auf 0 setzen, falls das Timeout überschritten wurde
            if current_time - self.last_left_time > self.velocity_timeout:
                self.left_wheel_velocity = 0
    
            if current_time - self.last_right_time > self.velocity_timeout:
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
        
        self.robot_x, self.robot_y, self.robot_angle = self.update_robot(self.robot_x, self.robot_y, self.robot_angle, left_wheel_velocity, np.sign(right_wheel_velocity) * self.right_wheel_velocity, time_step)

        self.left_wheel_velocities.append(self.left_wheel_velocity)
        self.right_wheel_velocities.append(self.right_wheel_velocity)
        self.times.append(current_time)
        self.left_wheel_velocity_targets.append(left_wheel_velocity)
        self.right_wheel_velocity_targets.append(right_wheel_velocity)

  
    def get_position_and_angle(self):
        return self.robot_x, self.robot_y, self.robot_angle

  
    def set_position_and_angle(self, x, y, theta):
        self.robot_x, self.robot_y, self.robot_angle = x, y, theta

  
    def filter_sensor_readings(self, sensor_readings, time_step):
        filtered_readings = []
        for i, reading in enumerate(sensor_readings):
            filtered_reading = self.lpf_sensors[i].filter(reading, time_step)
            filtered_readings.append(filtered_reading)
        
        return filtered_readings 

  
    def _update_count_left(self, channel):
        #a_state = GPIO.input(self.pin_a_left)
        b_state = GPIO.input(self.pin_b_left)
        if True:#b_state:
            self.counter_left += 1
        else:
            self.counter_left -= 1

  
    def _update_count_right(self, channel):
        #a_state = GPIO.input(self.pin_a_right)
        b_state = GPIO.input(self.pin_b_right)
        if True:#b_state:
            self.counter_right += 1
        else:
            self.counter_right -= 1

  
    def _update_velocity_left(self, channel):
        current_time = time.monotonic()
        time_step = current_time - self.last_left_time
        if time_step > 0:
            # Bestimmen der Richtung
            direction = 1 
            left_wheel_velocity = self.wheel_circumference / (self.ppr * time_step)
            self.left_wheel_velocity = self.lpf_speed.filter(left_wheel_velocity, time_step)
            
        self.last_left_time = current_time

  
    def _update_velocity_right(self, channel):
        current_time = time.monotonic()
        time_step = current_time - self.last_right_time
        if time_step > 0:
            # Bestimmen der Richtung
            direction = 1 
            right_wheel_velocity = self.wheel_circumference / (self.ppr * time_step)
            self.right_wheel_velocity = self.lpf_speed_right.filter(right_wheel_velocity, time_step)
            
        self.last_right_time = current_time
