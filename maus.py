import time
import motoron
import RPi.GPIO as GPIO
import math
import numpy as np

class Robot:
    def __init__ (self, mesurment_noise_mean = 0, mesurment_noise_standard_deviation = 1, system_noise_mean = 0, system_noise_standard_deviation = 1, init_robot_x = 0, init_robot_y = 0, init_robot_angle = 0):
        self.robot_radius = 20
        self.wheel_distance = 40  # Distance between wheels
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
        self.velocity_timeout = 0.2  # Sekundenschwelle, nach der die Geschwindigkeit auf 0 gesetzt wird

        #self.lpf_sensors = [LowPassFilter(cutoff_freq=3) for _ in self.sensor_angles]

        self.pin_a_left = 17
        self.pin_b_left = 27
        self.pin_a_right = self.pin_a_left
        self.pin_b_right = self.pin_b_left
        
        self.ppr = 12  # Pulses Per Revolution
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

        # Interrupt on A pin
        GPIO.add_event_detect(self.pin_a_left, GPIO.BOTH, callback=self._update_velocity_left)
        GPIO.add_event_detect(self.pin_a_right, GPIO.BOTH, callback=self._update_velocity_right)

        

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

    def get_sensor_readings(self,sensor_distances):   
        return sensor_distances

    def update_robot(self, x, y, theta, left_wheel_velocity, right_wheel_velocity, time_step):
        """Update the robot's position and orientation based on wheel velocities."""
        v = (left_wheel_velocity + right_wheel_velocity) / 2
        omega = (right_wheel_velocity - left_wheel_velocity) / self.wheel_distance
        new_x = x + v * math.cos(theta) * time_step
        new_y = y + v * math.sin(theta) * time_step
        new_theta = theta + omega * time_step
        
        # if not ev.check_collision(new_x, new_y):
        
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

    def state_estimate(self):
         current_time = time.monotonic()

        # Geschwindigkeit auf 0 setzen, falls das Timeout überschritten wurde
        if current_time - self.last_left_time > self.velocity_timeout:
            self.left_wheel_velocity = 0

        if current_time - self.last_right_time > self.velocity_timeout:
            self.right_wheel_velocity = 0

        # Positions-Update basierend auf der aktuellen Geschwindigkeit
        time_step = current_time - self.last_time
        self.robot_x, self.robot_y, self.robot_angle = self.update_robot(self.robot_x, self.robot_y, self.robot_angle, self.left_wheel_velocity, self.right_wheel_velocity, time_step)
        self.last_time = current_time

    def get_position_and_angle(self):
        return self.robot_x, self.robot_y, self.robot_angle

    def set_position_and_angle(self, x, y, theta):
        self.robot_x, self.robot_y, self.robot_angle = x, y, theta

    # def filter_sensor_readings(self, sensor_readings, time_step):
    #     filtered_readings = []
    #     for i, reading in enumerate(sensor_readings):
    #         filtered_reading = self.lpf_sensors[i].filter(reading, time_step)
    #         filtered_readings.append(filtered_reading)
        
    #     return filtered_readings 

    def _update_velocity_left(self, channel):
        current_time = time.monotonic()
        time_delta = current_time - self.last_left_time
        if time_delta > 0:
            self.left_wheel_velocity = self.wheel_circumference / (self.ppr * time_delta)
        self.last_left_time = current_time

    def _update_velocity_right(self, channel):
        current_time = time.monotonic()
        time_delta = current_time - self.last_right_time
        if time_delta > 0:
            self.right_wheel_velocity = self.wheel_circumference / (self.ppr * time_delta)
        self.last_right_time = current_time


robot = Robot()
mc = motoron.MotoronI2C()
#mc2 = motoron.MotoronI2C(address=18)
mc.reinitialize()  
mc.disable_crc()
mc.clear_reset_flag()

mc.set_max_acceleration(1, 140)
mc.set_max_deceleration(1, 300)


while True:
    mc.set_speed(1, 400)
    robot.state_estimate()
    print(robot.get_left_wheel_velocity())
    time.sleep(0.1)
