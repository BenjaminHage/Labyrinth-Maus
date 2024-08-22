import pygame
import math
import numpy as np
import time
import random

#random.seed(0)
#######################################################################################################

#classes


class PIDController:

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0 #überlegung den setzbar zumachen falls der die probleme macht
        self.integral = 0

    def update(self, setpoint, measurement, time_step):
        error = setpoint - measurement
        self.integral += error * time_step
        derivative = (error - self.previous_error) / time_step
        derivative = max(min(2, derivative),-2)
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative
    
    def set_previous_error(self,error):
        self.previous_error = error


class HighPassFilter:
    def __init__(self, cutoff_freq):
        self.cutoff_freq = cutoff_freq
        self.prev_input = 0
        self.prev_output = 0

    def filter(self, input_signal, sampling_period):
        
        alpha = (2 * np.pi * self.cutoff_freq * sampling_period) / (1 + 2 * np.pi * self.cutoff_freq * sampling_period)
        output_signal = alpha * (self.prev_output + input_signal - self.prev_input)
        self.prev_input = input_signal
        self.prev_output = output_signal
        return output_signal
    
    def set_previous_input(self,input):
        self.prev_input = input


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


class ESCController:
    def __init__(self, dither_freq, dither_amp, learning_rate):
        self.dither_freq = dither_freq
        self.dither_amp = dither_amp
        self.learning_rate = learning_rate
        self.time = 0
        self.integrator = 0
        self.control_signal = 0
        self.hpf = HighPassFilter(dither_freq - 0.2)
        self.lpf = LowPassFilter(dither_freq + 0.2)
    
    def update(self, cost, time_step):
       
        #cost = self.lpf.filter(cost, time_step)
       
        # High-pass filter (HPF)
        high_pass_output = self.hpf.filter(cost, time_step)

        # Demodulation
        demodulated_signal = high_pass_output * np.sin(2 * np.pi * self.dither_freq * (self.time - time_step))
        
        # Update integrator
        self.integrator = self.integrator + demodulated_signal
        
        # Dither signal
        dither_signal = self.dither_amp * np.sin(2 * np.pi * self.dither_freq * self.time)

        # Control signal before LPF
        self.control_signal = self.learning_rate * self.integrator + dither_signal
        

        # Update previous cost and time
        self.previous_cost = cost
        self.time += time_step

        return self.control_signal 
    
    def set_previous_hpf_input(self, cost):
        self.hpf.set_previous_input(cost)


class AutonomousController:
    def __init__(self, angle_pid, wall_distance_pid, point_distance_pid, esc, base_speed, base_rotation_speed, desired_distance, sensor_activation_threshold, wheel_distance, sensor_angles, control_distance = 5, angle_toleranz = 0.01, distance_toleranz = 0.03, 
                 esc_angle_comparison_interval = 1, esc_angel_toleranz = 0.8,  feature_toleranz = 3, direkt_change_toleranz = 15):
        
        self.state = 0
        
        self.left_wheel_velocity = 0
        self.right_wheel_velocity = 0
        self.base_speed = base_speed
        self.base_rotation_speed = base_rotation_speed
        self.desired_distance = desired_distance
        self.control_distance = control_distance
        self.wheel_distance = wheel_distance
        self.sensor_angles = sensor_angles

        self.follow_sensor = []
        self.angle_setpoint = 0
        self.target_x = 0
        self.target_y = 0
        self.angle_toleranz = angle_toleranz
        self.distance_toleranz = distance_toleranz
        self.feature_toleranz = feature_toleranz 
        self.direkt_change_toleranz = direkt_change_toleranz
        self.activation_threshold = sensor_activation_threshold
        self.near_activation_threshold = 60 

        self.esc_angle_comparison_interval = esc_angle_comparison_interval
        self.esc_angel_toleranz = esc_angel_toleranz

        self.front = self.get_sensor_index(sensor_angles, "front")                 #1
        self.front_right = self.get_sensor_index(sensor_angles, "front_right")     #2
        self.right = self.get_sensor_index(sensor_angles, "right")                 #3
        self.front_left = self.get_sensor_index(sensor_angles, "front_left")       #4
        self.left = self.get_sensor_index(sensor_angles, "left")                   #5


        self.front_sensor_active = False
        self.left_sensor_active = False
        self.right_sensor_active = False
        self.front_left_sensor_active = False
        self.front_right_sensor_active = False

        self.prev_state = 0
        self.previous_time = pygame.time.get_ticks() / 1000.0  #überlegen zeit unabhängig von pygame machen
        self.previous_theta = np.inf
        self.feature_list = np.empty((0, 2))
        self.change_feature_list = np.empty((0, 2))
        self.prev_feature = []
        self.on_closed_Loop = False

        self.pledge_count = []

        self.angle_pid = angle_pid
        self.wall_distance_pid = wall_distance_pid
        self.point_distance_pid = point_distance_pid
        self.esc = esc

    def autonomous_control_right_hand(self, sensor_readings, x, y, theta, current_time, time_step):
        

        ############################# V1 ####################################

        # Read the side sensors
        front_sensor = sensor_readings[self.front]
        left_sensor = sensor_readings[self.left]
        front_left_sensor = sensor_readings[self.front_left]
        right_sensor = sensor_readings[self.right]
        front_right_sensor = sensor_readings[self.front_right]

        self.left_sensor_active, left_sensor_flanke = self.flanke_detektion(left_sensor, self.left_sensor_active, self.activation_threshold )
        self.right_sensor_active, right_sensor_flanke = self.flanke_detektion(right_sensor, self.right_sensor_active, self.activation_threshold )
        self.front_left_sensor_active, front_left_sensor_flanke = self.flanke_detektion(front_left_sensor, self.front_left_sensor_active, self.activation_threshold ) #
        self.front_right_sensor_active, front_right_sensor_flanke = self.flanke_detektion(front_right_sensor, self.front_right_sensor_active, self.activation_threshold ) #


        ############################# V1 ####################################


        if self.state == 0:
            if not all(x >= self.activation_threshold for x in sensor_readings):
                
                min_range_sensor = np.argmin(sensor_readings)
                if min_range_sensor == self.right:
                    self.prev_state = self.state
                    self.state = 3
                elif min_range_sensor == self.left:    
                    self.prev_state = self.state
                    self.state = 4
                elif min_range_sensor == self.front_left:
                    self.prev_state = self.state
                    self.state = 11
                elif min_range_sensor == self.front_right:
                    self.prev_state = self.state
                    self.state = 10 
                elif min_range_sensor == self.front:
                    self.prev_state = self.state
                    self.state = 12
            else:
                self.prev_state = self.state
                self.state = 15

        elif self.state == 1:
            if front_sensor <= self.desired_distance + self.control_distance:
                self.prev_state = self.state
                self.state = 9
            elif not self.front_left_sensor_active:
                self.prev_state = self.state
                self.state = 6

        elif self.state == 2:
            if front_sensor <= self.desired_distance + self.control_distance:
                self.prev_state = self.state
                self.state = 9
            elif not self.front_right_sensor_active:
                self.prev_state = self.state
                self.state = 6    

        elif self.state == 3:
            self.prev_state = self.state
            self.state = 5

        elif self.state == 4:
            self.prev_state = self.state
            self.state = 5

        elif self.state == 5:
            if self.angle_toleranz > abs(self.angle_setpoint - theta):
                if self.follow_sensor == []:
                    self.esc.set_previous_hpf_input(-front_sensor)
                    self.prev_state = self.state
                    self.state = 13
                elif self.follow_sensor == self.right and self.right_sensor_active:
                    self.prev_state = self.state
                    self.state = 2
                    error = self.desired_distance - right_sensor
                    self.wall_distance_pid.set_previous_error(error)    
                elif self.follow_sensor == self.left and self.left_sensor_active:
                    self.prev_state = self.state
                    self.state = 1
                    error = self.desired_distance - left_sensor
                    self.wall_distance_pid.set_previous_error(error)   
                else:
                    self.prev_state = self.state
                    self.state = 6                  

        elif self.state == 6:
            if (left_sensor_flanke and not self.left_sensor_active and self.follow_sensor == self.left) or \
                    (right_sensor_flanke and not self.right_sensor_active and self.follow_sensor == self.right):
                self.prev_state = self.state
                self.state = 7
            elif self.control_distance > abs(self.get_distance_to_point(x, y, self.target_x, self.target_y)) and not self.prev_state == 5 and\
                    (self.follow_sensor == self.right or self.follow_sensor == self.left):
                self.prev_state = self.state
                self.state = 8
            elif left_sensor <= self.near_activation_threshold and self.follow_sensor == self.left and self.prev_state != 7 and self.prev_state != 1:
                self.prev_state = self.state
                self.state = 1
                error = self.desired_distance - left_sensor
                self.wall_distance_pid.set_previous_error(error)   
            elif right_sensor <= self.near_activation_threshold and self.follow_sensor == self.right and self.prev_state != 7 and self.prev_state != 2:
                self.prev_state = self.state
                self.state = 2
                error = self.desired_distance - right_sensor
                self.wall_distance_pid.set_previous_error(error)     
            elif self.follow_sensor == [] and not all(x >= self.activation_threshold  for x in sensor_readings):
                self.prev_state = self.state
                self.state = 0
            elif self.follow_sensor == self.front and front_sensor <= (self.desired_distance + self.control_distance) :
                self.prev_state = self.state
                self.state = 9

        elif self.state == 7:
            self.prev_state = self.state
            self.state = 6

        elif self.state == 8:
            if self.distance_toleranz > self.get_distance_to_point(x, y, self.target_x, self.target_y):
                if self.follow_sensor == self.left: 
                    self.prev_state = self.state
                    self.state = 4
                elif self.follow_sensor == self.right:
                    self.prev_state = self.state
                    self.state = 3

        elif self.state == 9:
            if abs(self.desired_distance - front_sensor) < self.distance_toleranz:
                if self.follow_sensor == self.left:
                    self.prev_state = self.state
                    self.state = 3     
                elif self.follow_sensor == self.right:
                    self.prev_state = self.state
                    self.state = 4  
                elif self.follow_sensor == self.front and self.front_left_sensor_active:
                    self.prev_state = self.state
                    self.state = 4   
                elif self.follow_sensor == self.front and self.front_right_sensor_active:
                    self.prev_state = self.state
                    self.state = 3     
                elif self.follow_sensor == self.front:
                    self.prev_state = self.state
                    self.state = 11    

        elif self.state == 10:
            self.prev_state = self.state
            self.state = 5

        elif self.state == 11:
            self.prev_state = self.state
            self.state = 5  

        elif self.state == 12:
            self.prev_state = self.state
            self.state = 5       

        elif self.state == 13:
            
            if current_time - self.previous_time >= self.esc_angle_comparison_interval:
                
                if self.has_angle_changed_less_than_threshold(self.previous_theta, theta, self.esc_angel_toleranz):
                    self.prev_state = self.state
                    self.state = 6
                self.previous_theta = theta
                self.previous_time = current_time

        elif self.state == 14:
            if not all(x >= self.activation_threshold  for x in sensor_readings):
                self.prev_state = self.state
                self.state = 0
            elif theta >= self.angle_setpoint:
                self.prev_state = self.state
                self.state = 6    

        elif self.state == 15:
            self.prev_state = self.state
            self.state = 14


        ############################# V1 ####################################
        # Calculate control action for maintaining a constant distance to the wall

        if self.state == 0: #init
            self.left_wheel_velocity = 0
            self.right_wheel_velocity = 0
        
        elif self.state == 1: #linke wand folgen
            angle_control = self.wall_distance_pid.update(self.desired_distance, left_sensor, time_step)

            # Adjust wheel velocities
            self.left_wheel_velocity = self.base_speed + angle_control
            self.right_wheel_velocity = self.base_speed - angle_control
            self.follow_sensor = self.left

        elif self.state == 2: #rechte wand folgen
            angle_control = self.wall_distance_pid.update(self.desired_distance, right_sensor, time_step)

            # Adjust wheel velocities
            self.left_wheel_velocity = self.base_speed - angle_control
            self.right_wheel_velocity = self.base_speed + angle_control
            self.follow_sensor = self.right

        elif self.state == 3: #set up 90° recht drehen
            self.angle_setpoint = theta - math.radians(90)
            if self.follow_sensor == self.front:
                self.follow_sensor = self.left
            elif self.follow_sensor == self.left and self.prev_state == 9:
                self.check_features(x,y)

        elif self.state == 4: #set up 90° links drehen 
            self.angle_setpoint = theta + math.radians(90)
            if self.follow_sensor == self.front:
                self.follow_sensor = self.right
            elif self.follow_sensor == self.right and self.prev_state == 9:
                self.check_features(x,y)    

        elif self.state == 5: #geregelt drehen
            angle_control = self.angle_pid.update(self.angle_setpoint, theta, time_step)
            self.left_wheel_velocity = - angle_control
            self.right_wheel_velocity = angle_control

        elif self.state == 6: #ungeregelt gerade aus
            self.left_wheel_velocity = self.base_speed
            self.right_wheel_velocity = self.base_speed

        elif self.state == 7: #set up forwoard point
            self.target_x, self.target_y = self.get_forward_point(x, y, theta, self.desired_distance)
            self.check_features(x,y)

        elif self.state == 8: #regelung zum punkt
            distance_control = self.point_distance_pid.update(0, self.get_distance_to_point(x, y, self.target_x, self.target_y), time_step)
            self.left_wheel_velocity = distance_control
            self.right_wheel_velocity = distance_control

        elif self.state == 9: #regelung wandabstand forne
            distance_control = self.point_distance_pid.update(self.desired_distance, front_sensor, time_step)
            self.left_wheel_velocity = distance_control
            self.right_wheel_velocity = distance_control  

        elif self.state == 10: #set up 45° recht drehen
            self.angle_setpoint = theta - math.radians(45)

        elif self.state == 11: #set up 45° links drehen 
            self.angle_setpoint = theta + math.radians(45)
            if self.follow_sensor == self.front:
                self.follow_sensor = self.right 

        elif self.state == 12: #set up 0° links drehen 
            self.angle_setpoint = theta     

        elif self.state == 13: #esc front
            # Update the control input using ESC
            control_input = self.esc.update(-front_sensor, time_step)

            # Apply the control input to the robot's rotation
            angle_control = self.angle_pid.update(self.angle_setpoint + control_input, theta, time_step)
            self.left_wheel_velocity = -angle_control
            self.right_wheel_velocity = angle_control

            self.follow_sensor = self.front
        
        elif self.state == 14: #ungeregelt drehen
            self.left_wheel_velocity = -(self.wheel_distance / 2.0) * self.base_rotation_speed
            self.right_wheel_velocity = (self.wheel_distance / 2.0) * self.base_rotation_speed

        elif self.state == 15: #set up 360° recht drehen
            self.angle_setpoint = theta + math.radians(360)    

       
        ############################# V1 ####################################

        return self.left_wheel_velocity, self.right_wheel_velocity
    
    def flanke_detektion(self, sensor_value, prev_value, threshold = 151):
        if sensor_value >= threshold:
            sensor_aktive = False
        else:
            sensor_aktive = True

        if sensor_aktive is not prev_value:
            sensor_flanke = True
        else:
            sensor_flanke = False
    
        return sensor_aktive, sensor_flanke

    def get_distance_to_point(self, x, y, target_x, target_y):
        """
        Berechnet den Abstand zwischen dem Roboter und einem gegebenen Punkt.

        :param x1: x-Koordinate des Roboters
        :param y1: y-Koordinate des Roboters
        :param x2: x-Koordinate des Zielpunkts
        :param y2: y-Koordinate des Zielpunkts
        :return: Abstand zwischen den beiden Punkten
        """
        distance = math.sqrt((target_x - x) ** 2 + (target_y - y) ** 2)
        return distance

    def get_forward_point(self, x, y, theta, distance):
        """
        Berechnet die Koordinaten eines Punktes, der sich in einer bestimmten Distanz
        in Fahrtrichtung des Roboters befindet.

        :param x: aktuelle x-Koordinate des Roboters
        :param y: aktuelle y-Koordinate des Roboters
        :param theta: aktuelle Orientierung des Roboters (in Radiant)
        :param distance: Entfernung zum Zielpunkt in Fahrtrichtung
        :return: (neue_x, neue_y) Koordinaten des Zielpunkts
        """
        new_x = x + distance * math.cos(theta)
        new_y = y + distance * math.sin(theta)
        return new_x, new_y

    def has_angle_changed_less_than_threshold(self, previous_theta, current_theta, esc_angel_toleranz):
        angle_change = abs(current_theta - previous_theta) * (180 / math.pi)
        return angle_change < esc_angel_toleranz

    def reset(self):
        self.state = 0
        self.prev_state = 0
        self.follow_sensor = []
        self.previous_theta = np.inf
        self.feature_list = np.empty((0, 2))
        self.change_feature_list = np.empty((0, 2))
        self.prev_feature = []
        self.on_closed_Loop = False
        self.pledge_count = []

    def set_base_speed(self, base_speed):
        self.base_speed = base_speed

    def check_features(self,x,y):
        
        if all(self.feature_toleranz < self.get_distance_to_point(x, y, feature[0], feature[1]) for feature in self.feature_list):
            # Koordinaten als 2D-Array hinzufügen / das aktuelle feature ist kein zuvor besuchtes feature
            self.feature_list = np.append(self.feature_list, [[x, y]], axis=0)
        else:
            self.on_closed_Loop = True

        self.prev_feature = [x , y]    

    def check_change_features(self):
        
        if all(self.feature_toleranz < self.get_distance_to_point(self.prev_feature[0], self.prev_feature[1], feature[0], feature[1]) for feature in self.change_feature_list):
            # Koordinaten als 2D-Array hinzufügen / das aktuelle feature ist kein zuvor besuchtes feature
            self.change_feature_list = np.append(self.change_feature_list, [self.prev_feature], axis=0)

            return True
        else:
            return False
    
    def get_base_speed(self):
        return self.base_speed
    
    def get_state(self):
        return self.state
        
    def get_follow_sensor(self):
        return self.follow_sensor
    
    def get_on_closed_loop(self):
        return self.on_closed_Loop

    def autonomous_control_V2(self, sensor_readings, x, y, theta, current_time, time_step):
        
        ############################# V2 ####################################

        # Read the side sensors
        front_sensor = sensor_readings[self.front]
        left_sensor = sensor_readings[self.left]
        front_left_sensor = sensor_readings[self.front_left]
        right_sensor = sensor_readings[self.right]
        front_right_sensor = sensor_readings[self.front_right]

        self.front_sensor_active, front_sensor_flanke = self.flanke_detektion(front_sensor, self.front_sensor_active, self.activation_threshold )
        self.left_sensor_active, left_sensor_flanke = self.flanke_detektion(left_sensor, self.left_sensor_active, self.activation_threshold )
        self.right_sensor_active, right_sensor_flanke = self.flanke_detektion(right_sensor, self.right_sensor_active, self.activation_threshold )
        self.front_left_sensor_active, front_left_sensor_flanke = self.flanke_detektion(front_left_sensor, self.front_left_sensor_active, self.activation_threshold )
        self.front_right_sensor_active, front_right_sensor_flanke = self.flanke_detektion(front_right_sensor, self.front_right_sensor_active, self.activation_threshold )

        ############################# V2 ####################################

        if self.state == 0:
            if not all(x >= self.activation_threshold for x in sensor_readings):
                
                min_range_sensor = np.argmin(sensor_readings)
                if min_range_sensor == self.right:
                    self.prev_state = self.state
                    self.state = 3
                elif min_range_sensor == self.left:    
                    self.prev_state = self.state
                    self.state = 4
                elif min_range_sensor == self.front_left:
                    self.prev_state = self.state
                    self.state = 11
                elif min_range_sensor == self.front_right:
                    self.prev_state = self.state
                    self.state = 10 
                elif min_range_sensor == self.front:
                    self.prev_state = self.state
                    self.state = 12
            else:
                self.prev_state = self.state
                self.state = 15

        elif self.state == 1:
            if front_sensor <= self.desired_distance + self.control_distance:
                self.prev_state = self.state
                self.state = 9
            elif not self.front_left_sensor_active:
                self.prev_state = self.state
                self.state = 6
            elif self.on_closed_Loop and self.right_sensor_active:                
                if self.check_change_features():
                    if abs(right_sensor - self.desired_distance) < self.direkt_change_toleranz :
                        self.state = 17  
                    else:     
                        self.state = 3

        elif self.state == 2:
            if front_sensor <= self.desired_distance + self.control_distance:
                self.prev_state = self.state
                self.state = 9
            elif not self.front_right_sensor_active:
                self.prev_state = self.state
                self.state = 6  
            elif self.on_closed_Loop and self.left_sensor_active:
                if self.check_change_features():
                    if abs(left_sensor - self.desired_distance) < self.direkt_change_toleranz: 
                        self.state = 16  
                    else:
                        self.state = 4

        elif self.state == 3:
            self.prev_state = self.state
            self.state = 5

        elif self.state == 4:
            self.prev_state = self.state
            self.state = 5

        elif self.state == 5:
            if self.angle_toleranz > abs(self.angle_setpoint - theta):
                if self.follow_sensor == []:
                    self.esc.set_previous_hpf_input(-front_sensor)
                    self.prev_state = self.state
                    self.state = 13 
                elif self.follow_sensor == self.right and self.right_sensor_active:
                    self.prev_state = self.state
                    self.state = 19
                elif self.follow_sensor == self.left and self.left_sensor_active:
                    self.prev_state = self.state
                    self.state = 18
                else:
                    self.prev_state = self.state
                    self.state = 6                  

        elif self.state == 6:
            if (left_sensor_flanke and not self.left_sensor_active and self.follow_sensor == self.left) or \
                    (right_sensor_flanke and not self.right_sensor_active and self.follow_sensor == self.right):
                self.prev_state = self.state
                self.state = 7
            elif self.control_distance > abs(self.get_distance_to_point(x, y, self.target_x, self.target_y)) and not self.prev_state == 5 and\
                    (self.follow_sensor == self.right or self.follow_sensor == self.left):
                self.prev_state = self.state
                self.state = 8 
            elif self.front_sensor_active and self.on_closed_Loop and not self.prev_state == 5 and\
                    (self.follow_sensor == self.right or self.follow_sensor == self.left):
                if self.check_change_features():
                    self.prev_state = self.state
                    self.esc.set_previous_hpf_input(-front_sensor)
                    self.state = 13   
            elif left_sensor <= self.near_activation_threshold and self.follow_sensor == self.left and self.prev_state != 7 and self.prev_state != 1:
                self.prev_state = self.state
                self.state = 18
            elif right_sensor <= self.near_activation_threshold and self.follow_sensor == self.right and self.prev_state != 7 and self.prev_state != 2:
                self.prev_state = self.state
                self.state = 19       
            elif self.follow_sensor == [] and not all(x >= self.activation_threshold  for x in sensor_readings):
                self.prev_state = self.state
                self.state = 0
            elif self.follow_sensor == self.front and front_sensor <= (self.desired_distance + self.control_distance) :
                self.prev_state = self.state
                self.state = 9

        elif self.state == 7:
            self.prev_state = self.state
            self.state = 6

        elif self.state == 8:
            if self.distance_toleranz > self.get_distance_to_point(x, y, self.target_x, self.target_y):
                if self.follow_sensor == self.left: 
                    self.prev_state = self.state
                    self.state = 4
                elif self.follow_sensor == self.right:
                    self.prev_state = self.state
                    self.state = 3

        elif self.state == 9:
            if abs(self.desired_distance - front_sensor) < self.distance_toleranz:
                if self.follow_sensor == self.left:
                    self.prev_state = self.state
                    self.state = 3     
                elif self.follow_sensor == self.right:
                    self.prev_state = self.state
                    self.state = 4  
                elif self.follow_sensor == self.front and self.front_left_sensor_active:
                    self.prev_state = self.state
                    self.state = 4   
                elif self.follow_sensor == self.front and self.front_right_sensor_active:
                    self.prev_state = self.state
                    self.state = 3     
                elif self.follow_sensor == self.front:
                    self.prev_state = self.state
                    self.state = 11    

        elif self.state == 10:
            self.prev_state = self.state
            self.state = 5

        elif self.state == 11:
            self.prev_state = self.state
            self.state = 5  

        elif self.state == 12:
            self.prev_state = self.state
            self.state = 5       

        elif self.state == 13:
            
            if current_time - self.previous_time >= self.esc_angle_comparison_interval:
                
                if self.has_angle_changed_less_than_threshold(self.previous_theta, theta, self.esc_angel_toleranz):
                    self.prev_state = self.state
                    self.state = 6
                self.previous_theta = theta
                self.previous_time = current_time

        elif self.state == 14:
            if not all(x >= self.activation_threshold  for x in sensor_readings):
                self.prev_state = self.state
                self.state = 0
            elif theta >= self.angle_setpoint:
                self.prev_state = self.state
                self.state = 6    

        elif self.state == 15:
            self.prev_state = self.state
            self.state = 14

        elif self.state == 16:
            self.state = 1  

        elif self.state == 17:
            self.state = 2 

        elif self.state == 18:
            self.state = 1  

        elif self.state == 19:
            self.state = 2 


        ############################# V2 ####################################

        # Calculate control action for maintaining a constant distance to the wall

        if self.state == 0: #init
            self.left_wheel_velocity = 0
            self.right_wheel_velocity = 0
        
        elif self.state == 1: #linke wand folgen
            angle_control = self.wall_distance_pid.update(self.desired_distance, left_sensor, time_step)

            # Adjust wheel velocities
            self.left_wheel_velocity = self.base_speed + angle_control
            self.right_wheel_velocity = self.base_speed - angle_control
            self.follow_sensor = self.left

        elif self.state == 2: #rechte wand folgen
            angle_control = self.wall_distance_pid.update(self.desired_distance, right_sensor, time_step)

            # Adjust wheel velocities
            self.left_wheel_velocity = self.base_speed - angle_control
            self.right_wheel_velocity = self.base_speed + angle_control
            self.follow_sensor = self.right

        elif self.state == 3: #set up 90° recht drehen
            self.angle_setpoint = theta - math.radians(90)
            if self.follow_sensor == self.front:
                self.follow_sensor = self.left
                self.on_closed_Loop = False
            elif self.follow_sensor == self.left and self.prev_state == 9:
                #self.check_features(x,y)
                pass
            elif self.right_sensor_active and self.on_closed_Loop and self.follow_sensor == self.left:
                self.follow_sensor = [] # müsste hier auch front eintragen können damit beim wechsel state 6 statt 13 machen und somit esc überspringen. Dann aber auch im state 6 eine wechsel auf front einbauen für den front sensor wechsel 
                self.on_closed_Loop = False
            if self.prev_state != 0:
                self.check_features(x,y)

        elif self.state == 4: #set up 90° links drehen 
            self.angle_setpoint = theta + math.radians(90)
            if self.follow_sensor == self.front:
                self.follow_sensor = self.right
                self.on_closed_Loop = False
            elif self.follow_sensor == self.right and self.prev_state == 9:
                #self.check_features(x,y)   
                pass
            elif self.left_sensor_active and self.on_closed_Loop and self.follow_sensor == self.right:
                self.follow_sensor = [] 
                self.on_closed_Loop = False

            if self.prev_state != 0:
                self.check_features(x,y)

        elif self.state == 5: #geregelt drehen
            angle_control = self.angle_pid.update(self.angle_setpoint, theta, time_step)
            self.left_wheel_velocity = - angle_control
            self.right_wheel_velocity = angle_control

        elif self.state == 6: #ungeregelt gerade aus
            self.left_wheel_velocity = self.base_speed
            self.right_wheel_velocity = self.base_speed

        elif self.state == 7: #set up forwoard point
            self.target_x, self.target_y = self.get_forward_point(x, y, theta, self.desired_distance)
            self.check_features(x,y)

        elif self.state == 8: #regelung zum punkt
            distance_control = self.point_distance_pid.update(0, self.get_distance_to_point(x, y, self.target_x, self.target_y), time_step)
            self.left_wheel_velocity = distance_control
            self.right_wheel_velocity = distance_control

        elif self.state == 9: #regelung wandabstand forne
            distance_control = self.point_distance_pid.update(self.desired_distance, front_sensor, time_step)
            self.left_wheel_velocity = distance_control
            self.right_wheel_velocity = distance_control  

        elif self.state == 10: #set up 45° recht drehen
            self.angle_setpoint = theta - math.radians(45)

        elif self.state == 11: #set up 45° links drehen 
            self.angle_setpoint = theta + math.radians(45)
            if self.follow_sensor == self.front:
                self.follow_sensor = self.right 

        elif self.state == 12: #set up 0° links drehen 
            self.angle_setpoint = theta     

        elif self.state == 13: #esc front
            # Update the control input using ESC
            control_input = self.esc.update(-front_sensor, time_step)

            # Apply the control input to the robot's rotation
            angle_control = self.angle_pid.update(self.angle_setpoint + control_input, theta, time_step)
            self.left_wheel_velocity = -angle_control
            self.right_wheel_velocity = angle_control

            self.follow_sensor = self.front
        
        elif self.state == 14: #ungeregelt drehen
            self.left_wheel_velocity = -(self.wheel_distance / 2.0) * self.base_rotation_speed
            self.right_wheel_velocity = (self.wheel_distance / 2.0) * self.base_rotation_speed

        elif self.state == 15: #set up 360° recht drehen
            self.angle_setpoint = theta + math.radians(360)    

        elif self.state == 16: #direktes wechseln nach links
            self.follow_sensor = self.left
            self.on_closed_Loop = False    

        elif self.state == 17: #direktes wechseln nach rechts
            self.follow_sensor = self.right
            self.on_closed_Loop = False    

        elif self.state == 18: #setup pid für linke wand folgen
 
            error = self.desired_distance - left_sensor
            self.wall_distance_pid.set_previous_error(error)    

        elif self.state == 19: #setup pid für rechte wand folgen
 
            error = self.desired_distance - right_sensor
            self.wall_distance_pid.set_previous_error(error)  

        ############################# V2 ####################################

        return self.left_wheel_velocity, self.right_wheel_velocity
    
    def get_set_angle_set_point(self):
        return self.angle_setpoint

    def autonomous_control_pledge(self, sensor_readings, x, y, theta, current_time, time_step):
        
        ############################# pledge ####################################

        # Read the side sensors
        front_sensor = sensor_readings[self.front]
        left_sensor = sensor_readings[self.left]
        front_left_sensor = sensor_readings[self.front_left]
        right_sensor = sensor_readings[self.right]
        front_right_sensor = sensor_readings[self.front_right]

        self.front_sensor_active, front_sensor_flanke = self.flanke_detektion(front_sensor, self.front_sensor_active, self.activation_threshold )
        self.left_sensor_active, left_sensor_flanke = self.flanke_detektion(left_sensor, self.left_sensor_active, self.activation_threshold )
        self.right_sensor_active, right_sensor_flanke = self.flanke_detektion(right_sensor, self.right_sensor_active, self.activation_threshold )
        self.front_left_sensor_active, front_left_sensor_flanke = self.flanke_detektion(front_left_sensor, self.front_left_sensor_active, self.activation_threshold )
        self.front_right_sensor_active, front_right_sensor_flanke = self.flanke_detektion(front_right_sensor, self.front_right_sensor_active, self.activation_threshold )

        ############################# pledge ####################################

        if self.state == 0:
            if not all(x >= self.activation_threshold for x in sensor_readings):
                
                min_range_sensor = np.argmin(sensor_readings)
                if min_range_sensor == self.right:
                    self.prev_state = self.state
                    self.state = 3
                elif min_range_sensor == self.left:    
                    self.prev_state = self.state
                    self.state = 4
                elif min_range_sensor == self.front_left:
                    self.prev_state = self.state
                    self.state = 11
                elif min_range_sensor == self.front_right:
                    self.prev_state = self.state
                    self.state = 10 
                elif min_range_sensor == self.front:
                    self.prev_state = self.state
                    self.state = 12
            else:
                self.prev_state = self.state
                self.state = 15

        elif self.state == 1:
            if front_sensor <= self.desired_distance + self.control_distance:
                self.prev_state = self.state
                self.state = 9
            elif not self.front_left_sensor_active:
                self.prev_state = self.state
                self.state = 6

        elif self.state == 2:
            if front_sensor <= self.desired_distance + self.control_distance:
                self.prev_state = self.state
                self.state = 9
            elif not self.front_right_sensor_active:
                self.prev_state = self.state
                self.state = 6  

        elif self.state == 3:
            self.prev_state = self.state
            self.state = 5

        elif self.state == 4:
            self.prev_state = self.state
            self.state = 5

        elif self.state == 5:
            if self.angle_toleranz > abs(self.angle_setpoint - theta):
                if self.follow_sensor == []:
                    self.esc.set_previous_hpf_input(-front_sensor)
                    self.prev_state = self.state
                    self.state = 13 
                elif self.follow_sensor == self.right and self.right_sensor_active:
                    self.prev_state = self.state
                    self.state = 19
                elif self.follow_sensor == self.left and self.left_sensor_active:
                    self.prev_state = self.state
                    self.state = 18
                else:
                    self.prev_state = self.state
                    self.state = 6                  

        elif self.state == 6:
            if ((left_sensor_flanke and not self.left_sensor_active and self.follow_sensor == self.left) or \
                    (right_sensor_flanke and not self.right_sensor_active and self.follow_sensor == self.right)) and self.pledge_count != 0:
                self.prev_state = self.state
                self.state = 7
            elif self.control_distance > abs(self.get_distance_to_point(x, y, self.target_x, self.target_y)) and self.prev_state == 7 and\
                    (self.follow_sensor == self.right or self.follow_sensor == self.left):
                self.prev_state = self.state
                self.state = 8 
            elif self.front_sensor_active and self.pledge_count == 0 and self.prev_state != 13:
                self.prev_state = self.state
                self.esc.set_previous_hpf_input(-front_sensor)
                self.state = 13   
            elif left_sensor <= self.near_activation_threshold and self.follow_sensor == self.left and self.prev_state != 7 and self.prev_state != 1:
                self.prev_state = self.state
                self.state = 18
            elif right_sensor <= self.near_activation_threshold and self.follow_sensor == self.right and self.prev_state != 7 and self.prev_state != 2:
                self.prev_state = self.state
                self.state = 19       
            elif self.follow_sensor == [] and not all(x >= self.activation_threshold  for x in sensor_readings) and self.pledge_count != 0: 
                self.prev_state = self.state
                self.state = 0
            elif self.follow_sensor == self.front and front_sensor <= (self.desired_distance + self.control_distance) :
                self.prev_state = self.state
                self.state = 9

        elif self.state == 7:
            self.prev_state = self.state
            self.state = 6

        elif self.state == 8:
            if self.distance_toleranz > self.get_distance_to_point(x, y, self.target_x, self.target_y):
                if self.follow_sensor == self.left: 
                    self.prev_state = self.state
                    self.state = 4
                elif self.follow_sensor == self.right:
                    self.prev_state = self.state
                    self.state = 3

        elif self.state == 9:
            if abs(self.desired_distance - front_sensor) < self.distance_toleranz:
                if self.follow_sensor == self.left:
                    self.prev_state = self.state
                    self.state = 3     
                elif self.follow_sensor == self.right:
                    self.prev_state = self.state
                    self.state = 4  
                elif self.follow_sensor == self.front and self.front_left_sensor_active: 
                    self.prev_state = self.state
                    self.state = 4   
                elif self.follow_sensor == self.front and self.front_right_sensor_active:
                    self.prev_state = self.state
                    self.state = 3     
                elif self.follow_sensor == self.front:
                    self.prev_state = self.state
                    self.state = 11    

        elif self.state == 10:
            self.prev_state = self.state
            self.state = 5

        elif self.state == 11:
            self.prev_state = self.state
            self.state = 5  

        elif self.state == 12:
            self.prev_state = self.state
            self.state = 5       

        elif self.state == 13:
            
            if current_time - self.previous_time >= self.esc_angle_comparison_interval:
                
                if self.has_angle_changed_less_than_threshold(self.previous_theta, theta, self.esc_angel_toleranz):
                    self.prev_state = self.state
                    self.state = 6
                self.previous_theta = theta
                self.previous_time = current_time

        elif self.state == 14:
            if not all(x >= self.activation_threshold  for x in sensor_readings):
                self.prev_state = self.state
                self.state = 0
            elif theta >= self.angle_setpoint:
                self.prev_state = self.state
                self.state = 6    

        elif self.state == 15:
            self.prev_state = self.state
            self.state = 14

        elif self.state == 16:
            pass

        elif self.state == 17:
            pass

        elif self.state == 18:
            self.state = 1  

        elif self.state == 19:
            self.state = 2 


        ############################# pledge ####################################

        # Calculate control action for maintaining a constant distance to the wall

        if self.state == 0: #init
            self.left_wheel_velocity = 0
            self.right_wheel_velocity = 0
        
        elif self.state == 1: #linke wand folgen
            angle_control = self.wall_distance_pid.update(self.desired_distance, left_sensor, time_step)

            # Adjust wheel velocities
            self.left_wheel_velocity = self.base_speed + angle_control
            self.right_wheel_velocity = self.base_speed - angle_control
            self.follow_sensor = self.left

        elif self.state == 2: #rechte wand folgen
            angle_control = self.wall_distance_pid.update(self.desired_distance, right_sensor, time_step)

            # Adjust wheel velocities
            self.left_wheel_velocity = self.base_speed - angle_control
            self.right_wheel_velocity = self.base_speed + angle_control
            self.follow_sensor = self.right

        elif self.state == 3: #set up 90° recht drehen
            self.angle_setpoint = theta - math.radians(90)
            if self.follow_sensor == self.front:
                self.follow_sensor = self.left
                self.on_closed_Loop = False

            if self.pledge_count != []:
                self.pledge_count -= 1    

        elif self.state == 4: #set up 90° links drehen 
            self.angle_setpoint = theta + math.radians(90)
            if self.follow_sensor == self.front:
                self.follow_sensor = self.right
                self.on_closed_Loop = False
            
            if self.pledge_count != []:
                self.pledge_count += 1

        elif self.state == 5: #geregelt drehen
            angle_control = self.angle_pid.update(self.angle_setpoint, theta, time_step)
            self.left_wheel_velocity = - angle_control
            self.right_wheel_velocity = angle_control

        elif self.state == 6: #ungeregelt gerade aus
            self.left_wheel_velocity = self.base_speed
            self.right_wheel_velocity = self.base_speed

            if self.pledge_count == 0:
                self.follow_sensor = self.front

        elif self.state == 7: #set up forwoard point
            self.target_x, self.target_y = self.get_forward_point(x, y, theta, self.desired_distance)

        elif self.state == 8: #regelung zum punkt
            distance_control = self.point_distance_pid.update(0, self.get_distance_to_point(x, y, self.target_x, self.target_y), time_step)
            self.left_wheel_velocity = distance_control
            self.right_wheel_velocity = distance_control

        elif self.state == 9: #regelung wandabstand forne
            distance_control = self.point_distance_pid.update(self.desired_distance, front_sensor, time_step)
            self.left_wheel_velocity = distance_control
            self.right_wheel_velocity = distance_control  

        elif self.state == 10: #set up 45° recht drehen
            self.angle_setpoint = theta - math.radians(45)

        elif self.state == 11: #set up 45° links drehen 
            self.angle_setpoint = theta + math.radians(45)
            if self.follow_sensor == self.front:
                self.follow_sensor = self.right 

        elif self.state == 12: #set up 0° links drehen 
            self.angle_setpoint = theta     

        elif self.state == 13: #esc front
            # Update the control input using ESC
            control_input = self.esc.update(-front_sensor, time_step)

            # Apply the control input to the robot's rotation
            angle_control = self.angle_pid.update(self.angle_setpoint + control_input, theta, time_step)
            self.left_wheel_velocity = -angle_control
            self.right_wheel_velocity = angle_control

            self.follow_sensor = self.front
        
        elif self.state == 14: #ungeregelt drehen
            self.left_wheel_velocity = -(self.wheel_distance / 2.0) * self.base_rotation_speed
            self.right_wheel_velocity = (self.wheel_distance / 2.0) * self.base_rotation_speed

        elif self.state == 15: #set up 360° recht drehen
            self.angle_setpoint = theta + math.radians(360)    

        elif self.state == 16: #direktes wechseln nach links
            self.follow_sensor = self.left     

        elif self.state == 17: #direktes wechseln nach rechts
            self.follow_sensor = self.right
              
        elif self.state == 18: #setup pid für linke wand folgen
            error = self.desired_distance - left_sensor
            self.wall_distance_pid.set_previous_error(error)    

            if self.pledge_count == []:
                self.pledge_count = 0

        elif self.state == 19: #setup pid für rechte wand folgen 
            error = self.desired_distance - right_sensor
            self.wall_distance_pid.set_previous_error(error)  

            if self.pledge_count == []:
                self.pledge_count = 0

        ############################# pledge ####################################

        return self.left_wheel_velocity, self.right_wheel_velocity

    def get_pledge_count(self):
        return self.pledge_count
        
    def get_sensor_index(self, sensor_angles, direction):
        # Definiere die Richtungen im Bogenmaß
        direction_map = {
            'left': np.pi / 2,
            'front_left': np.pi / 4,
            'front': 0,
            'front_right': -np.pi / 4,
            'right': -np.pi / 2,
        }

        target_angle = direction_map[direction]
        
        # Berechne die Differenz zwischen den Sensorwinkeln und der gesuchten Richtung
        angle_differences = [abs(angle - target_angle) for angle in sensor_angles]
        
        # Finde den Index des kleinsten Unterschieds
        closest_index = np.argmin(angle_differences)
        
        return closest_index


class OccupancyGridMapping:
    def __init__(self, grid_size, resolution, sensor_activation_threshold, sensor_angles):
        self.grid_size = (grid_size, grid_size)  # Größe des Gitters in Zellen (width, height)
        self.resolution = resolution  # Größe jeder Zelle in Metern
        self.sensor_range = sensor_activation_threshold  # Reichweite der Sensoren in Metern
        self.sensor_angles = sensor_angles  # Roboter Modellparameter
        self.grid = np.zeros(self.grid_size)  # Belegungsgrid initialisieren
        self.visited = np.zeros(self.grid_size)  # Raster für besuchte Zellen
        
    def world_to_grid(self, x, y):
        grid_x = int(x / self.resolution)
        # Hier wird die Y-Koordinate gespiegelt
        grid_y = self.grid_size[1] - int(y / self.resolution) - 1
        return grid_x, grid_y

    def update(self, robot_position, robot_angle, sensor_readings):
        self.robot_position = robot_position  # Aktuelle Roboterposition speichern
        robot_x, robot_y = robot_position
        grid_robot_x, grid_robot_y = self.world_to_grid(robot_x, robot_y)
        
        for i, sensor_distance in enumerate(sensor_readings):
            angle = robot_angle + self.sensor_angles[i]
            if sensor_distance < self.sensor_range:
                endpoint_x = robot_x + sensor_distance * np.cos(angle)
                endpoint_y = robot_y + sensor_distance * np.sin(angle)
                grid_endpoint_x, grid_endpoint_y = self.world_to_grid(endpoint_x, endpoint_y)
                
                # Zeichne eine Linie von Roboterposition zur Sensorendposition und markiere sie als besuchte Zellen
                for x, y in self.bresenham(grid_robot_x, grid_robot_y, grid_endpoint_x, grid_endpoint_y):
                    if 0 <= x < self.grid_size[0] and 0 <= y < self.grid_size[1]:
                        self.visited[y, x] = 1
                        self.grid[y, x] = 1 if (x, y) == (grid_endpoint_x, grid_endpoint_y) else -1

    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points

    def get_grid_size(self):
        return self.grid_size
    
    def get_visited(self):
        return self.visited
    
    def get_grid(self):
        return self.grid
    
    def get_resolution(self):
        return self.resolution
    
    def reset(self):
        self.grid = np.zeros(self.grid_size)  # Belegungsgrid initialisieren
        self.visited = np.zeros(self.grid_size)  # Raster für besuchte Zellen


class Enviroment:
    def __init__ (self, screen_width, screen_height, x_offset, walls_list, scale, grid_size, robot_x=0, robot_y=0, robot_angle=0, acceleration=1, screen_refresh_rate = 1, map_refresh_rate = 1):

        # Define colors
        self.WHITE = (245, 245, 230)
        self.BLACK = (0, 0, 0)
        self.RED = (255, 0, 0)
        self.GREY = (200, 200, 200)
        self.DARK_GRAY = (100, 100, 100)
        self.GREEN = (0, 255, 0)

       
        self.robot_x = robot_x
        self.robot_y = robot_y
        self.robot_angle = robot_angle
       
        self.i = 0
        self.acceleration = acceleration
        self.zoom_factor = 1
        

        # Initialize Pygame
        pygame.init()


        # Define screen dimensions and create screen object
        self.screen_width, self.screen_height = screen_width, screen_height
        self.x_offset = x_offset
        self.y_offset = 0
        self.scale = scale
        self.display = pygame.display.set_mode((screen_width, screen_height))
        pygame.display.set_caption('Differential Drive Robot Simulation')

        # Globale Variablen zur Entprellung
        self.last_input_time = 0
        self.input_delay = 0.15  # Zeit in Sekunden, die zwischen zwei Eingaben vergehen muss
        

        # Font for displaying text
        self.font = pygame.font.SysFont('Consolas', 16)
        self.text_distance = 20.5
        # Wall properties

        self.walls_list_index = 0
        self.walls_list = walls_list
        if self.walls_list and isinstance(self.walls_list[0], list):
            self.walls = self.walls_list[self.walls_list_index]
        else:
            self.walls = self.walls_list

        self.grid_size = (grid_size, grid_size)
        self.map_surface = pygame.Surface((self.grid_size[0] *  self.scale , self.grid_size[1] *  self.scale)) 
        self.text_surface = pygame.Surface((self.map_surface.get_width(), screen_height - self.map_surface.get_height())) 
        self.screen = pygame.Surface((screen_width, screen_height)) 

        self.map_refresh_rate = map_refresh_rate
        self.screen_refresh_rate = screen_refresh_rate
          
    def draw_grid(self):
            """Draw the grid on the screen."""
            for i in range(0, self.screen_width, int(20 * self.zoom_factor)):
                pygame.draw.line(self.screen, self.GREY, (i, 0), (i, self.screen_height))
            for j in range(0, self.screen_height, int(20 * self.zoom_factor)):
                pygame.draw.line(self.screen, self.GREY, (0, j), (self.screen_width, j))

    def draw_robot(self, x, y, theta, robot_radius):
        scaled_x = int(x * self.zoom_factor)
        scaled_y = int(((self.screen_height / self.zoom_factor) - y) * self.zoom_factor)  # Spiegeln der Y-Koordinate
        scaled_radius = int(robot_radius * self.zoom_factor)
        
        pygame.draw.circle(self.screen, self.RED, (scaled_x, scaled_y), scaled_radius)
        line_length = scaled_radius
        end_x = scaled_x + line_length * math.cos(theta)
        end_y = scaled_y - line_length * math.sin(theta)  # Anpassen der Y-Richtung
        pygame.draw.line(self.screen, self.BLACK, (scaled_x, scaled_y), (end_x, end_y), 2)

    def display_text(self, text, position):
        """Display text on the screen at the given position."""
        label = self.font.render(text, True, self.GREY)
        self.text_surface.blit(label, position)
        self.display.blit(self.text_surface, (self.x_offset, self.map_surface.get_height()))
        
    def draw_sensors(self, x, y, theta, sensor_readings, sensor_range, sensor_angles):
        scaled_x = int(x * self.zoom_factor)
        scaled_y = int(((self.screen_height / self.zoom_factor) - y) * self.zoom_factor)  # Spiegeln der Y-Koordinate
        scaled_sensor_range = int(sensor_range * self.zoom_factor)
        
        for angle in sensor_angles:
            sensor_theta = theta + angle
            sensor_reading = sensor_readings[sensor_angles.index(angle)] * self.zoom_factor
            if sensor_reading < scaled_sensor_range:
                end_x = scaled_x + sensor_reading * math.cos(sensor_theta)
                end_y = scaled_y - sensor_reading * math.sin(sensor_theta)  # Anpassen der Y-Richtung
            else:
                end_x = scaled_x + scaled_sensor_range * math.cos(sensor_theta)
                end_y = scaled_y - scaled_sensor_range * math.sin(sensor_theta)  # Anpassen der Y-Richtung
            pygame.draw.line(self.screen, self.GREEN, (scaled_x, scaled_y), (end_x, end_y), 1)

    def get_all_sensor_to_wall_distances(self, sensor_angles, sensor_range):
    
        sensor_readings = []
        for angle in sensor_angles:
            sensor_theta = self.robot_angle + angle
            sensor_reading = self.get_sensor_to_wall_distance(self.robot_x, self.robot_y, sensor_theta, self.walls, sensor_range)
            sensor_readings.append(sensor_reading)
    
        return sensor_readings
    
    def draw_walls(self):
        for wall in self.walls:
            wall_x, wall_y, wall_width, wall_height = wall
            scaled_rect = pygame.Rect(wall_x * self.zoom_factor, ((self.screen_height / self.zoom_factor) - wall_y - wall_height) * self.zoom_factor,
                                    wall_width * self.zoom_factor, wall_height * self.zoom_factor)
            pygame.draw.rect(self.screen, self.BLACK, scaled_rect)

    def get_sensor_to_wall_distance(self, x, y, theta, walls, sensor_range):
        min_dist = sensor_range
        for wall in walls:
            wall_x, wall_y, wall_width, wall_height = wall
            corners = [
                (wall_x, wall_y),
                (wall_x + wall_width, wall_y),
                (wall_x, wall_y + wall_height),
                (wall_x + wall_width, wall_y + wall_height)
            ]
            edges = [
                (corners[0], corners[1]),  # Oberkante
                (corners[0], corners[2]),  # Linke Kante
                (corners[1], corners[3]),  # Rechte Kante
                (corners[2], corners[3])   # Unterkante
            ]
            for edge in edges:
                intersect_point = self.get_line_intersection(
                    (x, y),
                    (x + sensor_range * math.cos(theta), y + sensor_range * math.sin(theta)),
                    edge[0], edge[1]
                )
                if intersect_point:
                    dist = math.sqrt((intersect_point[0] - x) ** 2 + (intersect_point[1] - y) ** 2)
                    if dist < min_dist:
                        min_dist = dist
        return min_dist

    def get_line_intersection(self, p0, p1, p2, p3):
        s1_x = p1[0] - p0[0]
        s1_y = p1[1] - p0[1]
        s2_x = p3[0] - p2[0]
        s2_y = p3[1] - p2[1]
        denominator = (-s2_x * s1_y + s1_x * s2_y)
        if denominator == 0:
            return None  # Parallele Linien
        s = (-s1_y * (p0[0] - p2[0]) + s1_x * (p0[1] - p2[1])) / denominator
        t = (s2_x * (p0[1] - p2[1]) - s2_y * (p0[0] - p2[0])) / denominator
        if 0 <= s <= 1 and 0 <= t <= 1:
            intersect_x = p0[0] + (t * s1_x)
            intersect_y = p0[1] + (t * s1_y)
            return (intersect_x, intersect_y)
        return None
    
    def check_collision(self, x, y, robot_radius):
        """Check if the robot collides with any wall."""
        for wall in self.walls:
            rect = pygame.Rect(wall[0], wall[1], wall[2], wall[3])
            if self.check_circle_rect_collision(x, y, robot_radius, rect):
                return True
        return False

    def check_circle_rect_collision(self, cx, cy, radius, rect):
        """Check if a circle with center (cx, cy) and radius collides with a rectangle."""
        # Find the closest point to the circle within the rectangle
        nearest_x = max(rect.left, min(cx, rect.right))
        nearest_y = max(rect.top, min(cy, rect.bottom))
        
        # Calculate the distance between the circle's center and this closest point
        dx = cx - nearest_x
        dy = cy - nearest_y
        
        # If the distance is less than the circle's radius, a collision occurs
        return (dx * dx + dy * dy) < (radius * radius)

    def get_nearest_point_on_line(self, line_start, line_end, point):
        """Calculate the nearest point on a line segment to a given point."""
        px, py = point
        x1, y1 = line_start
        x2, y2 = line_end

        line_dx = x2 - x1
        line_dy = y2 - y1

        if line_dx == 0 and line_dy == 0:
            return line_start

        t = ((px - x1) * line_dx + (py - y1) * line_dy) / (line_dx ** 2 + line_dy ** 2)
        t = max(0, min(1, t))

        nearest_x = x1 + t * line_dx
        nearest_y = y1 + t * line_dy

        return (nearest_x, nearest_y)

    def handle_user_input(self, angle_setpoint, base_speed, autonomous_mode, mesurment_noise, system_noise, algo, collision_active, reset_auto, set_robo_pos, reset_map):
        """Handle user input to adjust wheel velocities."""
        current_time = time.time()
        
        # Überprüfen, ob die Eingabeentprellungszeit verstrichen ist
        if current_time - self.last_input_time > self.input_delay:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_LEFT]:
                angle_setpoint += math.radians(5)
                self.last_input_time = current_time
            if keys[pygame.K_RIGHT]:
                angle_setpoint -= math.radians(5)
                self.last_input_time = current_time
            if keys[pygame.K_UP]:
                base_speed += 1
                self.last_input_time = current_time
            if keys[pygame.K_DOWN]:
                base_speed -= 1
                self.last_input_time = current_time
            if keys[pygame.K_a]:
                autonomous_mode = not autonomous_mode
                if autonomous_mode is False:
                    angle_setpoint = self.robot_angle
                    base_speed = 0
                else:
                    reset_auto()
                self.last_input_time = current_time
            if keys[pygame.K_o]:
                autonomous_mode = True 
            if keys[pygame.K_s]:
                system_noise = not system_noise
                self.last_input_time = current_time
            if keys[pygame.K_m]:
                mesurment_noise = not mesurment_noise
                self.last_input_time = current_time    
            if keys[pygame.K_PLUS] or keys[pygame.K_KP_PLUS]:  
                self.zoom_factor *= 1.05  # Zoom in
                self.last_input_time = current_time
            if keys[pygame.K_MINUS] or keys[pygame.K_KP_MINUS]:  
                self.zoom_factor /= 1.05  # Zoom out
                self.last_input_time = current_time
            if keys[pygame.K_r]:  
                reset_auto()
                reset_map()
                set_robo_pos(self.robot_x, self.robot_y, self.robot_angle)              
                self.last_input_time = current_time
            if keys[pygame.K_1]:  
                algo = 'RH' 
                reset_auto() 
                self.last_input_time = current_time
            if keys[pygame.K_2]:  
                algo = 'V2' 
                reset_auto() 
                self.last_input_time = current_time
            if keys[pygame.K_3]:  
                algo = 'PG' 
                reset_auto()
                self.last_input_time = current_time
            if keys[pygame.K_k]:  
                collision_active = not collision_active
                self.last_input_time = current_time
            if keys[pygame.K_TAB]:  
                self.change_walls()
                reset_auto()
                reset_map()
                set_robo_pos(self.robot_x, self.robot_y, self.robot_angle)
                autonomous_mode = False
                base_speed = 0
                self.last_input_time = current_time

        return angle_setpoint, base_speed, autonomous_mode, mesurment_noise, system_noise, algo, collision_active

    def draw_map(self, scale, x_robot, y_robot, grid_size, visited, grid, resolution):
        # Karte auf dem Surface zeichnen
        self.map_surface.fill(self.DARK_GRAY)
        
        for y in range(grid_size[1]):
            for x in range(grid_size[0]):
                color = self.DARK_GRAY  # Grau für nicht besuchte Zellen
                if visited[y, x] == 1:
                    if grid[y, x] == 1:
                        color = (0, 0, 0)  # Schwarz für belegte Zellen
                    elif grid[y, x] == -1:
                        color = (200, 200, 200)  # Hellgrau für freie Zellen
                pygame.draw.rect(self.map_surface, color, (x * scale, y * scale, scale + 0.29, scale + 0.29))

        # Zeichne die aktuelle Roboterposition als rotes Kästchen
        grid_robot_x, grid_robot_y = self.world_to_grid(x_robot, y_robot, resolution)
        pygame.draw.rect(self.map_surface, (255, 0, 0), (grid_robot_x * scale, grid_robot_y * scale, scale + 0.29, scale + 0.29))

    def blit_map(self):
        # Gespeicherte Karte auf den Bildschirm blitten
        self.display.blit(self.map_surface, (self.x_offset, self.y_offset))

    def world_to_grid(self, x, y, resolution):
        # Wandelt Weltkoordinaten in Rasterkoordinaten um
        grid_x = int(x / resolution)
        # Hier wird die Y-Koordinate ebenfalls gespiegelt
        grid_y = self.grid_size[1] - int(y / resolution) - 1
        return grid_x, grid_y  

    def get_position_and_angle(self):
        return self.robot_x, self.robot_y, self.robot_angle
    
    def update_position_and_angle(self, x, y, theta, collision_active, robot_radius):
        if (not self.check_collision(x, y, robot_radius) and collision_active) or not collision_active:
            self.robot_x, self.robot_y, self.robot_angle = x, y, theta
        
    def get_running(self):
        running = True
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False  
        
        return running          

    def get_sensor_label(self, angle):
        if angle == math.radians(0):
            return "front"
        elif angle < math.radians(0):
            if angle == math.radians(-45):
                return "frontright"
            else:
                return "right"
        else:
            if angle == math.radians(45):
                return "frontleft"
            else:
                return "left"

    def draw(self, robot_radius, sensor_distances, sensor_readings, sensor_range, sensor_angles, x, y, visited, grid, resolution, left_wheel_velocity, right_wheel_velocity,
         angle_setpoint, base_speed, autonomous_mode, state, follow_sensor, on_closed_loop, mesurment_noise_active, system_noise_active, algo, plegde_count, collision_active):

        if (self.i % self.screen_refresh_rate) == 0:
            # Clear screen
            self.screen.fill(self.WHITE)

            # Draw grid
            self.draw_grid()

            # Draw walls
            self.draw_walls()

            # Draw robot and sensors
            self.draw_robot(self.robot_x, self.robot_y, self.robot_angle, robot_radius)
            self.draw_sensors(self.robot_x, self.robot_y, self.robot_angle, sensor_distances, sensor_range, sensor_angles)
            self.display.blit(self.screen,(0,0))

        ##############################################################################
        # Display text information
        self.text_surface.fill(self.BLACK)
        if autonomous_mode:
            if algo == "V2":
                info = [
                    f"Left Wheel V:      {left_wheel_velocity:.2f}",
                    f"Right Wheel V:     {right_wheel_velocity:.2f}",
                    f"Position:  {self.robot_x:.2f}  {self.robot_y:.2f}",
                    f"Orientation:       {math.degrees(self.robot_angle):.2f}°",
                    f"Angle Setpoint:    {math.degrees(angle_setpoint):.2f}°",
                    f"Base Speed:        {base_speed:.2f}",
                    f"Autonomous Mode:   {'ON' if autonomous_mode else 'OFF'}  {algo}",
                    f"State:             {state}",
                    f"Follow Sensor:     {self.get_sensor_label(sensor_angles[follow_sensor]) if follow_sensor != [] else follow_sensor}",
                    f"On Closed Loop:    {on_closed_loop}",
                    f"Measurement Noise: {'ON' if mesurment_noise_active else 'OFF'}",
                    f"System Noise:      {'ON' if system_noise_active else 'OFF'}",
                ]
            elif algo == "RH":
                info = [
                    f"Left Wheel V:      {left_wheel_velocity:.2f}",
                    f"Right Wheel V:     {right_wheel_velocity:.2f}",
                    f"Position:  {self.robot_x:.2f}  {self.robot_y:.2f}",
                    f"Orientation:       {math.degrees(self.robot_angle):.2f}°",
                    f"Angle Setpoint:    {math.degrees(angle_setpoint):.2f}°",
                    f"Base Speed:        {base_speed:.2f}",
                    f"Autonomous Mode:   {'ON' if autonomous_mode else 'OFF'}  {algo}",
                    f"State:             {state}",
                    f"Follow Sensor:     {self.get_sensor_label(sensor_angles[follow_sensor]) if follow_sensor != [] else follow_sensor}",
                    f"Measurement Noise: {'ON' if mesurment_noise_active else 'OFF'}",
                    f"System Noise:      {'ON' if system_noise_active else 'OFF'}",
                ]
            elif algo == "PG":
                info = [
                    f"Left Wheel V:      {left_wheel_velocity:.2f}",
                    f"Right Wheel V:     {right_wheel_velocity:.2f}",
                    f"Position:  {self.robot_x:.2f}  {self.robot_y:.2f}",
                    f"Orientation:       {math.degrees(self.robot_angle):.2f}°",
                    f"Angle Setpoint:    {math.degrees(angle_setpoint):.2f}°",
                    f"Base Speed:        {base_speed:.2f}",
                    f"Autonomous Mode:   {'ON' if autonomous_mode else 'OFF'}  {algo}",
                    f"State:             {state}",
                    f"Follow Sensor:     {self.get_sensor_label(sensor_angles[follow_sensor]) if follow_sensor != [] else follow_sensor}",
                    f"Plegde count  :    {plegde_count}",
                    f"Measurement Noise: {'ON' if mesurment_noise_active else 'OFF'}",
                    f"System Noise:      {'ON' if system_noise_active else 'OFF'}",
                ]

        else:
            info = [
                f"Left Wheel V:      {left_wheel_velocity:.2f}",
                f"Right Wheel V:     {right_wheel_velocity:.2f}",
                f"Position:  {self.robot_x:.2f}  {self.robot_y:.2f}",
                f"Orientation:       {math.degrees(self.robot_angle):.2f}°",
                f"Angle Setpoint:    {math.degrees(angle_setpoint):.2f}°",
                f"Base Speed:        {base_speed:.2f}",
                f"Autonomous Mode:   {'ON' if autonomous_mode else 'OFF'}  {algo}",
                f"Collision:         {'ON' if collision_active else 'OFF'}",
                f"Measurement Noise: {'ON' if mesurment_noise_active else 'OFF'}",
                f"System Noise:      {'ON' if system_noise_active else 'OFF'}",
            ]
            
        # Add sensor readings to info list
        max_label_length = max(len(self.get_sensor_label(angle)) for angle in sensor_angles)
        for i, sensor_reading in enumerate(sensor_readings):
            label = self.get_sensor_label(sensor_angles[i])
            padded_label = label.ljust(max_label_length)
            info.append(f"Sensor {padded_label}: {sensor_reading:.2f}")

        # Display all info texts
        for i, line in enumerate(info):
            self.display_text(line, (10, 10 + i * self.text_distance))
        ################################################################################

        if (self.i % self.map_refresh_rate) == 0:
            self.draw_map(self.scale, x, y, self.grid_size, visited, grid, resolution)
        self.blit_map()

        self.i += 1
        pygame.display.flip()
        
        # Cap the frame rate
        pygame.time.Clock().tick(240)

    def get_time(self):
        time =  self.acceleration * pygame.time.get_ticks() / 1000.0 
        return time

    def quit(self):
        pygame.quit()

    def change_walls(self):
            if self.walls_list and isinstance(self.walls_list[0], list):
                if self.walls_list_index < (len(self.walls_list) - 1):
                    self.walls_list_index += 1
                else:     
                    self.walls_list_index = 0

                self.walls =  self.walls_list[self.walls_list_index]  

    
class Robot:
    def __init__ (self, mesurment_noise_mean = 0, mesurment_noise_standard_deviation = 1, system_noise_mean = 0, system_noise_standard_deviation = 1, init_robot_x = 0, init_robot_y = 0, init_robot_angle = 0):
        self.robot_radius = 20
        self.wheel_distance = 40  # Distance between wheels
        
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

        self.lpf_sensors = [LowPassFilter(cutoff_freq=3) for _ in self.sensor_angles]

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

    def state_estimate(self, left_wheel_velocity, right_wheel_velocity, time_step):
        self.robot_x, self.robot_y, self.robot_angle = self.update_robot(self.robot_x, self.robot_y, self.robot_angle, left_wheel_velocity, right_wheel_velocity, time_step)

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
        
########################################################################################################
    


########################################################################################################

def generate_walls_square(wall_length, screen_width, screen_height, wall_thickness):
    half_length = wall_length // 2
    half_screen_width = screen_width // 2
    half_screen_height = screen_height // 2
    walls = [
        (half_screen_width - half_length, half_screen_height - half_length, wall_length, wall_thickness),  # Oben
        (half_screen_width + half_length - wall_thickness, half_screen_height - half_length, wall_thickness, wall_length),  # Rechts
        (half_screen_width - half_length, half_screen_height + half_length - wall_thickness, wall_length, wall_thickness),  # Unten
        (half_screen_width - half_length, half_screen_height - half_length, wall_thickness, wall_length)  # Links
    ]
    return walls

def generate_walls_two_squares(outer_wall_length, gap_between_squares, screen_width, screen_height, wall_thickness):
    inner_wall_length = outer_wall_length - 2 * gap_between_squares

    half_outer_length = outer_wall_length // 2
    half_inner_length = inner_wall_length // 2
    half_screen_width = screen_width // 2
    half_screen_height = screen_height // 2

    walls = [
        # Äußeres Quadrat
        (half_screen_width - half_outer_length, half_screen_height - half_outer_length, outer_wall_length, wall_thickness),  # Oben
        (half_screen_width + half_outer_length - wall_thickness, half_screen_height - half_outer_length, wall_thickness, outer_wall_length),  # Rechts
        (half_screen_width - half_outer_length, half_screen_height + half_outer_length - wall_thickness, outer_wall_length, wall_thickness),  # Unten
        (half_screen_width - half_outer_length, half_screen_height - half_outer_length, wall_thickness, outer_wall_length),  # Links

        # Inneres Quadrat
        (half_screen_width - half_inner_length, half_screen_height - half_inner_length, inner_wall_length, wall_thickness),  # Oben
        (half_screen_width + half_inner_length - wall_thickness, half_screen_height - half_inner_length, wall_thickness, inner_wall_length),  # Rechts
        (half_screen_width - half_inner_length, half_screen_height + half_inner_length - wall_thickness, inner_wall_length, wall_thickness),  # Unten
        (half_screen_width - half_inner_length, half_screen_height - half_inner_length, wall_thickness, inner_wall_length)  # Links
    ]

    return walls

def generate_walls_large_rectangle_with_squares(
    outer_rectangle_width, outer_rectangle_height,
    square1_length, square2_length,
    gap_between_squares, screen_width, screen_height, wall_thickness):
    
    # Berechnung der Positionen der Wände für das große Rechteck
    half_outer_width = outer_rectangle_width // 2
    half_outer_height = outer_rectangle_height // 2
    half_screen_width = screen_width // 2
    half_screen_height = screen_height // 2

    rectangle_walls = [
        (half_screen_width - half_outer_width, half_screen_height - half_outer_height, outer_rectangle_width, wall_thickness),  # Oben
        (half_screen_width + half_outer_width - wall_thickness, half_screen_height - half_outer_height, wall_thickness, outer_rectangle_height),  # Rechts
        (half_screen_width - half_outer_width, half_screen_height + half_outer_height - wall_thickness, outer_rectangle_width, wall_thickness),  # Unten
        (half_screen_width - half_outer_width, half_screen_height - half_outer_height, wall_thickness, outer_rectangle_height)  # Links
    ]

    # Berechnung der Positionen des ersten Quadrats
    square1_top_left_x = half_screen_width - (square1_length + gap_between_squares + square2_length) // 2
    square1_top_left_y = half_screen_height - square1_length // 2

    square1_walls = [
        (square1_top_left_x, square1_top_left_y, square1_length, wall_thickness),  # Oben
        (square1_top_left_x + square1_length - wall_thickness, square1_top_left_y, wall_thickness, square1_length),  # Rechts
        (square1_top_left_x, square1_top_left_y + square1_length - wall_thickness, square1_length, wall_thickness),  # Unten
        (square1_top_left_x, square1_top_left_y, wall_thickness, square1_length)  # Links
    ]

    # Berechnung der Positionen des zweiten Quadrats
    square2_top_left_x = square1_top_left_x + square1_length + gap_between_squares
    square2_top_left_y = half_screen_height - square2_length // 2

    square2_walls = [
        (square2_top_left_x, square2_top_left_y, square2_length, wall_thickness),  # Oben
        (square2_top_left_x + square2_length - wall_thickness, square2_top_left_y, wall_thickness, square2_length),  # Rechts
        (square2_top_left_x, square2_top_left_y + square2_length - wall_thickness, square2_length, wall_thickness),  # Unten
        (square2_top_left_x, square2_top_left_y, wall_thickness, square2_length)  # Links
    ]

    walls = rectangle_walls + square1_walls + square2_walls

    return walls



def generate_maze(width, height):
    maze = [[0 for _ in range(width)] for _ in range(height)]
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    def is_valid(x, y):
        return 1 <= x < width - 1 and 1 <= y < height - 1 and maze[y][x] == 0

    def carve_passages_from(x, y):
        maze[y][x] = 1
        random.shuffle(directions)
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if is_valid(nx, ny) and maze[ny][nx] == 0:
                if 1 <= nx + dx < width - 1 and 1 <= ny + dy < height - 1 and maze[ny + dy][nx + dx] == 0:
                    maze[ny][nx] = 1
                    maze[ny + dy][nx + dx] = 1
                    carve_passages_from(nx + dx, ny + dy)

    start_x, start_y = random.randint(1, width - 2), random.randint(1, height - 2)
    carve_passages_from(start_x, start_y)
    
    return maze

def maze_to_walls(maze, cell_size, wall_thickness, Top = True, Bottom = True, Left = True, Right = True):
    height = len(maze)
    width = len(maze[0])
    walls = []
    for y in range(height):
        for x in range(width):
            cx = x * cell_size #+ (screen_width - width * cell_size) // 2
            cy = y * cell_size #+ (screen_height - height * cell_size) // 2
            
            if maze[y][x] == 0:
                if Top and Bottom and Right and Left:
                    walls.append((cx, cy, cell_size + 3, cell_size + 3))
                else:
                    if Top:
                        walls.append((cx, cy, cell_size, wall_thickness))
                    # Left wall
                    if Left:
                        walls.append((cx, cy, wall_thickness, cell_size))
                    
                    # Bottom wall
                    if Bottom:
                        walls.append((cx, cy + cell_size - wall_thickness, cell_size, wall_thickness))
                    # Right wall
                    if Right:
                        walls.append((cx + cell_size - wall_thickness, cy, wall_thickness, cell_size))
    return walls

def shift_walls(walls, offset_x, offset_y):
    shifted_walls = []
    for wall in walls:
        shifted_wall = (wall[0] + offset_x, wall[1] + offset_y, wall[2], wall[3])
        shifted_walls.append(shifted_wall)
    return shifted_walls




def init_Classes():

    #Enviroment
    screen_width, screen_height = 1250, 650
    x_offset_sreen = screen_width // 2 + 360
    acceleration = 2.8
    screen_refresh_rate = 1
    map_refresh_rate = 40
    wall_thickness = 10
    
    
    wall_length = 640
    gap_between_squares = 190
    walls_list = []
    
    walls_list.append(generate_walls_square(wall_length, screen_width, screen_height, wall_thickness))
    walls_list.append(generate_walls_two_squares(wall_length, gap_between_squares, screen_width, screen_height, wall_thickness))

    outer_rectangle_width = 1200
    outer_rectangle_height = 640
    square1_length = 300 
    square2_length = 300
    gap_between_inner_squares = 190 
    
    walls_list.append(generate_walls_large_rectangle_with_squares(outer_rectangle_width, outer_rectangle_height,
                                                         square1_length, square2_length,gap_between_inner_squares, screen_width, screen_height, wall_thickness))

    maze_width = 21     # number of zells in width
    maze_height = 14    # number of zells in height
    maze_cell_size = 120
    
    maze = generate_maze(maze_width, maze_height)
    walls_list.append(maze_to_walls(maze, maze_cell_size, wall_thickness))
    walls_list.append(maze_to_walls(maze, maze_cell_size, wall_thickness, Right=False, Bottom=False))

    #init Position    
    init_robot_x = (screen_width // 2) #- 210
    init_robot_y = screen_height // 2 #650, 400  #screen_width // 2, screen_height // 2 #650, self.activation_threshold #
    init_robot_angle =  math.radians(-180)
   
    #ESC
    dither_frequency = 0.8  # Frequency of dither signal (Hz)
    dither_amplitude = 0.055   # Amplitude of dither signal
    learning_rate = 0.13  # Learning rate for ESC
    esc_angle_comparison_interval = 1
    esc_angel_toleranz = 0.6
    
    # Robot
    sys_sigma = 1
    mes_sigma = 1

    #Auto
    init_base_speed = 25
    init_base_rotation_speed = 0.5
    desired_distance = 45  # Desired distance from the wall
    sensor_activation_threshold = 150 #= robot.get_sensor_range() * 0.75 
    direkt_change_toleranz = 32


    # Occupancy Grid Mapping initialisieren
    grid_size = 150
    resolution = 17
    scale = 4 * (68 / grid_size)


    robot = Robot(system_noise_standard_deviation=sys_sigma, mesurment_noise_standard_deviation=mes_sigma, init_robot_x=init_robot_x, init_robot_y=init_robot_y, init_robot_angle=init_robot_angle)
    
    
    angle_pid = PIDController(kp=15.0, ki=0, kd=0.1) # PID controller for angle control
    wall_distance_pid = PIDController(kp=0.1, ki=0, kd=1)  # PID controller for wall distance control
    point_distance_pid = PIDController(kp=-5, ki=0, kd=0.1) # PID controller for point distance control
    esc = ESCController(dither_frequency, dither_amplitude, learning_rate)
    
    autonomous_controller = AutonomousController(angle_pid, wall_distance_pid, point_distance_pid, esc, init_base_speed, init_base_rotation_speed, desired_distance, sensor_activation_threshold, 
                                                 robot.get_wheel_distance(), robot.get_sensor_angles(), esc_angel_toleranz=esc_angel_toleranz, esc_angle_comparison_interval=esc_angle_comparison_interval,
                                                 direkt_change_toleranz=direkt_change_toleranz)

     
    mapping = OccupancyGridMapping(grid_size, resolution, sensor_activation_threshold, robot.get_sensor_angles()) 
    

    ev = Enviroment(screen_width, screen_height, x_offset_sreen, walls_list, scale, grid_size, robot_x=init_robot_x, robot_y=init_robot_y, robot_angle=init_robot_angle, 
                    screen_refresh_rate=screen_refresh_rate, map_refresh_rate=map_refresh_rate)


    return ev, robot, angle_pid, autonomous_controller, mapping 

def main():

    ev, robot, angle_pid, autonomous_controller, mapping = init_Classes()

    running = ev.get_running()
    autonomous_mode = False
    collision_active = True
   
    auto_algo = 'PG'
    mesurment_noise_active = False
    system_noise_active = False

    angle_setpoint = ev.get_position_and_angle()[2]
    base_speed = 0
    prev_time = 0  
    i = 0

    while running:

        current_time = ev.get_time()            
        #time_step = current_time - prev_time
        time_step = 0.085
        prev_time = current_time
         
        x, y, theta = robot.get_position_and_angle()    

        sensor_distances = ev.get_all_sensor_to_wall_distances(robot.get_sensor_angles(), robot.get_sensor_range())
        
        if not mesurment_noise_active:
            sensor_readings = robot.get_sensor_readings(sensor_distances)
        else:    
            sensor_readings = robot.get_sensor_readings_with_noise(sensor_distances)
            sensor_readings = robot.filter_sensor_readings(sensor_readings, time_step)

        mapping.update((x, y), theta, sensor_readings)
        
        angle_setpoint, base_speed, autonomous_mode, mesurment_noise_active, system_noise_active, auto_algo, collision_active = ev.handle_user_input(angle_setpoint, base_speed, autonomous_mode, mesurment_noise_active, system_noise_active,
                                                                                                                         auto_algo, collision_active, autonomous_controller.reset, robot.set_position_and_angle, mapping.reset)

        state = []
        follow_sensor = []
        on_closed_loop = []
        plegde_count = []

        if not autonomous_mode:
            # PID controller to adjust wheel velocities
            angle_control = angle_pid.update(angle_setpoint, theta, time_step)
            left_wheel_velocity = base_speed - angle_control
            right_wheel_velocity = base_speed + angle_control
        else:
            # Autonomous control based on sensor readings
            if auto_algo == 'PG':
                left_wheel_velocity, right_wheel_velocity = autonomous_controller.autonomous_control_pledge(sensor_readings, x, y, theta, current_time, time_step)
            elif auto_algo == 'V2':
                left_wheel_velocity, right_wheel_velocity = autonomous_controller.autonomous_control_V2(sensor_readings, x, y, theta, current_time, time_step)   
            elif auto_algo == 'RH':
                left_wheel_velocity, right_wheel_velocity = autonomous_controller.autonomous_control_right_hand(sensor_readings, x, y, theta, current_time, time_step)
            
            base_speed = autonomous_controller.get_base_speed()
            state = autonomous_controller.get_state()
            on_closed_loop = autonomous_controller.get_on_closed_loop()
            follow_sensor = autonomous_controller.get_follow_sensor()
            angle_setpoint = autonomous_controller.get_set_angle_set_point()
            plegde_count = autonomous_controller.get_pledge_count()
            collision_active = True

       
        # Update robot position and orientation  
        ev_x, ev_y, ev_theta = ev.get_position_and_angle()      
        if not system_noise_active:   
            ev_x, ev_y, ev_theta = robot.update_robot(ev_x, ev_y, ev_theta, left_wheel_velocity, right_wheel_velocity, time_step)
        else:
            ev_x, ev_y, ev_theta = robot.update_robot_with_noise(ev_x, ev_y, ev_theta, left_wheel_velocity, right_wheel_velocity, time_step)
           
        robot.state_estimate(left_wheel_velocity, right_wheel_velocity, time_step)    
        ev.update_position_and_angle(ev_x, ev_y, ev_theta, collision_active, robot.get_robot_radius())
        
        if (i % 1) == 0:
            ev.draw(robot.get_robot_radius(), sensor_distances, sensor_readings,robot.get_sensor_range(), robot.get_sensor_angles(), x, y, mapping.get_visited(), mapping.get_grid(),
                    mapping.get_resolution(), left_wheel_velocity, right_wheel_velocity, angle_setpoint, base_speed, autonomous_mode, state, follow_sensor, on_closed_loop, 
                    mesurment_noise_active, system_noise_active, auto_algo, plegde_count, collision_active)
        i += 1
        
        running = ev.get_running()

    ev.quit()


if __name__ == "__main__":
    main()