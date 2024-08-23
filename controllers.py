import numpy as np
from filter import HighPassFilter
from filter import LowPassFilter



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
            if (self.I  < self.i_min) or (self.I < -self.i_minmax):#-self.imax:
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
            if self.pid_max <= self.pid_minmax:
                self.PID = self.pid_max
            else:
                self.PID = self.pid_minmax
        
        return self.PID
    
    def set_previous_error(self,error):
        self.previous_error = error
        
    def set_integral(self, integral):
        self.integral = integral



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

