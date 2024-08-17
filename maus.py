import time
import motoron
import RPi.GPIO as GPIO
import math
import numpy as np
import keyboard  # Modul für die Handhabung von Tastatureingaben
from ADCDifferentialPi import ADCDifferentialPi
import matplotlib.pyplot as plt
#import matplotlib.animation as animation 
from collections import deque
#from rich.console import Console
#from rich.text import Text
#import curses
import threading

class OutputManager:
    def __init__(self, rtp_window_size = 10):
        #self.plotter = RealTimePlotter(rtp_window_size)
        self.console_output = ConsoleOutput()

    def start_console_output(self):
        self.console_output.start()

    def stop_console_output(self):
        self.console_output.stop()

    def update_console_output(self, robot, left_wheel_velocity, right_wheel_velocity,
                              base_speed, angle_setpoint, angle_control, pid_r, pid_l):
        self.console_output.update(robot, left_wheel_velocity, right_wheel_velocity,
                                   base_speed, angle_setpoint, angle_control, pid_r, pid_l)

    def update_plot(self, current_time, left_wheel_velocity, right_wheel_velocity,
                    left_wheel_velocity_target, right_wheel_velocity_target):
        self.plotter.update_plot(current_time, left_wheel_velocity, right_wheel_velocity,
                                 left_wheel_velocity_target, right_wheel_velocity_target)

    def show_final_plot(self):
        self.plotter.show_final_plot()


class ConsoleOutput:
    def __init__(self):
        self.is_running = False
        self.console_thread = None
        self.info_lines = []

    def start(self):
        """Startet die Konsole."""
        self.is_running = True
        self.console_thread = threading.Thread(target=self.run_console_output)
        self.console_thread.start()

    def stop(self):
        """Stoppt die Konsole."""
        self.is_running = False
        if self.console_thread:
            self.console_thread.join()

    def run_console_output(self):
        """Hält die Konsole am Laufen."""
        while self.is_running:
            self.display_console_output()
            time.sleep(1)  # Aktualisiere die Konsole alle 1 Sekunde

    def update(self, robot, left_wheel_velocity, right_wheel_velocity, base_speed,
               angle_setpoint, angle_control, pid_r, pid_l):
        """Aktualisiert die Informationen für den Konsolenoutput."""
        x, y, theta = robot.get_position_and_angle()

        self.info_lines = [
            "---------------------------------------------------------------------",
            f"Left Wheel Velocity:         {robot.get_left_wheel_velocity():.2f} m/s",
            f"Left Wheel Velocity target:  {left_wheel_velocity:.2f} m/s",
            f"P: {pid_l.P:6.2f}    I: {pid_l.I:6.2f}    D: {pid_l.D:6.2f}    PID: {pid_l.PID:6.2f}",
            "",
            f"Right Wheel Velocity:        {robot.get_right_wheel_velocity():.2f} m/s",
            f"Right Wheel Velocity target: {right_wheel_velocity:.2f} m/s",
            f"P: {pid_r.P:6.2f}    I: {pid_r.I:6.2f}    D: {pid_r.D:6.2f}    PID: {pid_r.PID:6.2f}",
            "",
            f"Base_Speed:                  {base_speed:.2f} m/s",
            f"Angle:                       {math.degrees(theta):.2f} °",
            f"Angle_setpoint:              {math.degrees(angle_setpoint):.2f} °",
            f"Angle_control:               {angle_control:.2f}"
        ]

    def display_console_output(self):
        """Zeigt den Konsolenoutput an."""
        print("\033[H\033[J", end="")  # Lösche die Konsole
        print("\n".join(self.info_lines))



class RealTimePlotter:
    def __init__(self, time_window=10):
        self.time_window = time_window  # Zeitfenster in Sekunden
        self.times = deque(maxlen=1000)  # Speichert die letzten Zeitpunkte
        self.left_wheel_velocities = deque(maxlen=1000)
        self.right_wheel_velocities = deque(maxlen=1000)
        self.left_wheel_velocity_targets = deque(maxlen=1000)
        self.right_wheel_velocity_targets = deque(maxlen=1000)
        
        plt.ion()  # Interaktiver Modus zum Echtzeit-Plotten
        self.fig, self.ax = plt.subplots()
        self.left_wheel_line, = self.ax.plot([], [], label="Left Wheel Velocity", color='b')
        self.right_wheel_line, = self.ax.plot([], [], label="Right Wheel Velocity", color='r')
        self.left_target_line, = self.ax.plot([], [], label="Left Wheel Target", linestyle='-.', color='c')
        self.right_target_line, = self.ax.plot([], [], label="Right Wheel Target", linestyle='-.', color='m')

        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Velocity (m/s)")
        self.ax.set_title("Wheel Velocity Over Time")
        self.ax.legend()
        self.ax.grid(True)

    def update_plot(self, current_time, left_velocity, right_velocity, left_target, right_target):
        # Nur die Datenpunkte innerhalb des Zeitfensters anzeigen
        self.times.append(current_time)
        self.left_wheel_velocities.append(left_velocity)
        self.right_wheel_velocities.append(right_velocity)
        self.left_wheel_velocity_targets.append(left_target)
        self.right_wheel_velocity_targets.append(right_target)

        min_time = current_time - self.time_window
        indices = [i for i, t in enumerate(self.times) if t >= min_time]

        times_window = [self.times[i] for i in indices]
        left_velocities_window = [self.left_wheel_velocities[i] for i in indices]
        right_velocities_window = [self.right_wheel_velocities[i] for i in indices]
        left_targets_window = [self.left_wheel_velocity_targets[i] for i in indices]
        right_targets_window = [self.right_wheel_velocity_targets[i] for i in indices]

        self.left_wheel_line.set_data(times_window, left_velocities_window)
        self.right_wheel_line.set_data(times_window, right_velocities_window)
        self.left_target_line.set_data(times_window, left_targets_window)
        self.right_target_line.set_data(times_window, right_targets_window)

        self.ax.set_xlim(min_time, current_time)
        self.ax.set_ylim(min(min(left_velocities_window), min(right_velocities_window)) - 0.1,
                         max(max(left_velocities_window), max(right_velocities_window)) + 0.1)

        plt.draw()
        plt.pause(0.01)  # Pause für eine kurze Zeit, um den Plot zu aktualisieren

    def show(self):
        plt.show(block=False)  # Blockieren des Hauptprogramms vermeiden



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


class Robot:
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
            GPIO.add_event_detect(self.pin_b_left, GPIO.RISING, callback=self._update_velocity_left)
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
        
        
################################################################################################################
################################################################################################################


def handle_user_input(angle_setpoint, base_speed):
    base_speed = 0
    angle_setpoint = 0
    if keyboard.is_pressed('up'):  # Up arrow key
        base_speed += 0.5
    if keyboard.is_pressed('down'):  # Down arrow key
        base_speed -= 0.5
    if keyboard.is_pressed('left'):  # Left arrow key
        angle_setpoint += 0.15
    if keyboard.is_pressed('right'):  # Right arrow key
        angle_setpoint -= 0.15

    return angle_setpoint, base_speed


def print_terminal(robot, left_wheel_velocity, right_wheel_velocity, base_speed,
                   angle_setpoint, angle_control, pid_r, pid_l):
        
        x, y, theta = robot.get_position_and_angle()

        info_lines = [
            "---------------------------------------------------------------------",
            f"Left Wheel Velocity:         {robot.get_left_wheel_velocity():.2f} m/s",
            f"Left Wheel Velocity target:  {left_wheel_velocity:.2f} m/s",
            f"P: {pid_l.P:6.2f}    I: {pid_l.I:6.2f}    D: {pid_l.D:6.2f}    PID: {pid_l.PID:6.2f}",
            "",
            f"Right Wheel Velocity:        {robot.get_right_wheel_velocity():.2f} m/s",
            f"Right Wheel Velocity target: {right_wheel_velocity:.2f} m/s",
            f"P: {pid_r.P:6.2f}    I: {pid_r.I:6.2f}    D: {pid_r.D:6.2f}    PID: {pid_r.PID:6.2f}",
            "",
            f"Base_Speed:                  {base_speed:.2f} m/s",
            f"angle:                       {math.degrees(theta):.2f} °",
            f"angle_setpoint:              {math.degrees(angle_setpoint):.2f} °",
            f"angle_control:               {angle_control:.2f}"
        ]

        print("\033[H\033[J", end="")  # ANSI escape code to clear the screen
        print("\n".join(info_lines))
    

###############################################################################################################



def main():
    
    speed_pid_left = PIDController(kp=450, ki=4000, kd=0, i_max = 550,d_max= 70, i_min = 0, pid_min = 0) #4500 16000	
    speed_pid_right = PIDController(kp=450, ki=4000, kd=0, i_max = 550, d_max= 70, i_min = 0, pid_min = 0)
    speed_pid_left.set_integral(0.00000000000000001)
    speed_pid_right.set_integral(0.00000000000000001)
    
    angle_pid = PIDController(kp=0.2, ki=0.000, kd=0)
    
    robot = Robot()
    lpf = LowPassFilter(1)

    mc = motoron.MotoronI2C()
    mc_right = motoron.MotoronI2C(address=17)

    mc.reinitialize()  
    mc.disable_crc()
    mc.clear_reset_flag()

    mc_right.reinitialize()  
    mc_right.disable_crc()
    mc_right.clear_reset_flag()

    mc.set_max_acceleration(1, 500)
    mc.set_max_deceleration(1, 500)
    mc.set_starting_speed(1,10)

    mc_right.set_max_acceleration(1, 500)
    mc_right.set_max_deceleration(1, 500)
    mc_right.set_starting_speed(1,10)

   # plotter = RealTimePlotter(time_window=10)
   # plotter.show()
    out = OutputManager()
    out.start_console_output()

    last_time = time.monotonic()
    angle_setpoint = 0
    base_speed = 0
    
    start_Time = time.monotonic()
    duration = np.inf
    

    try:
        while True:
            current_time = time.monotonic()
            time_step = current_time - last_time
            last_time = current_time
            
            if (current_time - start_Time) > duration:
                break
            
            x, y, theta = robot.get_position_and_angle()
            
            angle_setpoint, base_speed = handle_user_input(angle_setpoint, base_speed)


            # PID controller to adjust wheel velocities
            
            angle_control = angle_pid.update(angle_setpoint, theta, time_step)
            left_wheel_velocity = base_speed - angle_setpoint#- angle_control
            right_wheel_velocity = base_speed + angle_setpoint#+ angle_control
            
            
            if right_wheel_velocity == 0:
                speed_pid_right.set_integral(0.00000000000000001)
            right_motor_control = speed_pid_right.update(abs(right_wheel_velocity), robot.get_right_wheel_velocity(), time_step)
            mc_right.set_speed(1, int(-right_motor_control * np.sign(right_wheel_velocity)))
            #mc_right.set_speed(1, 100 * -int(base_speed))
            
            if left_wheel_velocity == 0:
                speed_pid_left.set_integral(0.00000000000000001)
            left_motor_control = speed_pid_left.update(abs(left_wheel_velocity), robot.get_left_wheel_velocity(), time_step)

            mc.set_speed(1, int(left_motor_control * np.sign(left_wheel_velocity)))
            #mc.set_speed(1, int(left_motor_control * np.sign(right_wheel_velocity)))
            
            robot.state_estimate(left_wheel_velocity, right_wheel_velocity)
           
            out.update_console_output(robot, left_wheel_velocity, right_wheel_velocity, base_speed, angle_setpoint, angle_control, speed_pid_right, speed_pid_left)

            #print_terminal(robot, left_wheel_velocity, right_wheel_velocity, base_speed, angle_setpoint, angle_control, speed_pid_right, speed_pid_left)

#             plotter.update_plot(current_time, 
#                                 robot.get_left_wheel_velocity(), 
#                                 robot.get_right_wheel_velocity(), 
#                                 left_wheel_velocity, 
#                                 right_wheel_velocity)
                


            #time.sleep(0.01)  # Ggf. die Schleifenfrequenz anpassen

    except KeyboardInterrupt:
        print("Messung beendet.")
        
    
    try:
        out.stop_console_output()
    
        # Plot anzeigen
        plt.figure(figsize=(10, 6))
        plt.plot(robot.times, robot.left_wheel_velocities, label="Left Wheel Velocity", color = 'b')
        plt.plot(robot.times, robot.right_wheel_velocities, label="Right Wheel Velocity", linestyle='-', color = 'r')
        plt.plot(robot.times, robot.left_wheel_velocity_targets, label="Left Wheel Target", linestyle='-.', color = 'c')
        plt.plot(robot.times, robot.right_wheel_velocity_targets, label="Right Wheel Target", linestyle='-.', color = 'm')
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.title("Wheel Velocity Over Time")
        plt.legend()
        plt.grid(True)
        plt.show()
    
    except KeyboardInterrupt:
        print("Plot Closed")
        


if __name__ == "__main__":
    main()

