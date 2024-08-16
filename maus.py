import time
import motoron
import RPi.GPIO as GPIO
import math
import numpy as np
import keyboard  # Modul für die Handhabung von Tastatureingaben
from ADCDifferentialPi import ADCDifferentialPi
import matplotlib.pyplot as plt
#import matplotlib.animation as animation --break-system-packages

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

        #self.lpf_sensors = [LowPassFilter(cutoff_freq=3) for _ in self.sensor_angles]
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

    # def filter_sensor_readings(self, sensor_readings, time_step):
    #     filtered_readings = []
    #     for i, reading in enumerate(sensor_readings):
    #         filtered_reading = self.lpf_sensors[i].filter(reading, time_step)
    #         filtered_readings.append(filtered_reading)
        
    #     return filtered_readings 

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

    def __init__(self, kp, ki, kd, i_max = np.inf, d_max = np.inf):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.imax = i_max
        self.d_max = d_max
        self.previous_error = 0 #überlegung den setzbar zumachen falls der die probleme macht
        self.integral = 0

    def update(self, setpoint, measurement, time_step):
        error = setpoint - measurement
        self.integral += error * time_step
        derivative = (error - self.previous_error) / time_step
        self.previous_error = error
        
        P = self.kp * error
        I = self.ki * self.integral
        D = self.kd * derivative
        
        
        if I < -self.imax:
            I = -self.imax
            self.integral = -self.imax / self.ki
        elif I > self.imax:
            I = self.imax
            self.integral = self.imax / self.ki
        
        if D < -self.d_max:
            D = -self.d_max
        elif D > self.d_max:
            D = self.d_max
        
        
        return P + I + D
    
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



###############################################################################################################



def main():
    
    speed_pid_left = PIDController(kp=2000, ki=310, kd=0, i_max = 800) #4500 1600
    speed_pid_right = PIDController(kp=2000, ki=310, kd=0, i_max = 800)
    speed_pid_left.set_integral(1)
    speed_pid_right.set_integral(1)
    
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
                speed_pid_right.set_integral(1)
            right_motor_control = speed_pid_right.update(abs(right_wheel_velocity), robot.get_right_wheel_velocity(), time_step)
            mc_right.set_speed(1, int(-right_motor_control * np.sign(right_wheel_velocity)))
            #mc_right.set_speed(1, 100 * -int(base_speed))
            
            if left_wheel_velocity == 0:
                speed_pid_left.set_integral(1)
            left_motor_control = speed_pid_left.update(abs(left_wheel_velocity), robot.get_left_wheel_velocity(), time_step)
            mc.set_speed(1, int(left_motor_control * np.sign(left_wheel_velocity)))
            #mc.set_speed(1, int(left_motor_control * np.sign(right_wheel_velocity)))
            
            robot.state_estimate(left_wheel_velocity, right_wheel_velocity)
            info = [
                f"---------------------------------------------------------------------",
                f"Left Wheel Velocity:         {robot.get_left_wheel_velocity():.2f} m/s",
                f"Left Wheel Velocity target:  {left_wheel_velocity:.2f} m/s",
                f"left_motor_control:          {left_motor_control:.2f}",
                f"Right Wheel Velocity:        {robot.get_right_wheel_velocity():.2f} m/s",
                f"Right Wheel Velocity target: {right_wheel_velocity:.2f} m/s",
                f"Base_Speed:                  {base_speed:.2f} m/s",
                f"angle:                       {math.degrees(theta):.2f} °",
                f"angle_setpoint:              {math.degrees(angle_setpoint):.2f} °",
                f"angle_control:               {angle_control:.2f}",
                f"ADC_Values:                  {robot.get_formatted_sensor_readings(4)}"
            ]
            print("\n".join(info))
                


            #time.sleep(0.01)  # Ggf. die Schleifenfrequenz anpassen

    except KeyboardInterrupt:
        print("Messung beendet.")
    
    try:
    
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

#################################################################################################################

    
# Plot anzeigen
#plt.figure(figsize=(10, 6))
#plt.plot(robot.times, robot.left_wheel_velocities, label="Left Wheel Velocity")
#plt.plot(robot.times, robot.right_wheel_velocities, label="Right Wheel Velocity", linestyle='--')
#plt.xlabel("Time (s)")
#plt.ylabel("Velocity (m/s)")
#plt.title("Wheel Velocity Over Time")
#plt.legend()
#plt.grid(True)
#plt.show()
