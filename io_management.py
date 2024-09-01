import time
import matplotlib.pyplot as plt
from collections import deque
import threading
import math
import keyboard  # Modul für die Handhabung von Tastatureingaben





class OutputManager:
    def __init__(self):
        self.plotter = None
        self.console_output = None
        
        self.final_rt_plot_aktive = False
        self.final_batch_plot_aktive = False

  
    def aktivate_final_rt_plot(self):
        self.final_rt_plot_aktive = True

  
    def aktivate_final_batch_plot(self):
        self.final_batch_plot_aktive = True

  
    def start_console_output(self,auto_controller=None, extra_thread=True):
        self.console_output = ConsoleOutput(auto_controller)
        if extra_thread:
            self.console_output.start()

  
    def stop_console_output(self):
        if self.console_output is not None:
            self.console_output.stop()

  
    def start_rt_plot(self, rtp_window_size = 10):
        self.plotter = RealTimePlotter(rtp_window_size)

  
    def stop_rt_plot(self):
        if self.plotter is not None:
            self.plotter.stop()

  
    def show_final_rt_plot(self):
        if self.plotter is not None:
            self.plotter.show_final_plot()

  
    def update_console_output(self, robot, left_wheel_velocity, right_wheel_velocity,
                              base_speed, angle_setpoint, angle_control, pid_r, pid_l, sensor_readings,
                              auto_active=False):
        
        if self.console_output is not None:
            self.console_output.update(robot, left_wheel_velocity, right_wheel_velocity,
                                       base_speed, angle_setpoint, angle_control, pid_r, pid_l, sensor_readings,
                                       auto_active=auto_active)

  
    def update_plot(self, current_time, left_wheel_velocity, right_wheel_velocity,
                    left_wheel_velocity_target, right_wheel_velocity_target):
        if self.plotter is not None:
            self.plotter.update_plot(current_time, left_wheel_velocity, right_wheel_velocity,
                                     left_wheel_velocity_target, right_wheel_velocity_target)

  
    def show_final_plots(self, robot = None):
        plt.ioff()
        if not self.final_rt_plot_aktive:
            self.stop_rt_plot()
        elif not self.final_batch_plot_aktive:
            self.show_final_rt_plot()

        if self.final_batch_plot_aktive:
            self.show_batch_plot(robot)

    def show_batch_plot(self,robot):
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



class ConsoleOutput:
    def __init__(self,auto_controller):
        self.is_running = False
        self.console_thread = None
        self.info_lines = []
        self.auto = auto_controller

  
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
            time.sleep(0.05)  # Aktualisiere die Konsole alle 1 Sekunde

  
    def update(self, robot, left_wheel_velocity, right_wheel_velocity, base_speed,
               angle_setpoint, angle_control, pid_r, pid_l, sensor_readings,
               auto_active=False):
        """Aktualisiert die Informationen für den Konsolenoutput."""
        x, y, theta, v = robot.get_position_and_angle()

        if auto_active:
            
            front_sensor = sensor_readings[self.auto.front]/100
            left_sensor = sensor_readings[self.auto.left]/100
            front_left_sensor = sensor_readings[self.auto.front_left]/100
            right_sensor = sensor_readings[self.auto.right]/100
            front_right_sensor = sensor_readings[self.auto.front_right]/100
            
            self.info_lines = [
            "---------------------------------------------------------------------",
            f"State:             {self.auto.state}",
            f"follow_sensor:     {self.auto.follow_sensor}",
          #  f"Desired_distance:  {self.desired_distance}",
          #  f"aktivation_thres:  {self.activation_threshold}",
          #  f"Base_Speed:        {self.base_speed:.2f} m/s",
            "",
            f"front_sensor:      {front_sensor:.4f}",
            f"front_left_sensor: {front_left_sensor:.4f}    {self.auto.front_left_sensor_active}",
            f"left_sensor:       {left_sensor:.4f}    {self.auto.left_sensor_active}",
            f"front_right_sensor:{front_right_sensor:.4f}    {self.auto.front_right_sensor_active}",
            f"right_sensor:      {right_sensor:.4f}    {self.auto.right_sensor_active}",
            "",
         #   f"Right Wheel Velocity:        {self.auto.right_wheel_velocity:.2f} m/s",
         #   f"left Wheel Velocity:        {self.auto.left_wheel_velocity:.2f} m/s",
         #   "",     
         #   f"Angle:                       {math.degrees(theta):.2f} °",
         #   f"Angle_setpoint:              {math.degrees(self.auto.angle_setpoint):.2f} °",
         #   f"x: {x} y:{y}   target_x: {self.auto.target_x} targe_y: {self.auto.target_y}",
         #   "",
            f"{self.auto.control_message}"
            ]
        else:
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
                f"Angle_control:               {angle_control:.2f}",
                f"Sensor Readings:	{robot.get_formatted_sensor_readings(sensor_readings, decimal_places=2)}",
                "",
                #f"Percent:       {robot.get_power_percentage():3.0f}%"
                
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


        self.show()
        plt.pause(0.0000000000001)  # Pause für eine kurze Zeit, um den Plot zu aktualisieren

  
    def show(self):
        plt.show(block=False)  # Blockieren des Hauptprogramms vermeiden

  
    def show_final_plot(self):
        plt.ioff()
        plt.show()

  
    def stop(self):
        plt.close(self.fig)  # Schließe den Plot, wenn er nicht mehr benötigt wird

  
    def __del__(self):
        self.stop()




def handle_user_input(angle_setpoint, base_speed, autonomous_mode, close = False):
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
    if keyboard.is_pressed('c'):
        close = True
    if keyboard.is_pressed('a'):
        autonomous_mode = not autonomous_mode
        
    return angle_setpoint, base_speed, close, autonomous_mode

