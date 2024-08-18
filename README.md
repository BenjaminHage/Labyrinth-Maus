# Labyrinth-Maus

sudo apt install git python3-dev python3-pip

sudo pip3 install smbus2 --break-system-packages
sudo raspi-config nonint do_i2c 0
i2cdetect -y 1

sudo apt remove python3-rpi.gpio
sudo apt update
sudo apt install python3-rpi-lgpio

git clone https://github.com/pololu/motoron-python.git
cd motoron-python
sudo python3 setup.py install #--break-system-packages

git clone https://github.com/abelectronicsuk/ABElectronics_Python_Libraries.git
cd ABElectronics_Python_Libraries
sudo python3 setup.py install #--break-system-packages

sudo pip3 install matplotlib --break-system-packages
sudo pip3 install keyboard --break-system-packages
sudo pip3 install scipy --break-system-package

chmod +x setup_pi_ap.sh

chmod +x git-push-script.sh








import math
import threading
import time

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

# Beispielverwendung in OutputManager

class OutputManager:
    def __init__(self):
        self.plotter = RealtimePlotter()
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

# Beispielverwendung im Hauptprogramm
def main():
    output_manager = OutputManager()
    robot = ...  # Dein Roboterobjekt
    left_wheel_velocity = ...
    right_wheel_velocity = ...
    base_speed = ...
    angle_setpoint = ...
    angle_control = ...
    pid_r = ...
    pid_l = ...

    # Start der Konsolenausgabe
    output_manager.start_console_output()

    # Hauptschleife
    while program_is_running:
        # Aktualisiere den Konsolenoutput
        output_manager.update_console_output(robot, left_wheel_velocity, right_wheel_velocity,
                                             base_speed, angle_setpoint, angle_control, pid_r, pid_l)

        # Aktualisiere das Echtzeit-Plotting
        current_time = time.monotonic()
        output_manager.update_plot(current_time, left_wheel_velocity, right_wheel_velocity,
                                   left_wheel_velocity_target, right_wheel_velocity_target)
        # Weitere Logik...

    # Beenden der Konsolenausgabe und Zeigen des finalen Plots
    output_manager.stop_console_output()
    output_manager.show_final_plot()


