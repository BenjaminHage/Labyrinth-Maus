import time
import motoron
import RPi.GPIO as GPIO
import math
import numpy as np
import keyboard  # Modul für die Handhabung von Tastatureingaben
from ADCDifferentialPi import ADCDifferentialPi
import matplotlib.pyplot as plt
from collections import deque
import threading

from controllers import PIDController
from io_management import OutputManager
from robot import DifferentialDriveRobot as Robot

        
################################################################################################################
################################################################################################################


def handle_user_input(angle_setpoint, base_speed, close):
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
    return angle_setpoint, base_speed, close


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
    #lpf = LowPassFilter(1)

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

    out = OutputManager()
    out.start_console_output()
    #out.start_rt_plot()

    last_time = time.monotonic()
    angle_setpoint = 0
    base_speed = 0
    close = False
    
    start_Time = time.monotonic()
    duration = np.inf
    

    try:
        while True:
            current_time = time.monotonic() - start_Time
            time_step = current_time - last_time
            last_time = current_time
            
            if ((current_time - start_Time) > duration) or close:
                break
            
            x, y, theta = robot.get_position_and_angle()
            
            angle_setpoint, base_speed, close = handle_user_input(angle_setpoint, base_speed, close)


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
            out.update_plot(current_time, 
                                robot.get_left_wheel_velocity(), 
                                robot.get_right_wheel_velocity(), 
                                left_wheel_velocity, 
                                right_wheel_velocity)
            


    except KeyboardInterrupt:
        print("main loop closed by keyboardInterupt")
        
    
    try:
        print("Messung beendet.")
       
        out.stop_console_output()
        
        out.aktivate_final_batch_plot()
        out.aktivate_final_rt_plot()
        
        out.show_final_plots(robot)
        
    
    except KeyboardInterrupt:
        print("Plot Closed by keyboardInterupt")
        
    print("\nend of programm")    
        


if __name__ == "__main__":
    main()

