import time
import math
import numpy as np

from controllers import PIDController
import io_management as io
from robot import DifferentialDriveRobot as Robot

        
################################################################################################################
################################################################################################################


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
    
    out = io.OutputManager()
    out.start_console_output()
    #out.start_rt_plot()

    
    angle_setpoint = 0
    base_speed = 0
    close = False
    
    last_time = time.monotonic()
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
            
            angle_setpoint, base_speed, close = io.handle_user_input(angle_setpoint, base_speed, close)


            # PID controller to adjust wheel velocities
            
            angle_control = angle_pid.update(angle_setpoint, theta, time_step)
            left_wheel_velocity = base_speed - angle_setpoint#- angle_control
            right_wheel_velocity = base_speed + angle_setpoint#+ angle_control
            
            if right_wheel_velocity == 0:
                speed_pid_right.set_integral(0.00000000000000001)
            right_motor_control = speed_pid_right.update(abs(right_wheel_velocity), robot.get_right_wheel_velocity(), time_step)
            robot.set_right_motor(int(-right_motor_control * np.sign(right_wheel_velocity)))
            
            if left_wheel_velocity == 0:
                speed_pid_left.set_integral(0.00000000000000001)
            left_motor_control = speed_pid_left.update(abs(left_wheel_velocity), robot.get_left_wheel_velocity(), time_step)
            robot.set_left_motor(int(left_motor_control * np.sign(left_wheel_velocity)))
            
            
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
        
    print("\nEnd of Program")    
        


if __name__ == "__main__":
    main()

