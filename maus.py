import argparse
import time
import math
import numpy as np
import traceback

from controllers import PIDController
from controllers import AutonomousController
from controllers import ESCController
import io_management as io
from robot import DifferentialDriveRobot as Robot



def main():
    
    parser = argparse.ArgumentParser(description="Robot control script")
    parser.add_argument('-f', '--filename', type=str, default=None, help='Name der Datei mit den parametern für die Distanzmessung')
    args = parser.parse_args()
    
    
    # Initialisiere den Roboter mit dem gegebenen Dateinamen (falls angegeben)
    if args.filename:
        robot = Robot(param_file=args.filename)
    else:
        robot = Robot()
        

    
    ###### Manuell #####
    speed_pid_left = PIDController(kp=700, ki=3500, kd=0, i_max=550, d_max=70, i_min=0, pid_min=-100, pid_max=600)
    speed_pid_right = PIDController(kp=700, ki=3500, kd=0, i_max=550, d_max=70, i_min=0, pid_min=-100, pid_max=600)
    speed_pid_left.set_integral(0.00000000000000001)
    speed_pid_right.set_integral(0.00000000000000001)
    
    #angle_pid = PIDController(kp=150, ki=100.0, kd=10.00, d_minmax=100, i_minmax=100)
    angle_pid = PIDController(kp=0.045, ki=0.00, kd=0.0, d_minmax=0.029, i_minmax=0.1, pid_minmax=1)
    ###### Manuell #####


    ###### Auto #####
    
    #Auto
    init_base_speed = 0.15
    init_base_rotation_speed = 0.6
    desired_distance = 9  # Desired distance from the wall
    sensor_activation_threshold = 35 #= robot.get_sensor_range() * 0.75 
    direkt_change_toleranz = 5
    
    #ESC
    dither_frequency = 0.0007  # Frequency of dither signal (Hz)
    dither_amplitude = 1.3   # Amplitude of dither signal
    learning_rate = 0.4  # Learning rate for ESC
    esc_angle_comparison_interval = 1
    esc_angel_toleranz = 1
    
    point_distance_pid = PIDController(kp=-0.55, ki=-0.0, kd=0, i_minmax=100, d_max=70, pid_minmax=0.5)
    wall_distance_pid = PIDController(kp=0.25, ki=0.000, kd=0.3, d_minmax=0.029, i_minmax=1)
    esc = ESCController(dither_frequency, dither_amplitude, learning_rate)
    
    auto = AutonomousController(angle_pid, wall_distance_pid, point_distance_pid, esc, init_base_speed,
                                        init_base_rotation_speed, desired_distance, sensor_activation_threshold,
                                        robot.get_wheel_distance(), robot.get_robot_radius(),robot.get_sensor_angles())
     
    ###### Auto #####
   
    out = io.OutputManager()
    #out.start_console_output()
    #out.start_rt_plot()


    print("start")
    angle_setpoint = 0
    base_speed = 0
    close = False
    autonomous_mode = True
    
    
    start_Time = time.monotonic() 
    last_time = time.monotonic() - start_Time
    duration = np.inf
    angle_control = 0

    try:
        while True:
            current_time = time.monotonic() - start_Time
            time_step = current_time - last_time
            last_time = current_time
            
            if ((current_time - start_Time) > duration) or close:
                break
            
            x, y, theta, v = robot.get_position_and_angle()
            ukf_x, ukf_y, ukf_theta, ukf_v = robot.get_ukf_position_and_angle()
#             print(f"x    : {x:8.4f}    y    : {x:8.4f}    o    : {theta:8.4f}    v    : {v:8.4f}")
#             print(f"ukf_x: {ukf_x:8.4f}    ukf_y: {ukf_y:8.4f}    ukf_o: {ukf_theta:8.4f}    ukf_v: {ukf_v:8.4f}")
#             
            
            
            sensor_readings = robot.get_sensor_readings()
            sensor_readings = robot.filter_sensor_readings(sensor_readings, time_step)
            imu_gyro_readings = robot.get_imu_readings()
            gyro_w = imu_gyro_readings[2]
            
            angle_setpoint, base_speed, close, autonomous_mode = io.handle_user_input(angle_setpoint, base_speed,autonomous_mode, close=close)
                        
            if autonomous_mode:
                left_wheel_velocity_target, right_wheel_velocity_target = auto.autonomous_control_right_hand(sensor_readings, x, y, theta, gyro_w, current_time, time_step)
                #print(sensor_readings)
            else:
                # PID controller to adjust wheel velocities
                if abs(angle_setpoint - theta) <= math.radians(1.5):
                    angle_pid.set_integral(0)
                angle_control = np.sign(angle_pid.previous_error)* 0.01 *(init_base_rotation_speed - abs(gyro_w)) + angle_pid.update(angle_setpoint, theta, time_step)
                
                left_wheel_velocity_target = base_speed   - angle_control #- angle_setpoint
                right_wheel_velocity_target = base_speed  + angle_control #+ angle_setpoint 
                
                        
            
            if abs(right_wheel_velocity_target) <= 0.00:
                speed_pid_right.set_integral(0.00)
            right_motor_control = speed_pid_right.update(abs(right_wheel_velocity_target), robot.get_right_wheel_velocity(), time_step)
            #right_motor_control = speed_pid_right.update(abs(base_speed), abs(v), time_step)
                
                
            if abs(left_wheel_velocity_target) <= 0.00:
                speed_pid_left.set_integral(0.00)
            left_motor_control = speed_pid_left.update(abs(left_wheel_velocity_target), robot.get_left_wheel_velocity(), time_step)
            #left_motor_control = speed_pid_left.update(abs(base_speed), abs(v), time_step)
            
                
            
            robot.set_right_motor(int((-right_motor_control * np.sign(right_wheel_velocity_target)))) #- angle_control))
            robot.set_left_motor(int((left_motor_control * np.sign(left_wheel_velocity_target))))# - angle_control))
            
           
           
            robot.state_estimate(left_wheel_velocity_target, right_wheel_velocity_target, gyro_w, current_time, time_step)

            out.update_console_output(robot, left_wheel_velocity_target, right_wheel_velocity_target, base_speed, angle_setpoint,
                                      angle_control, speed_pid_right, speed_pid_left, sensor_readings)
            out.update_plot(current_time, 
                            robot.get_left_wheel_velocity(), 
                            robot.get_right_wheel_velocity(), 
                            left_wheel_velocity_target, 
                            right_wheel_velocity_target)
            
            
            

    except KeyboardInterrupt:
        print("\nmain loop closed by keyboardInterupt")
        
        
    except Exception as e:
        print(f"\033[91m{type(e).__name__} during main Loop:  {e}\033[0m")
        traceback.print_exc()
        
    
        
    try:
        robot.set_right_motor(0) 
        robot.set_left_motor(0)
            
        print("Messung beendet.")
        out.stop_console_output()
        out.aktivate_final_batch_plot()
        out.aktivate_final_rt_plot()
        out.show_final_plots(robot)
        
    except KeyboardInterrupt:
        print("\nPlot Closed by keyboardInterupt")
        
    print("\nEnd of Program")    

if __name__ == "__main__":
    main()
