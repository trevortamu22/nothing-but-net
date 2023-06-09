''' Determines the correct motor steps for a given set of angles provided by find_angles.py
    -Trevor Sides 05/03/23'''

import numpy as np

# Function that calculates the necessary motor steps to make the desired backboard position happen.
# The inputs to this function are Blaine's theta_z and theta_x, and the three solution matrices from the motion study.
def motor_steps(theta_z, theta_x, motor1_array, motor2_array, motor3_array):
    # make an array with difference between Blaine's theta_x and my theta_x matrix. Then find the index of the minum difference
    # we will use the motor1_array here
    theta_x_difference_array = motor1_array[2] - theta_x
    theta_x_difference_array = np.absolute(theta_x_difference_array)
    index = np.argmin(theta_x_difference_array)
    
    # now divide that index by 2 becuase we will rotate motor 2 in the opposite direction. This way...
    # the theta_z effects cancel out and the theta_x effects add to each other
    # index = index // 2  # this needs to be an integer
    motor1_step_command = motor1_array[0][index]
    theta_z_motor1 = motor1_array[1][index]     #theta_z from motor 1 will be used later
    # motor2_step_command = -1*motor1_step_command
    motor2_step_command = 0
    

    # now control motor 3 in order to achieve the rest of theta_z that needs to be reached
    theta_z_from_1 = theta_z_motor1
    theta_z_needed = theta_z - theta_z_from_1
    
    # make an array with difference between Blaine's (brute_force/find_angles.py) theta_z and my theta_z matrix. The minumum value's...
    # index in this array corresonds to the motor position we need in the solution matrix
    theta_z_difference_array = motor3_array[1] - theta_z_needed
    theta_z_difference_array = np.absolute(theta_z_difference_array)
    index = np.argmin(theta_z_difference_array)
    motor3_step_command = motor3_array[0][index]
    
    # # make an array with difference between Blaine's theta_z and my theta_z matrix. The minumum value's...
    # # index in this array corresonds to the motor position we need in the solution matrix
    # theta_z_difference_array = motor3_array[1] - theta_z
    # theta_z_difference_array = np.absolute(theta_z_difference_array)
    # index = np.argmin(theta_z_difference_array)
    # motor3_step_command = motor3_array[0][index]
    
    
    motor3_step_command = motor3_step_command // 2
    motor1_step_command += -1*(motor3_step_command)
    motor2_step_command += -1*(motor3_step_command)
    
    
    return -motor1_step_command, -motor2_step_command, motor3_step_command
