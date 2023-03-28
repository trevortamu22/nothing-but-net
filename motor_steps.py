# -*- coding: utf-8 -*-
"""
Created on Thu Mar  2 11:53:26 2023

@author: alexs
"""

from sympy import *
import numpy as np
import matplotlib.pyplot as plt

# this script has a function that takes the backboard angles as input and 
# calculates the needed motor steps

# AS OF RIGHT NOW, THE POINT WHERE THE U-JOINT CONTACTS THE BACKBOARD IS THE ORIGIN

# set physical system's parameters here
motor_angle_range = [-90*(np.pi/180), 90*(np.pi/180)]   # radians
step_increment = np.pi / 180                            # radians
num_steps_possible = int((motor_angle_range[1] - motor_angle_range[0]) / step_increment)     # discrete number of steps the motors can do
l1 = 0.1397     # meter
l2 = 0.3048     # meter
p = l1/l2
h_t = 0.1621    # meter
h_lrx = 0.2413  # meter
h_lrz = 0.199   # meter
theta_x = 0.009
theta_z = 0.08

# make an array all possible angles the motors can be positioned to
angles_possible = np.linspace(motor_angle_range[0], motor_angle_range[1], num_steps_possible)   # radians
# print(num_steps_possible)
# print(angles_possible)

# make an array with each possible displacement hypotenuse as a function of motor angle
hypotenuse_array_all_motors = l1 * (np.cos(angles_possible) + (1/p) * np.sqrt(1-((p**2) * (np.sin(angles_possible))**2))) # Brooke's equation (in meters)
# print(len(hypotenuse_array_all_motors))

# make possible coordinate arrays for left motor for each motor angle


# make possible coordinate arrays for top motor for each motor angle
# x_array_top = np.zeros(num_steps_possible)
# z_array_top = l1 * (np.cos(angles_possible) + (1/p) * np.sqrt(1-((p**2) * (np.sin(angles_possible))**2))) # Brooke's equation (in meters)
# theta_z_array = np.arcsin(z_array_top / h_t)  # this is theta_z as a function of z for the top
# y_array_top = z_array_top * np.sin(theta_z_array)
# print(len(x_array_top))
# print(z_array_top)
# print(y_array_top)

# make possible coordinate arrays for right motor for each motor angle


# make a z axis array for all motors that has all possible contact point positions as a function of motor angle
z_array_all_motors = l1 * (np.cos(angles_possible) + (1/p) * np.sqrt(1-((p**2) * (np.sin(angles_possible))**2))) # Brooke's equation (in meters)
# print(z_array_all_motors)
# plt.plot(angles_possible, z_array_all_motors)

# make a y axis array for top motor that has contact point positions as a function of motor angle
# y_array_top = z_array_all_motors * np.tan()

# Function that takes angles the backboard makes with coordinate axes as input and calculates the positions of
# where each contact point needs to be in 3D space in order to catch the ball
def contact_points_loc(theta_z, theta_x, h_t, h_lrz, h_lrx):
    # left contact point location
    z_left = h_lrx * np.sin(theta_x)
    x_left = -1*h_lrx + np.abs((z_left * np.tan(theta_x)))  # absolute value because no matter which way the angle is, it will be adding in our coord system
    y_left = -1*h_lrz + np.abs(z_left * np.tan(theta_z))    # absolute value because no matter which way the angle is, it will be adding in our coord system
    left_pos = np.array([x_left, y_left, z_left])
    
    # top contact point location
    z_top = h_t * np.sin(theta_z)
    y_top = h_t - np.abs(z_top * np.tan(theta_z))           # absolute value because no matter which way the angle is, it will be subtracting in our coord system
    top_pos = np.array([0, y_top, z_top])
    
    # right contact point location
    z_right = -1 * h_lrx * np.sin(theta_x)
    x_right = h_lrx - np.abs(z_right * np.tan(theta_x))     # absolute value because no matter which way the angle is, it will be subtracting in our coord system
    y_right = -1*h_lrz + np.abs(z_right * np.tan(theta_z))  # absolute value because no matter which way the angle is, it will be adding in our coord system
    right_pos = np.array([x_right, y_right, z_right])
    
    return left_pos, top_pos, right_pos

# print(contact_points_loc(theta_z, theta_x, h_t, h_lrz, h_lrx))
# print('')
# print('')

# Function that calculates the necessary motor steps to make the desired backboard position happen.
# The inputs to this function are supposed to be 3 arrays with 3 elements: [x, y, z] of backboard contact points.
def motor_steps(left_pos, top_pos, right_pos, h_t, h_lrz, h_lrx, l1, l2):
    # set values to all home positions
    left_home_pos = np.array([-1*h_lrx, -1*h_lrz, 0])   # left contact point home position before shot [x, y, z]
    top_home_pos = np.array([0, h_t, 0])                # top contact point home position before shot [x, y, z]
    right_home_pos = np.array([h_lrx, -1*h_lrz, 0])     # right contact point home position before shot [x, y, z]
    
    # calculate needed distance to move in each dimension for each contact point
    left_needed_distance = left_pos - left_home_pos     # [x, y, z]
    top_needed_distance = top_pos - top_home_pos        # [x, y, z]
    right_needed_distance = right_pos - right_home_pos  # [x, y, z]
    
#################################################################################################### RETURNING IMAGINARY NUMBERS HERE
    # calculate angle needed for each motor
    M = symbols('M')
    func = l1 * (cos(M) + (1/p) * sqrt(1-((p**2) * (sin(M))**2))) - top_needed_distance[2]
    top_motor_angle = solve(func, M)
    print(top_motor_angle)
####################################################################################################
    
    return left_needed_distance, top_needed_distance, right_needed_distance

lpos = contact_points_loc(theta_z, theta_x, h_t, h_lrz, h_lrx)[0]
tpos = contact_points_loc(theta_z, theta_x, h_t, h_lrz, h_lrx)[1]
rpos = contact_points_loc(theta_z, theta_x, h_t, h_lrz, h_lrx)[2]
print(motor_steps(lpos, tpos, rpos, h_t, h_lrz, h_lrx, l1, l2))
