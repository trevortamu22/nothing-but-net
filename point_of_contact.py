# -*- coding: utf-8 -*-
"""
Created on Tue Feb  7 13:20:54 2023

@author: alexs
"""

import numpy as np

# this script takes the basketball's position regression coefficients and equations as input,
# then calculates the x and y position of the ball when the ball is at z = 0 (contacting unmoved backboard)

# The code within the two lines will be included in the master script. 
# They were put in this script so that I could have all the necessary variables.
# Only the function should be copied into the master script.
##############################################################################################
# import camera data
csv_data = np.loadtxt('arc1.csv', delimiter=',')
t = csv_data[:, 0]/(10**6)
t = t - t[0]
x = (csv_data[:, 1]/1000) + 0.1
y = (csv_data[:, 2]/1000) - 0.75
z = csv_data[:, 3]/1000

# set limit equal to length of time array
limit = len(t)

# calculate coefficients and regression equations
x_coeff = np.polyfit(t[:limit], x[:limit], 1)   # highest order term is first
y_coeff = np.polyfit(t[:limit], -y[:limit], 2)  # highest order term is first
z_coeff = np.polyfit(t[:limit], z[:limit], 1)   # highest order term is first
# Px = np.polynomial.Polynomial(np.flip(x_coeff))
# Py = np.polynomial.Polynomial(np.flip(y_coeff))
# Pz = np.polynomial.Polynomial(np.flip(z_coeff))
###############################################################################################


# function that will calculate the 3D point the ball is at when z = 0
def end_point(x_coeff, y_coeff, z_coeff):
    
    # get time when ball is at z = 0. We have to subract the initial time from the csv
    time_hitting_backboard = np.roots(z_coeff)[0] - t[0]
    # print(time_hitting_backboard)
    
    # calc x and y position when z = 0
    x_pos = np.polyval(x_coeff, time_hitting_backboard)
    y_pos = np.polyval(y_coeff, time_hitting_backboard)
    z_pos = np.polyval(z_coeff, time_hitting_backboard)
    # print(x_pos)
    # print(y_pos)
    # print(z_pos)
    ball_location = np.array([x_pos, y_pos, 0])
    return ball_location
    
#print(end_point(x_coeff, y_coeff, z_coeff))
    


# function that will calucate 3D point at a certain z value
def mid_point(z_loc, x_coeff, y_coeff, z_coeff):
    
    # get time when ball is at z = desired location
    time_at_z = (z_loc + (z_coeff[0] * t[0]) - z_coeff[1]) / z_coeff[0]
    #print(z_coeff[0], t[0], z_coeff[1])
    
    # calc x and y position when z = desired location
    x_pos = np.polyval(x_coeff, time_at_z)
    y_pos = np.polyval(y_coeff, time_at_z)
    # print(x_pos)
    # print(y_pos)
    # print(z_pos)
    ball_location = np.array([x_pos, y_pos, z_loc])
    return ball_location

z_loc = 0.049261   # z location to analyze (in meters)
#print(mid_point(z_loc, x_coeff, y_coeff, z_coeff))

