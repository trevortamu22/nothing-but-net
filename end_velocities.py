# -*- coding: utf-8 -*-
"""
Created on Thu Feb  9 11:53:52 2023

@author: alexs
"""

import numpy as np

# this script takes the basketball's position regression coefficients and equations as input,
# then calculates the velocity in each dimension of the ball when the ball is infintesimally close to the backboard

# The code within the two lines will be included in the master script. 
# They were put in this script so that I could have all the necessary variables.
# Only the function should be copied into the master script.
##############################################################################################
# import camera data
csv_data = np.loadtxt('arc1.csv', delimiter=',')
t = csv_data[:, 0]/(10**6)
t = t - t[0]
x = csv_data[:, 1]/1000
y = csv_data[:, 2]/1000
z = csv_data[:, 3]/1000

# set limit equal to length of time array
limit = len(t)

# calculate coefficients and regressio equations
x_coeff = np.polyfit(t[:limit], x[:limit], 1)
y_coeff = np.polyfit(t[:limit], -y[:limit], 2)
z_coeff = np.polyfit(t[:limit], z[:limit], 1)
# Px = np.polynomial.Polynomial(np.flip(x_coeff))
# Py = np.polynomial.Polynomial(np.flip(y_coeff))
# Pz = np.polynomial.Polynomial(np.flip(z_coeff))
###############################################################################################


def end_velocities(x_coeff, y_coeff, z_coeff):
    # find time hitting backboard
    time_hitting_backboard = np.roots(z_coeff)[0] - t[0]
    #print('time hitting ', time_hitting_backboard)
    
    # differentiate each position function w.r.t. time to find velocities
    x_vel = np.polyder(x_coeff)[0]
    #print('x coeff ', x_coeff)
    #print('x vel ', x_vel)
    y_vel = np.polyder(y_coeff)
    z_vel = np.polyder(z_coeff)[0]
    
    # we now have to substitute the time at which the ball contacts the backboard 
    # because the y velocity is a function of time
    y_vel = np.polyval(y_vel, time_hitting_backboard)
    
    # put velocities in an array
    ball_vel = np.array([x_vel, y_vel, z_vel])
    
    return ball_vel
    
#print(end_velocities(x_coeff, y_coeff, z_coeff))
    
    
    
# function that will calculate 3D velocity of ball at a desired z location
def mid_velocities(z_loc, x_coeff, y_coeff, z_coeff):
    # get time when ball is at z = desired location
    time_at_z = (z_loc + (z_coeff[0] * t[0]) - z_coeff[1]) / z_coeff[0]
    
    # differentiate each position function w.r.t. time to find velocities
    x_vel = np.polyder(x_coeff)[0]
    y_vel = np.polyder(y_coeff)
    z_vel = np.polyder(z_coeff)[0]

    # we now have to substitute the time at which the ball is at the desired z location 
    # because the y velocity is a function of time
    y_vel = np.polyval(y_vel, time_at_z)
    
    # put velocities in an array
    ball_vel = np.array([x_vel, y_vel, z_vel])
    
    return ball_vel
    
    
#z_loc = 4.275   # desired z location to analyze (in meters)
#print(mid_velocities(z_loc, x_coeff, y_coeff, z_coeff))
    
    
  