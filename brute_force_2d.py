import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, sqrt
from point_of_contact import end_point, mid_point
from end_velocities import end_velocities, mid_velocities
from sympy.solvers import nsolve
from sympy import Symbol
import csv

#determines the intercept in the y-z plane for the backboard given the y and z coefficients of parametric equations, and the backboard angle to be evaluated
def intercept_yz(y_coeff, z_coeff, board_angle):
    t1, t2 = Symbol('t1 t2')
    Ball_Path = ((y_coeff[0]*t1**2 + y_coeff[1]*t1 + y_coeff[2]), (z_coeff[0]*t1 + z_coeff[1]))
    Backboard_Angle = (board_angle[0]*t2, -board_angle[1]*t2)
    t = nsolve([Ball_Path[0] - Backboard_Angle[0], Ball_Path[1] - Backboard_Angle[1]], [t1, t2], [0.1, 0.1])
    return t



###THIS NEEDS TO BE IN FINAL PYTHON FILE###
#read in the pre-generated backboard angles and store them in a variable
board_angles = []
with open('backboard_angles_2d.csv', mode="r") as f:
    read = csv.reader(f)
    for i in read:
        y = []
        for k in range(len(i)):
            y.append(float(i[k]))
        board_angles.append(y)
    f.close


###THIS ALSO NEEDS TO BE IN FINAL PYTHON FILE###
#convert the angles from spherical to cartesian
cart_ang = []
for i in board_angles:
    x = sin(i[0])*sin(i[1])
    y = cos(i[0])
    z = sin(i[0])*cos(i[1])
    cart_ang.append([x, y, z])



'''#read in the trajectory of the ball using sample data
###import camera data and format (more info in point_of_contact)###
csv_data = np.loadtxt('arc1.csv', delimiter=',')
t = csv_data[:, 0]/(10**6)
t = t-t[0]
x = csv_data[:, 1]/1000
y = (csv_data[:, 2]/1000) - 0.75
z = csv_data[:, 3]/1000

limit = len(t)

x_coeff = np.polyfit(t[:limit], x[:limit], 1)
y_coeff = np.polyfit(t[:limit], -y[:limit], 2)
z_coeff = np.polyfit(t[:limit], z[:limit], 1)'''



#brute force function
def brute_force_2d(x_coeff, y_coeff, z_coeff):
    #establish hoop center
    hoop_center = [0, -0.2032, .2032]
    #determine intercepts at z=0 and vector components of each axis
    start_intercept = end_point(x_coeff, y_coeff, z_coeff)
    vel = end_velocities(x_coeff, y_coeff, z_coeff)

    #create variable to store closest distances to center of hoop, intercepts, and angle of attack
    dist_center = []
    intercept_list = []
    input_vect = []
    output_vect = []
    index = 0

    for j in cart_ang:
        #determine intercept of ball for the given normal angle of bboard
        '''compare intercept between two lines for x-z intercept.
        the time at z position should tell y-intersept, verify this by determining intercept between parabola and line'''
        yz_int = intercept_yz(y_coeff, z_coeff, [cart_ang[1], cart_ang[2]])
        y_vel = mid_velocities(yz_int[1], x_coeff, y_coeff, z_coeff)
        y_vel = y_vel[1]

        
        velocity = [-y_vel, -vel[2]]
        intercept = [yz_int[0], yz_int[1]]

        #these dont need to be here in final code, just used for visualization
        '''intercept_list.append(intercept)
        input_vect.append(velocity)'''

        #determine the angular components in spherical coordinates of the trajectory at the given intercept point
        '''this should be exactly the same as previous work'''
        r = sqrt(velocity[0]**2 + velocity[1]**2)
        in_phi = np.arccos(velocity[1]/r)
        
        #determine reflected trajectory
        '''convert from cartesian to spherical coordinates to get angle, compare angle to normal and determine output angle through 
        addition(?) subtraction(?) <---need to figure this out, probably just some conditional statement.
        convert output angle back to cartesian coordinates (already unit vector)'''
        phi, theta = board_angles[index]
        diff_ang_phi = phi - in_phi
        out_phi = phi + diff_ang_phi
        
        #compare reflected trajectory (unit vec) to vector of intercept point to hoop center, then perform pythagorean theorem to determine...
        #...closest distance to center of hoop (I dont think this is the best way to go about this but its the current potential solution)
        #smallest dist to center gets selected to be the angle of choice
        '''useful link: https://forum.unity.com/threads/how-do-i-find-the-closest-point-on-a-line.340058/
        Choose the ten(?) smallest distances and plot them for verification
        also run the time it takes to determine only the closest distance and output'''
        y_out = cos(out_phi)
        z_out = sin(out_phi)
        
        output_vect.append([y_out, z_out])

        z_hoop = intercept[2] - hoop_center[2]
        y_hoop = intercept[1] - hoop_center[1]

        hoop_mag = sqrt(y_hoop**2 + z_hoop**2)
        closest = np.dot([y_out, z_out], [y_hoop, z_hoop])

        dist = sqrt(hoop_mag**2 - closest**2)
        dist_center.append(dist)
        index += 1
    return dist_center



