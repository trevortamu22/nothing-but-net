import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, sqrt
from point_of_contact import end_point, mid_point
from end_velocities import end_velocities, mid_velocities
from sympy.solvers import solve
from sympy import Symbol
import sympy
import math
import csv
import time
###This code is SUUUPERRRR rough but it works in determining the best angle###



def equationroots(a, b, c):
 
    # calculating discriminant using formula
    dis = b * b - 4 * a * c
    sqrt_val = math.sqrt(abs(dis))
     
    # checking condition for discriminant
    #real and different roots
    if dis > 0:
        sol1 = ((-b + sqrt_val)/(2 * a))
        sol2 = ((-b - sqrt_val)/(2 * a))
        if abs(sol1) > abs(sol2):
            sol = sol2

        else:
            sol = sol1    
    #real and same roots
    elif dis == 0:
        
        sol = (-b / (2 * a))
     
    # when discriminant is less than 0
    #non-real numbers
    else:
        sol = None
    
    return sol


###THIS NEEDS TO BE IN FINAL PYTHON FILE###
#read in the pre-generated backboard angles and store them in a variable
board_angles = []
with open('backboard_angles.csv', mode="r") as f:
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



#read in the trajectory of the ball using sample data
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
z_coeff = np.polyfit(t[:limit], z[:limit], 1)



#brute force function
def brute_force(x_coeff, y_coeff, z_coeff):
    
    #establish hoop center
    hoop_center = [0, -0.2032, .2032]
    #determine intercepts at z=0 and vector components of each axis
    
    vel = end_velocities(x_coeff, y_coeff, z_coeff)    

    #create variable to store closest distances to center of hoop, intercepts, and angle of attack
    dist_center = []
    intercept_list = []
    input_sphere = []
    output_sphere = []
    input_vect = []
    output_vect = []
    index = 0

    for j in cart_ang:
        
        sol = 100
        #determine intercept of ball for the given normal angle of bboard
        #represent the trajectory of the ball in parametric equations, and the normal plane in terms of x, y, and z
        #solve for t when convergence occurs
        a = y_coeff[0]*j[1]
        b = x_coeff[0]*j[0] + y_coeff[1]*j[1] + z_coeff[0]*j[2]
        c = x_coeff[1]*j[0] + y_coeff[2]*j[1] + z_coeff[1]*j[2]
        sol = equationroots(a, b, c)
        '''t1 = Symbol('t1')
        Ball_Path = ((x_coeff[0]*t1 + x_coeff[1]), (y_coeff[0]*t1**2 + y_coeff[1]*t1 + y_coeff[2]), (z_coeff[0]*t1 + z_coeff[1]))
        Backboard_Angle = ((j[0]*Ball_Path[0]) + (j[1]*Ball_Path[1]) + (j[2]*Ball_Path[2]))
        t_sol = solve([Backboard_Angle], [t1])'''
        
        
        '''for i in t_sol:
            if type(i[0]) is not sympy.core.numbers.Float:
                sol = None
                break

            elif sol > abs(i[0]):
                sol = i[0]

            elif sol < abs(i[0]):
                sol = sol'''
        #print(sol)
        
        if sol == None:
            index += 1
            pass

        else:
            xyz_int = ((x_coeff[0]*sol + x_coeff[1]), (y_coeff[0]*sol**2 + y_coeff[1]*sol + y_coeff[2]), (z_coeff[0]*sol + z_coeff[1]))

            y_vel = mid_velocities(xyz_int[2], x_coeff, y_coeff, z_coeff)

            velocity = [-vel[0], -y_vel[1], -vel[2]]
            intercept = [xyz_int[0], xyz_int[1], xyz_int[2]]

            #these dont need to be here in final code, just used for visualization
            intercept_list.append(intercept)
            input_vect.append(velocity)

            #determine the angular components in spherical coordinates of the trajectory at the given intercept point
            '''this should be exactly the same as previous work'''
            r = sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
            in_phi = np.arccos(float(velocity[1]/r))
            in_theta = np.arctan(velocity[0] / velocity[2])
            
            input_sphere.append([in_phi, in_theta])
            #determine reflected trajectory
            '''convert from cartesian to spherical coordinates to get angle, compare angle to normal and determine output angle 
            convert output angle back to cartesian coordinates (already unit vector)'''
            phi, theta = board_angles[index]
            diff_ang_phi = phi - in_phi
            diff_ang_theta = theta - in_theta
            out_phi = phi + diff_ang_phi
            out_theta = theta + diff_ang_theta
            
            output_sphere.append([out_phi, out_theta])
            #compare reflected trajectory (unit vec) to vector of intercept point to hoop center, then perform pythagorean theorem to determine...
            #...closest distance to center of hoop (I dont think this is the best way to go about this but its the current potential solution)
            #smallest dist to center gets selected to be the angle of choice
            '''useful link: https://forum.unity.com/threads/how-do-i-find-the-closest-point-on-a-line.340058/
            also run the time it takes to determine only the closest distance and output'''
            x_out = sin(out_phi)*sin(out_theta)
            y_out = cos(out_phi)
            z_out = sin(out_phi)*cos(out_theta)
            
            output_vect.append([x_out, y_out, z_out])

            '''x_hoop = intercept[0]
            z_hoop = intercept[2] - hoop_center[2]
            y_hoop = intercept[1] - hoop_center[1]

            hoop_mag = sqrt(x_hoop**2 + y_hoop**2 + z_hoop**2)
            closest = np.dot([x_out, y_out, z_out], [x_hoop, y_hoop, z_hoop])

            dist = sqrt(hoop_mag**2 - closest**2)'''


            parameter = (hoop_center[1] - intercept[1]) / y_out
            x_plane = intercept[0] + x_out * parameter
            z_plane = intercept[2] + z_out * parameter

            dist = sqrt((x_plane - hoop_center[0])**2 + (z_plane - hoop_center[2])**2)

            dist_center.append(dist)
            index += 1
        #break
    
    return dist_center, intercept_list, input_vect, output_vect, input_sphere, output_sphere



'''dist_center, intercept_list, input_vect, output_vect = brute_force(x_coeff, y_coeff, z_coeff)

sort = sorted(dist_center)

solutions = []
contacts = []

iteration = 0
while iteration < 10:
    iteration += 1
    solutions.append(cart_ang[dist_center.index(sort[iteration])])
    contacts.append(intercept_list[dist_center.index(sort[iteration])])'''



#store intercepts and input velocities in csv files
'''with open('intercepts.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    for i in intercept_list:
        writer.writerow(i)
    file.close

with open('input_vectors.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    for j in input_vect:
        writer.writerow(j)
    file.close'''



# Create a 3D plot
'''fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x_val = []
y_val = []
z_val = []

for i in solutions:
    x, y, z = i
    x_val.append(x)
    y_val.append(y)
    z_val.append(z)

u_val = []
v_val = []
w_val = []

for j in contacts:
    u, v, w = j
    u_val.append(u)
    v_val.append(v)
    w_val.append(w)

ax.quiver(u_val, w_val, v_val, x_val, z_val, y_val, color='green', length=10, normalize=True) #"length" is the size of the vector arrow
ax.axes.set_xlim3d(left=-10, right=10) 
ax.axes.set_ylim3d(bottom=-10, top=10) 
ax.axes.set_zlim3d(bottom=-10, top=10)
ax.set_xlabel("Left Right")
ax.set_ylabel("Distance From Camera")
ax.set_zlabel("Height")
plt.show()

print(sort)'''