import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, sqrt
from point_of_contact import end_point, mid_point
from end_velocities import end_velocities, mid_velocities
from back_calc import intercept_xz
import csv
###This code is SUUUPERRRR rough but it works in determining the best angle###



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
def brute_force(x_coeff, y_coeff, z_coeff):
    #establish hoop center
    hoop_center = [0, -0.2032, .2032]
    #determine intercepts at z=0 and vector components of each axis
    start_intercept = end_point(x_coeff, y_coeff, z_coeff)
    vel = end_velocities(x_coeff, y_coeff, z_coeff)
    xz_intercept = [start_intercept[0], start_intercept[2]]
    #store slope of ball in x-z plane
    ball_slope = [vel[0], vel[2]]

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
        xz_int = intercept_xz([j[0], j[2]], ball_slope, xz_intercept)
        y_vel = mid_velocities(xz_int[1], x_coeff, y_coeff, z_coeff)
        y_vel = y_vel[1]

        y_int = mid_point(xz_int[1], x_coeff, y_coeff, z_coeff)
        y_int = y_int[1]
        
        velocity = [-vel[0], -y_vel, -vel[2]]
        intercept = [xz_int[0], y_int, xz_int[1]]

        #these dont need to be here in final code, just used for visualization
        intercept_list.append(intercept)
        input_vect.append(velocity)

        #determine the angular components in spherical coordinates of the trajectory at the given intercept point
        '''this should be exactly the same as previous work'''
        r = sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
        in_phi = np.arccos(velocity[1]/r)
        in_theta = np.arctan(velocity[0] / velocity[2])
        
        #determine reflected trajectory
        '''convert from cartesian to spherical coordinates to get angle, compare angle to normal and determine output angle through 
        addition(?) subtraction(?) <---need to figure this out, probably just some conditional statement.
        convert output angle back to cartesian coordinates (already unit vector)'''
        phi, theta = board_angles[index]
        diff_ang_phi = phi - in_phi
        diff_ang_theta = theta - in_theta
        out_phi = phi + diff_ang_phi
        out_theta = theta + diff_ang_theta
        
        #compare reflected trajectory (unit vec) to vector of intercept point to hoop center, then perform pythagorean theorem to determine...
        #...closest distance to center of hoop (I dont think this is the best way to go about this but its the current potential solution)
        #smallest dist to center gets selected to be the angle of choice
        '''useful link: https://forum.unity.com/threads/how-do-i-find-the-closest-point-on-a-line.340058/
        Choose the ten(?) smallest distances and plot them for verification
        also run the time it takes to determine only the closest distance and output'''
        x_out = sin(out_phi)*sin(out_theta)
        y_out = cos(out_phi)
        z_out = sin(out_phi)*cos(out_theta)
        
        output_vect.append([x_out, y_out, z_out])

        x_hoop = intercept[0]
        z_hoop = intercept[2] - hoop_center[2]
        y_hoop = intercept[1] - hoop_center[1]

        hoop_mag = sqrt(x_hoop**2 + y_hoop**2 + z_hoop**2)
        closest = np.dot([x_out, y_out, z_out], [x_hoop, y_hoop, z_hoop])

        dist = sqrt(hoop_mag**2 - closest**2)
        dist_center.append(dist)
        index += 1
    return dist_center, intercept_list, input_vect, output_vect



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