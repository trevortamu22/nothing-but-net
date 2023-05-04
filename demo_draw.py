''' Reads in a csv (last_toss.csv) and simulates the system response. The simulation is presented as a 3D matplotlib plot.
    This simulation runs the backboard solution algorithm (find_angles.py) and must be given the proper parameters
    -Trevor Sides 05/03/23'''

import matplotlib.pyplot as plt
import numpy as np
from numpy.polynomial import Polynomial
from math import sin, cos, pi
import csv
from find_angles import brute_force
import time


# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

#######################  Plotting the Rectangle ###############################
# Define the rectangle vertices
#the dimensions of the backboard are 23in x 16in
vertices = [(0.2921, 0, 0.2032), (-0.2921, 0, 0.2032), (-0.2921, 0, -0.2032),(0.2921, 0, -0.2032), (0.2921, 0, 0.2032)] 

def new_vertices(normal, rad_x, rad_y):
    x = rad_x * normal[2]
    z_x = -rad_x * normal[0]
    z_y= -rad_y * normal[1]
    y = rad_y * normal[2]

    points = [(x, (z_x + z_y), y), (-x, (-z_x + z_y), y), (-x, (-z_x - z_y), -y),(x, (z_x - z_y), -y), (x, (z_x + z_y), y)]

    return points
trans = 0
# Define the rectangle edges
edges = [(0, 1), (1, 2), (2, 3), (3, 0)] #list of tuples to iterate through

# Plot the rectangle edges
for edge in edges:
    x_bb = [vertices[edge[0]][0], vertices[edge[1]][0]]
    y_bb = [vertices[edge[0]][1], vertices[edge[1]][1]]
    z_bb = [vertices[edge[0]][2], vertices[edge[1]][2]]
    ax.plot(x_bb, y_bb, z_bb, color='black')

#########################  Plotting the Circle ################################
# Define the circle center and radius
center = (0, 0.165, -0.27) #hoop located level with bottom of backboard, 1 radius out
radius = 0.1016 #meters (8in)

# Define the number of points used to plot the circle
num_points = 100
# Plot the circle outline
theta = np.linspace(0, 2*np.pi, num_points)
x_circ = center[0] + radius*np.cos(theta)
y_circ = center[1] + radius*np.sin(theta)
z_circ = center[2] + np.zeros_like(theta)
ax.plot(x_circ, y_circ, z_circ, color='red')

###########################  3D Plot Settings  ################################
# Set the plot limits and labels
ax.axes.set_xlim3d(left=-1, right=1) 
ax.axes.set_ylim3d(bottom=0, top=2) 
ax.axes.set_zlim3d(bottom=-0.75, top=1)
ax.set_xlabel("Horizontal Distance (m)")
ax.set_ylabel("Distance From Camera (m)")
ax.set_zlabel("Height (m)")


# import csv data and adujust coordinate system from camera coordinates to hoop coordinates
csv_data = np.loadtxt('last_toss.csv', delimiter=',')

t = csv_data[:, 0]/(10**6)
t = t - t[0]
x = (csv_data[:, 1]/1000)
y = (csv_data[:, 2]/1000)-.34
z = csv_data[:, 3]/1000-.265

limit = 0
if limit < 2:
    limit = len(t)

# calculate coefficients and regression equations
x_coeff = np.polyfit(t[:limit], x[:limit], 1)
y_coeff = np.polyfit(t[:limit], -y[:limit], 2)
z_coeff = np.polyfit(t[:limit], z[:limit], 1)

#Plot scatter data, along with a point corresponding to the center of the hoop
ax.scatter(x, z, -y, c=t, label="raw data")
ax.scatter(center[0], center[1], center[2])

t_final = np.roots(z_coeff)[0] - t[0]

# Calculate line of fit and plot
t_fit = np.linspace(min(t), (t_final), 100)

Px = Polynomial(np.flip(np.polyfit(t[:limit], x[:limit], 1)))
Py = Polynomial(np.flip(np.polyfit(t[:limit], -y[:limit], 2)))
Pz = Polynomial(np.flip(np.polyfit(t[:limit], z[:limit], 1)))
ax.plot(Px(t_fit), Pz(t_fit), Py(t_fit), color="green", label="fitted data")

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
    x_cart = sin(i[0])*sin(i[1])
    y_cart = cos(i[0])
    z_cart = sin(i[0])*cos(i[1])
    cart_ang.append([x_cart, y_cart, z_cart])


start = time.perf_counter()
#run brute force function and store list of intercepts, input vectors, and output vectors
dist_center, intercept_list, input_vect, output_vect, _, valid = brute_force(x_coeff, y_coeff, z_coeff, True)
fin = time.perf_counter()
tot_time = (fin - start)
print(f"time to optimize: {tot_time: 0.6f} seconds")
#determine position in list of closest reflection
min_dist = np.nanmin(dist_center, initial=1, where=valid)
if min_dist > 0:
    index = dist_center.index(min_dist)
    print('Index:', index)
else:
    index = 0
#store intercept of closest impact
x_int, y_int, z_int = intercept_list[index]
#store backboard angle of closest impact
x_board, y_board, z_board = cart_ang[index]
#store input trajectory of closest impact
x_in, y_in, z_in = input_vect[index]
#store output trajectory of closest impact
x_out, y_out, z_out = output_vect[index]

def rad2deg(list):
    new_list = []
    for i in list:
        new_list.append(i * (180/pi))
    return new_list


# Set the origin point of the vectors
origin = [x_int, z_int, y_int]
#origin = [x_int, z_int, y_int]
########################  Plotting Vectors #################################
# Plot the ball trajectory vector
ax.quiver(*origin, x_in, z_in, y_in, color='green', length=0.25, normalize=True) #"length" is the size of the vector arrow

#plot the vector of the reflected trajectory
ax.quiver(*origin, x_out, z_out, y_out, color='blue', length=0.75, normalize=True) 

#plot the resultant normal vector corresponding to new plane angle
ax.quiver(*origin, x_board, z_board, y_board, color='red', length=0.25, normalize=True) 

#draw the new bboard plane based on optimized position
bb_rad_x = 0.2921
bb_rad_y = 0.2032
vertices = new_vertices(cart_ang[index], bb_rad_x, bb_rad_y)

for edge in edges:
    x_bb = [vertices[edge[0]][0], vertices[edge[1]][0]]
    y_bb = [vertices[edge[0]][1], vertices[edge[1]][1]]
    z_bb = [vertices[edge[0]][2], vertices[edge[1]][2]]
    ax.plot(x_bb, y_bb, z_bb, color='red')

plt.show()
