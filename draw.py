import matplotlib.pyplot as plt
import numpy as np
from numpy.polynomial import Polynomial
from point_of_contact import end_point, mid_point
from end_velocities import end_velocities, mid_velocities
from back_calc import optimization
import time

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

#######################  Plotting the Rectangle ###############################
# Define the rectangle vertices
#the dimensions of the backboard are 23in x 16in
vertices = [(0.2921, 0, 0.2032), (-0.2921, 0, 0.2032), (-0.2921, 0, -0.2032),(0.2921, 0, -0.2032), (0.2921, 0, 0.2032)] 

def new_vertices(normal, rad_x, rad_y, trans):
    x = rad_x * normal[1]
    z_x = -rad_x * normal[0]
    z_y= -rad_y * normal[2]
    y = rad_y * normal[1]

    points = [(x, (z_x + z_y) + trans, y), (-x, (-z_x + z_y) + trans, y), (-x, (-z_x - z_y) + trans, -y),(x, (z_x - z_y) + trans, -y), (x, (z_x + z_y) + trans, y)]

    return points

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
center = (0, 0.2032, -.2032) #hoop located level with bottom of backboard, 1 radius out
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
ax.set_xlabel("Left Right")
ax.set_ylabel("Distance From Camera")
ax.set_zlabel("Height")

start = time.perf_counter()
#plot the scatter and line of best fit
csv_data = np.loadtxt('arc1.csv', delimiter=',')

t = csv_data[:, 0]/(10**6)
t = t - t[0]
x = (csv_data[:, 1]/1000) + 0.1
y = (csv_data[:, 2]/1000) - 0.9
z = csv_data[:, 3]/1000

limit = 0
if limit < 2:
    limit = len(t)

# calculate coefficients and regression equations
x_coeff = np.polyfit(t[:limit], x[:limit], 1)
y_coeff = np.polyfit(t[:limit], -y[:limit], 2)
z_coeff = np.polyfit(t[:limit], z[:limit], 1)

#define the initial intercepts of the ball on the backboard
x_int, y_int, z_int = end_point(x_coeff, y_coeff, z_coeff)

# Define the vector components
x_vec, y_vec, z_vec = end_velocities(x_coeff, y_coeff, z_coeff)

t_final = np.roots(z_coeff)[0] - t[0]

new_norm, intercept, trans, iter = optimization(x_coeff, y_coeff, z_coeff)
fin = time.perf_counter()
time = (fin - start)
print(f"time to optimize: {time: 0.6f} seconds")
print('number of iterations:', iter)
print(trans)
# Setup figure and plot scatter data
ax.scatter(x, z, -y, c=t, label="raw data")

# Calculate line of fit and plot
t_fit = np.linspace(min(t), (t_final), 100)

Px = Polynomial(np.flip(np.polyfit(t[:limit], x[:limit], 1)))
Py = Polynomial(np.flip(np.polyfit(t[:limit], -y[:limit], 2)))
Pz = Polynomial(np.flip(np.polyfit(t[:limit], z[:limit], 1)))
ax.plot(Px(t_fit), Pz(t_fit), Py(t_fit), color="green", label="fitted data")


# Set the origin point of the vectors
origin = [intercept[0], intercept[1], intercept[2]]
#origin = [x_int, z_int, y_int]
########################  Plotting Vectors #################################
# Plot the ball trajectory vector
ax.quiver(*origin, -x_vec, -y_vec, -z_vec, color='green', length=0.25, normalize=True) #"length" is the size of the vector arrow

#plot the vector to the center of hoop from point ball hits on backboard
ax.quiver(*origin, (center[0]-intercept[0]), (center[1]-intercept[1]), (center[2]-intercept[2]), color='blue', length=0.25, normalize=True) 

#plot the resultant normal vector corresponding to new plane angle
ax.quiver(*origin, new_norm[0], new_norm[1], new_norm[2], color='red', length=0.25, normalize=True) 

#draw the new bboard plane based on optimized position
bb_rad_x = 0.2921
bb_rad_y = 0.2032
vertices = new_vertices(new_norm, bb_rad_x, bb_rad_y, trans)

for edge in edges:
    x_bb = [vertices[edge[0]][0], vertices[edge[1]][0]]
    y_bb = [vertices[edge[0]][1], vertices[edge[1]][1]]
    z_bb = [vertices[edge[0]][2], vertices[edge[1]][2]]
    ax.plot(x_bb, y_bb, z_bb, color='red')

print('normal:', new_norm)
print('hoop vector:', [(center[0]-intercept[0]), (center[1]-intercept[1]), (center[2]-intercept[2])])
print('ball trajectory:', [x_vec, y_vec, z_vec])

# Set the initial view of the plot
#ax.view_init(30, 750)
ax.view_init(azim=0, elev=90)
plt.show()
