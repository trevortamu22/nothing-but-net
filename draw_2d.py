import matplotlib.pyplot as plt
import numpy as np
from numpy.polynomial import Polynomial
from math import sin, cos
import csv
from brute_force_2d import brute_force_2d


# Define the circle center and radius
center = (0.2032, -.2032) #hoop located level with bottom of backboard, 1 radius out
rad_y = 0.2032

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111)


###########################  2D Plot Settings  ################################
# Set the plot limits and labels
ax.axes.set_xlim(left=-1, right=1) 
ax.axes.set_ylim(bottom=-2, top=2) 
ax.set_xlabel("Distance From Camera")
ax.set_ylabel("Height")



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

#Plot scatter data, along with a point corresponding to the center of the hoop
ax.scatter(z, -y, c=t, label="raw data")
ax.scatter(center[0], center[1])

t_final = np.roots(z_coeff)[0] - t[0]

# Calculate line of fit and plot
t_fit = np.linspace(min(t), (t_final), 100)

Px = Polynomial(np.flip(np.polyfit(t[:limit], x[:limit], 1)))
Py = Polynomial(np.flip(np.polyfit(t[:limit], -y[:limit], 2)))
Pz = Polynomial(np.flip(np.polyfit(t[:limit], z[:limit], 1)))
ax.plot(Pz(t_fit), Py(t_fit), color="green", label="fitted data")

#function that shows backboard position
def new_vertices(normal, rad_y):
    z_y= -rad_y * normal[0]
    y = rad_y * normal[1]

    points = [(z_y, y), (-z_y, -y)]

    return points


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
    x_cart = sin(i[0])*sin(i[1])
    y_cart = cos(i[0])
    z_cart = sin(i[0])*cos(i[1])
    cart_ang.append([x_cart, y_cart, z_cart])



#run brute force function and store list of intercepts, input vectors, and output vectors
dist_center, intercept_list, input_vect, output_vect = brute_force_2d(x_coeff, y_coeff, z_coeff)

#determine position in list of closest reflection
index = dist_center.index(min(dist_center))

#store intercept of closest impact
y_int, z_int = intercept_list[index]
#store backboard angle of closest impact
x_board, y_board, z_board = cart_ang[index]
#store input trajectory of closest impact
y_in, z_in = input_vect[index]
#store output trajectory of closest impact
y_out, z_out = output_vect[index]



# Set the origin point of the vectors
intersection = [float(z_int), float(y_int)]
#origin = [x_int, z_int, y_int]
print(intersection)
print(y_in, z_in)
########################  Plotting Vectors #################################
# Plot the ball trajectory vector
ax.quiver(*intersection, float(z_in), float(y_in), color='green')

#plot the vector of the reflected trajectory
ax.quiver(*intersection, float(z_out), float(y_out), color='blue') 

#plot the resultant normal vector corresponding to new plane angle
ax.quiver(*intersection, float(z_board), float(y_board), color='red') 

new_vert = new_vertices([y_board, z_board], rad_y)
y_vertices = []
z_vertices = []

for i in new_vert:
    z_vertices.append(i[0])
    y_vertices.append(i[1])

ax.plot(z_vertices, y_vertices)

plt.show()
print('distance to center at closest point: ', dist_center[index])
