''' This file runs the point selection code on a provided dataset of points collected by the camera.
    This is useful for seeing if different parameters would have allowed a specific shot be be calculated better.
    Most of the code is copied and pasted from elsewhere and this is not a critical program so it is not commented.
    -Trevor Sides 05/03/23'''

import matplotlib.pyplot as plt
import numpy as np
from numpy.polynomial import Polynomial
from find_angles import find_angles

raw_points = np.loadtxt('full_last_run.csv', delimiter=',')

MIN_POINTS = 5
START_FLIGHT_TOL = 2.5
LOCK_FLIGHT_TOL = 0.55

def is_flying(pos, thresh):
    pos = np.array(pos)
    a = np.polyfit(pos[:,0], -pos[:,2], 2)[0]*10**9
    return abs(4.9+a) < thresh


def calculate_angle(pos):
    t = pos[:, 0]/(10**6)
    t = t - t[0]
    x = (pos[:, 1])/1000
    y = (pos[:, 2]-340)/1000
    z = (pos[:, 3]-265)/1000
    x_coeff = np.polyfit(t, x, 1)
    y_coeff = np.polyfit(t, -y, 2)
    z_coeff = np.polyfit(t, z, 1)
    
    
    return find_angles(x_coeff,y_coeff,z_coeff, True)

data_points = []
in_flight = False

for i in range(len(raw_points)):
    data_points.append(raw_points[i])
    if(len(data_points) >= MIN_POINTS):
        if not in_flight and is_flying(data_points[-MIN_POINTS:], START_FLIGHT_TOL):
            in_flight = True
            data_points = data_points[-MIN_POINTS:]
        if is_flying(data_points, LOCK_FLIGHT_TOL):
            solution = calculate_angle(np.array(data_points))
            if solution is not None:
                print('Used', len(data_points), 'to find solution!')
                break
        elif len(data_points) > MIN_POINTS and not is_flying(data_points, START_FLIGHT_TOL):
            in_flight = False

# Limit the number of points used to calculate the regression (set to 0 for all points)
limit = 0

print(len(data_points), 'points used!')
data_points = np.array(data_points)
print((data_points[-1,0]-data_points[0,0])/1000, 'ms')
start_index = np.where(raw_points[:, 0] == data_points[0, 0])[0][0]
all_points = np.concatenate((raw_points[:start_index, :], raw_points[start_index+len(data_points):, :]), axis=0)

t = data_points[:, 0]/(10**6)
x = data_points[:, 1]/1000
y = data_points[:, 2]/1000
z = data_points[:, 3]/1000
xa = all_points[:, 1]/1000
ya = all_points[:, 2]/1000
za = all_points[:, 3]/1000

# Setup figure and plot scatter data
fig = plt.figure()
ax = plt.axes(projection = '3d')
ax.scatter(xa, za, -ya, color='c', label="raw data")
ax.scatter(x, z, -y, color='m', label="selected arc")
ax.set_xlabel("Left Right")
ax.set_ylabel("Distance From Camera")
ax.set_zlabel("Height")
ax.axes.set_xlim3d(left=-1.5, right=0.5) 
ax.axes.set_ylim3d(bottom=0, top=3) 
ax.axes.set_zlim3d(bottom=-1.5, top=1) 

# Calculate line of fit and plot
t_fit = np.linspace(min(t), max(t), 100)

if limit <= 2:
    limit = len(t)

Px = Polynomial(np.flip(np.polyfit(t[:limit], x[:limit], 1)))
Py = Polynomial(np.flip(np.polyfit(t[:limit], -y[:limit], 2)))
Pz = Polynomial(np.flip(np.polyfit(t[:limit], z[:limit], 1)))
ax.plot(Px(t_fit), Pz(t_fit), Py(t_fit), color="green", label="proj. path")

print(np.polyfit(t[:limit], -y[:limit], 2)[0])

plt.show()