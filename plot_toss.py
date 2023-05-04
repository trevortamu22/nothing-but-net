''' Reads in two csv files (full_last_run.csv & last_toss.csv) and plots the full collection of data points in blue while plotting the selected points from the last shot in magenta.
    This is useful for tweaking shot selection parameters. Helps the user visualize "did the system select all of the relavent points?"
    Incorrect parameters in track_ball.py will lead to valid data being excluded from the trajectory regression.
    -Trevor Sides 05/03/23'''

import matplotlib.pyplot as plt
import numpy as np
from numpy.polynomial import Polynomial


# Limit the number of points used to calculate the regression (set to 0 for all points)
limit = 0

raw_points = np.loadtxt('full_last_run.csv', delimiter=',')
data_points = np.loadtxt('last_toss.csv', delimiter=',')

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
ax.set_xlabel("Horizontal Distance (m)")
ax.set_ylabel("Distance From Camera (m)")
ax.set_zlabel("Height (m)")
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
