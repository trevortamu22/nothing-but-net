import csv
from cmath import pi, sin, cos
import numpy as np

#creates list corresponding to the normal vector of the backboard in spherical coordinates



#creates list of 20 points in vertical rotation, 40 points in horizontal rotation
'''phi = np.linspace(pi/2, pi*(3/4), 20)
theta = np.linspace(pi*(1/4), pi*(-1/4), 40)

with open(r'backboard_angles.csv', 'a', newline='') as f:
    for i in phi:
        for j in theta:
            combined = [i, j]
            writer = csv.writer(f)
            writer.writerow(combined)
f.close'''



#list of 90 points in vertical rotation only, constrained horizontal rotation
'''phi = np.linspace(pi/2, pi*(3/4), 90)
theta = 0

with open(r'backboard_angles_2d.csv', 'a', newline='') as f:
    for i in phi:
        combined = [i, theta]
        writer = csv.writer(f)
        writer.writerow(combined)
f.close'''