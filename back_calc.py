import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
from point_of_contact import end_point, mid_point
from end_velocities import end_velocities, mid_velocities

######################################################################################
#set hoop center
hoop_center = [0, 0.2032, -.2032]

#function that determines angle backboard needs to be at in the xz direction
#intercept contains xz coords of intercept, vect contains xz vectors at impact
def bb_ang_xz(intercept, velocity):
    #separate data into components
    x_int = intercept[0]
    z_int = intercept[1]







    x_vel = velocity[0]
    z_vel = velocity[1]

    #find vector to hoop
    x_hoop = x_int
    z_hoop = z_int - hoop_center[1]

    #determining unit vectors
    hoop_unit = [(z_hoop/(sqrt(x_hoop**2+z_hoop**2))), (x_hoop/(sqrt(x_hoop**2+z_hoop**2)))]
    vel_unit = [(z_vel/(sqrt(x_vel**2+z_vel**2))), (x_vel/(sqrt(x_vel**2+z_vel**2)))]

    #adding unit vectors and determining the resultant normal vector
    bb_norm_xz = [hoop_unit[0]+vel_unit[0], hoop_unit[1]+vel_unit[1]]
    bb_norm_xz = [(bb_norm_xz[0]/(sqrt(bb_norm_xz[0]**2+bb_norm_xz[1]**2))), (bb_norm_xz[1]/(sqrt(bb_norm_xz[0]**2+bb_norm_xz[1]**2)))]
    bb_norm_xz = [-1*bb_norm_xz[1], -1*bb_norm_xz[0]]

    return bb_norm_xz



#function that determines the angle the backboard needs to be at in the yz direction
#intercept contains yz coords of intercept, vect contains yz vectors at impact
def bb_ang_yz(intercept, velocity):
    #separate data into components
    y_int = intercept[0]
    z_int = intercept[1]
    
    y_vel = velocity[0]
    z_vel = velocity[1]
    
    #find vector to hoop
    y_hoop = y_int - hoop_center[2]
    z_hoop = z_int - hoop_center[1]
    
    #determining unit vectors
    hoop_unit = [(z_hoop/(sqrt(y_hoop**2+z_hoop**2))), (y_hoop/(sqrt(y_hoop**2+z_hoop**2)))]
    vel_unit = [(z_vel/(sqrt(y_vel**2+z_vel**2))), (y_vel/(sqrt(y_vel**2+z_vel**2)))]
    
    #adding unit vectors and determining the resultant normal vector
    bb_norm_yz = [hoop_unit[0]+vel_unit[0], hoop_unit[1]+vel_unit[1]]
    bb_norm_yz = [(bb_norm_yz[0]/(sqrt(bb_norm_yz[0]**2+bb_norm_yz[1]**2))), (bb_norm_yz[1]/(sqrt(bb_norm_yz[0]**2+bb_norm_yz[1]**2)))]
    bb_norm_yz = [-1*bb_norm_yz[1], -1*bb_norm_yz[0]]
    
    return bb_norm_yz



#function that determines new intercept of ball and bboard
#function takes normal vector of bboard, slope and intercept (at z=0 in vector form) of ball path in defined plane
def intercept_xz(normal, ball_vec, ball_intercept):
    #find the slope of the backboard from the normal vector
    bb_vec = [-normal[1], normal[0]]

    bb_slope = bb_vec[0]/bb_vec[1]
    bb_int = 0
    
    ball_slope = ball_vec[0]/ball_vec[1]
    ball_int = ball_intercept[0]
    
    z = (ball_int - bb_int) / (bb_slope - ball_slope)
    x = ball_slope * z + ball_int

    new_int = [x, z]

    return new_int


#function that determines how much translation the backboard needs to do in the z-axis as a result of the changing y-axis
#normal is the normal vector in y-z plane, and intercept corresponds to the y-z intercept of the ball trajectory with the backboard
def translate(normal, intercept):
    y = intercept[0]
    opp = normal[0]
    adj = normal[1]
    #just use trigonometry for determining displacement in z-direction
    z = -(opp/adj)*y
    
    return z



#optimization function
def optimization(x_coeff, y_coeff, z_coeff):
    #determine intercepts at z=0 and vector components of each axis
    start_intercept = end_point(x_coeff, y_coeff, z_coeff)
    vel = end_velocities(x_coeff, y_coeff, z_coeff)
    xz_intercept = [start_intercept[0], start_intercept[2]]
    #store slope of ball in x-z plane
    ball_slope = [vel[0], vel[2]]
    
    #set base value of backboard position
    old_xz_norm = [0, 1]
    iter = 0
    while True:
        iter += 1
        #recalculates backboard normal with new intercept
        new_norm = bb_ang_xz(xz_intercept, ball_slope)
        #compares the angle needed to the normal vector
        #if within 1 degree of each other, break
        if abs(old_xz_norm[0] - new_norm[0]) <= 0.0875:
            #determine velocity and intercept of y-axis
            y_vel = mid_velocities(xz_intercept[1], x_coeff, y_coeff, z_coeff)
            y_vel = y_vel[1]
            #print(y_vel)
            y_int = mid_point(xz_intercept[1], x_coeff, y_coeff, z_coeff)
            y_int = y_int[1]

            #calculate normal in y-z plane
            yz_norm = bb_ang_yz([y_int, xz_intercept[1]], [y_vel, vel[2]])
            
            #add the two angles and represent them as one normal vector
            magnitude = sqrt((old_xz_norm[0]**2)+(old_xz_norm[1]+yz_norm[1])**2+(yz_norm[0])**2)
            bboard = [old_xz_norm[0] / magnitude, (old_xz_norm[1]+yz_norm[1]) / magnitude, yz_norm[0] / magnitude]

            #determine translation in z-axis
            trans = translate(yz_norm, [y_int, [xz_intercept[1]]])
            trans = xz_intercept[1] - trans
            intercept = [xz_intercept[0], xz_intercept[1], y_int]
            print(intercept)
            return bboard, intercept, trans, iter
        
        #determine new intercept point of the ball to the backboard
        else:
            xz_intercept = intercept_xz(new_norm, ball_slope, xz_intercept)
            old_xz_norm = new_norm



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



#determine intercepts at z=0 and vector components of each axis
start_intercept = end_point(x_coeff, y_coeff, z_coeff)
vel = end_velocities(x_coeff, y_coeff, z_coeff)
xz_intercept = [start_intercept[0], start_intercept[2]]
#store slope of ball in x-z plane
ball_slope = [vel[0], vel[2]]

'''print("intercept", xz_intercept)
print("velocity", vel[0], vel[2])
print("bb normal unit", normal)
print("cramer", intercept(normal, ball_slope, xz_intercept))'''