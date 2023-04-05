import numpy as np


###THIS NEEDS TO BE IN FINAL PYTHON FILE###
#read in the pre-generated backboard angles and store them in a variable
board_angles = np.loadtxt('backboard_angles.csv', delimiter=",", unpack=True, encoding="utf-8-sig").T
#convert the angles from spherical to cartesian
cart_ang = []
for i in board_angles:
    x = np.sin(i[0])*np.sin(i[1])
    y = np.cos(i[0])
    z = np.sin(i[0])*np.cos(i[1])
    cart_ang.append([x, y, z])
 

def solve_all_roots(coeffs):
    #Solves all quadratics simultaneously
    a = coeffs[:,0]
    b = coeffs[:,1]
    c = coeffs[:,2]
    sqrt_val = np.sqrt(b**2-4*a*c)
    sol1 = ((-b + sqrt_val)/(2 * a))
    sol2 = ((-b - sqrt_val)/(2 * a))
    # Time must be positive filter
    sol1_nan = np.where(sol1>0, sol1, np.nan)
    sol2_nan = np.where(sol2>0, sol2, np.nan)
    # Use smallest time solution
    return np.fmin(sol1_nan, sol2_nan)   
    
    
def end_velocities(x_coeff, y_coeff, z_coeff):
    # find time hitting backboard
    contact_time = np.roots(z_coeff)[0] 
    
    # we now have to substitute the time at which the ball contacts the backboard 
    # because the y velocity is a function of time
    y_vel = 2*y_coeff[0]*contact_time+y_coeff[1]
    
    # output velocities as an array
    return np.array([x_coeff[0], y_vel, z_coeff[0]])


#brute force function
def brute_force(x_coeff, y_coeff, z_coeff):
    
    #establish hoop center
    hoop_center = [[0], [-0.2032], [.2032]] # Lots of brackets force proper shape for repeat
    vel = end_velocities(x_coeff, y_coeff, z_coeff)    
    coeff = np.array([[0, *x_coeff],[*y_coeff],[0, *z_coeff]])
    ang_coeffs = np.matmul(cart_ang,coeff) # all quadratic coeffs
    sols = solve_all_roots(ang_coeffs) # Finds all roots (contact time)
    t_vec = np.stack((sols**2, sols, np.ones(len(sols)))) # [t^2, t, 1] vector for contact time
    xyz_ints = np.matmul(coeff, t_vec) # May give real answer for a NaN solution. Position at contact time
    y_vels = np.matmul(y_coeff, np.stack((2*sols, np.ones(len(sols)), np.zeros(len(sols))))) # y_vel = 2*y0+y1
    velocities = np.stack((np.repeat(-vel[0], len(sols)), -y_vels, np.repeat(-vel[2], len(sols)))) # [-vel[0], -y_vel, -vel[2]] all input vectors
    rs = np.sqrt((velocities**2).sum(axis=0))
    out_angles = board_angles - np.stack((np.arccos(-y_vels/rs), np.repeat(vel[0]/vel[2], len(sols)))).T + board_angles
    sin_out_phi = np.sin(out_angles[:,0]) # Store to save time. May be faster to recalc.
    output_vectors = np.stack((sin_out_phi*np.sin(out_angles[:,1]), np.cos(out_angles[:,0]), sin_out_phi*np.cos(out_angles[:,1])))
    hoop_pos = xyz_ints - np.repeat(hoop_center, len(sols), axis = 1)
    hoop_mags = (hoop_pos**2).sum(axis=0) # should sqrt but it just gets undone
    close_vecs = (output_vectors*hoop_pos).sum(axis=0)
    dists = np.sqrt(hoop_mags - close_vecs**2)
    
    return dists.tolist(), xyz_ints.T, velocities.T, output_vectors.T


def find_angles(x_coeff, y_coeff, z_coeff):
    dist, _, _, _ = brute_force(x_coeff, y_coeff, z_coeff)
    return board_angles[dist.index(min(dist))]
