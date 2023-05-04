''' This is the main system script. All other required scripts are called by this script. The system requires a lot of software to be installed on the host machine.
    The most important and complicated package to install is pyk4a. pyk4a is the python wrapper that allows communication with the Kinect Azure camera.
    pyk4a does not play well with anaconda so run python bare metal. pyk4a can also have trouble installing with pip and may need to be installed manually following the github instructions.
    pyk4a also requires the Kinect Azure SDK and drivers. If you can run the Microsoft Kinect Azure Viewer program you have the SDK and drivers installed correctly.
    
    Aside from pyk4a you are going to need a couple packages that can be installed with "pip install <package>" in your pc cmd
    These packages are opencv-python, pyserial, and numpy.
    Extra packages such as matplotlib may be required to run supplementary scripts such as demo_draw.py or plot_toss.py
    
    The only files REQUIRED to run the system are track_ball.py, find_angles.py, motor_controller.py, motor_steps.py, and the motion study csv files (calibration_data)
    The arduino must be flashed with the arduino code and the correct COM port must be set. Typically it is COM3 but it may vary by machine
    The Kinect Azure requires a USB connection to the host machine and power from it's provided power brick
    
    Run the script using the cmd or a .bat file and "python track_ball.py"
    
    Written using Python 3.11.1
    
    -Trevor Sides 05/03/23
'''

import cv2
import numpy as np

import pyk4a
from pyk4a import Config, PyK4A, Calibration
import time
import sys

import motor_controller
from motor_steps import motor_steps
from find_angles import find_angles

def mask_to_circle(img, min_radius):
    # Find all contours in the image
    contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        # Enclose the biggest contour in the smallest possible circle
        center, radius = cv2.minEnclosingCircle(max(contours, key = cv2.contourArea))
        # If the biggest object does not meet the min_radius it likely is noise and the ball is not in the frame
        if radius >= min_radius:
            return int(center[0]), int(center[1]), int(radius)
    return -1000, -1000, -1000

# Checks if the ball is flying by comparing the downward acceleration to the gravitational constant given a provided tolerance
def is_flying(pos, thresh):
    pos = np.array(pos)
    a = np.polyfit(pos[:,0], -pos[:,2], 2)[0]*10**9
    return abs(4.9+a) < thresh


def calculate_angle(pos):
    # correct time so t0 = 0 and time is in seconds
    t = pos[:, 0]/(10**6)
    t = t - t[0]
    # change position units to meters from mm and change (0,0,0) from the camera to the expected location on the physical system
    x = (pos[:, 1])/1000
    y = (pos[:, 2]-340)/1000
    z = (pos[:, 3]-265)/1000
    # Perform a regression on the data points to find the kinematic equations for the ball
    x_coeff = np.polyfit(t, x, 1)
    y_coeff = np.polyfit(t, -y, 2)
    z_coeff = np.polyfit(t, z, 1)
    
    # Pass the motion tracking results to the backboard solver
    return find_angles(x_coeff,y_coeff,z_coeff, True)

def main():
    # Set System Variables
    WHITE_BALANCE = 3500 # Fix color temperature of the room. Allows colors to be in the expected range
    MIN_POINTS = 5 # How many of the most recent data points to check for flight
    START_FLIGHT_TOL = 2.5 # How close does the grav. const. have to be to 9.8 for the ball to be "in flight"
    LOCK_FLIGHT_TOL = 0.75 # How close does the grav. const. have to be to 9.8 for the trajectory to be accurate enough to attempt backboard calcs
    
    # Import motor calibration data
    motor1_cal = np.loadtxt('calibration_data/motor1_cal.csv', delimiter=",", unpack=True, encoding="utf-8-sig")
    motor2_cal = np.loadtxt('calibration_data/motor2_cal.csv', delimiter=",", unpack=True, encoding="utf-8-sig")
    motor3_cal = np.loadtxt('calibration_data/motor3_cal.csv', delimiter=",", unpack=True, encoding="utf-8-sig")
    
    # Define which camera modes to use (color res and depth FOV)
    color_res = pyk4a.ColorResolution.RES_1536P
    depth_cam_mode = pyk4a.DepthMode.NFOV_UNBINNED
    fps_mode = pyk4a.FPS.FPS_30
    
    # Define the camera object
    k4a = PyK4A(
        Config(
            color_resolution=color_res,
            depth_mode=depth_cam_mode,
            camera_fps=fps_mode,
            synchronized_images_only=True,
        )
    )
    k4a.start()
    
    # Set white balance for the color camera
    k4a.whitebalance = WHITE_BALANCE
    # Annouce System Parameters
    print('White Balance:', WHITE_BALANCE)
    print('Using Minimum', MIN_POINTS, 'Flight Points')
    
    # Define the calibration object to allow coordinate transformations
    cal = Calibration.from_raw(k4a.calibration_raw, depth_cam_mode, color_res)
    
    # Open Serial Connection to system
    serial_con = motor_controller.connect()
    
    # Declare large arrays in an attempt to manage memory allocation
    hsv = np.zeros((1080,1920,3))
    mask = np.zeros((1080,1920))
    blur = np.zeros((1080,1920))
    capture_color = np.zeros((1080,1920,3), dtype=np.uint8)
    
    # Initialize data arrays
    data_points = []
    all_points = []
    t_final = 0
    
    # Main loop (runs until Ctrl+C is pressed)
    while 1:
        in_flight = False # If the system is returning to attention the ball should not be thrown or in flight
        times = [0] # variable tracking average computation times
        print('\nWaiting for toss!',end='\r')
        while 1:
            t1 = time.process_time_ns()
            capture = k4a.get_capture() # Get frame from camera
            if np.any(capture.color):
                capture_color = (capture.color[:,:,:3]) # trim unwanted data from frame
                hsv = cv2.cvtColor(capture_color, cv2.COLOR_BGR2HSV) # convert frame to HSV for masking
                mask = cv2.inRange(hsv, np.array([3, 170, 80]), np.array([7, 255, 255])) # define/apply the mask (color range of ball in HSV)
                blur = cv2.inRange(cv2.blur(mask, (35,35)), 30, 255) # blur the mask slightly to remove noise and improve accuracy of detection
                x, y, r = mask_to_circle(blur, 15) # Find the ball from the mask
                # Confirm that the coordinates are valid (within the camera's resolution)
                if x > -1000 and x < 2048 and y < 1536 and capture.transformed_depth[y,x] > 0:
                    # Convert the data point from camera coordinates to real world coordinates.
                    # Combine the time of the data point with the coordinates and add to running list of active data_points
                    data_points.append((capture.color_timestamp_usec, *cal.convert_2d_to_3d((x,y), capture.transformed_depth[y,x], pyk4a.CalibrationType.COLOR)))
                    # Also add the point to the list of all points captured without recalculating anything
                    all_points.append(data_points[-1])
                    # Check that the active data points contains at least MIN_POINTS so it can attempt regression
                    if(len(data_points) >= MIN_POINTS):
                        # if the ball is not in flight check to see if it should now be considered in flight
                        if not in_flight and is_flying(data_points[-MIN_POINTS:], START_FLIGHT_TOL):
                            in_flight = True
                            data_points = data_points[-MIN_POINTS:] # Trim active data points to only include points that are in flight
                        if is_flying(data_points, LOCK_FLIGHT_TOL): # if the grav. const. is acceptable, attempt to find a solution. else keep collecting to improve traj. accuracy
                            solution = calculate_angle(np.array(data_points)) # find solution
                            if solution is not None:
                                # if a solution exists find the motor steps then output them on the serial port to the arduino. else keep collecting data, more accuracy may lead to a valid shot
                                steps = motor_steps(*solution, motor1_cal, motor2_cal, motor3_cal)
                                motor_controller.move(serial_con, *steps) # output steps to arduino over serial
                                t_final = k4a.get_capture().color_timestamp_usec
                                print('Used', len(data_points), 'to find solution!')
                                break
                        elif len(data_points) > MIN_POINTS and not is_flying(data_points, START_FLIGHT_TOL): # Catch if the ball managed to get out of flight without ever finding a valid shot.
                            in_flight = False
                    times.append((time.process_time_ns()-t1)/(10**6))
        # Sleep system and return home
        time.sleep(1) # Sleep gives time for the shot to be made once the angle is set
        motor_controller.returnhome(serial_con) # return the board home to get ready for the next shot
        print('Max time:', max(times), 'ms\tAverage Time:', sum(times)//len(times), 'ms\tTotal Time:', int((t_final-data_points[0][0])/1000), 'ms')
        np.savetxt('last_toss.csv', np.array(data_points), delimiter=",") # output the most recent selected data points
        np.savetxt('full_last_run.csv', np.array(all_points), delimiter=",") # output all data points collected
    # Close connction to camera and arduino
    k4a.stop()
    serial_con.close()

# Start the main function and properly handle a keyboard interrupt
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Terminating Program...')
        sys.exit(0)
