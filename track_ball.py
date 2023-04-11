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
    contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        center, radius = cv2.minEnclosingCircle(max(contours, key = cv2.contourArea))
        if radius >= min_radius:
            return int(center[0]), int(center[1]), int(radius)
    return -1000, -1000, -1000

def is_flying(pos):
    pos = np.array(pos)
    a = np.polyfit(pos[:,0], -pos[:,2], 2)[0]*10**9
    #if a < -4.35 and a > -5.45: print('Toss g =', round(-2*a, 2), 'm/s^2')
    return a < -4.35 and a > -5.45

def calculate_angle(pos):
    t = pos[:, 0]/(10**6)
    t = t - t[0]
    x = (pos[:, 1])/1000
    y = (pos[:, 2]-340)/1000
    z = (pos[:, 3]-265)/1000
    x_coeff = np.polyfit(t, x, 1)
    y_coeff = np.polyfit(t, -y, 2)
    z_coeff = np.polyfit(t, z, 1)
    
    #theta_z = 6.35 * (np.pi/180)
    #theta_x = -3.67 * (np.pi/180)
    #return (theta_z, theta_x) # Temp output (radians)
    
    return find_angles(x_coeff,y_coeff,z_coeff, True)

def main():
    # Set System Variables
    WHITE_BALANCE = 3500
    MIN_POINTS = 6
    
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
    print('White Balance:', WHITE_BALANCE)
    print('Using', MIN_POINTS, 'Flight Points')
    
    # Define the calibration object to allow coordinate transformations
    cal = Calibration.from_raw(k4a.calibration_raw, depth_cam_mode, color_res)
    
    # Open Serial Connection to system
    serial_con = motor_controller.connect()
    
    # Declare large arrays in an attempt to manage memory allocation
    hsv = np.zeros((1080,1920,3))
    mask = np.zeros((1080,1920))
    blur = np.zeros((1080,1920))
    capture_color = np.zeros((1080,1920,3), dtype=np.uint8)
    
    data_points = []
    times = [0]
    t_final = 0
    
    # Main loop (runs until Ctrl+C is pressed)
    while 1:
        print('\nWaiting for toss!',end='\r')
        while 1:
            t1 = time.process_time_ns()
            capture = k4a.get_capture()
            if np.any(capture.color):
                capture_color = (capture.color[:,:,:3])
                hsv = cv2.cvtColor(capture_color, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, np.array([3, 170, 80]), np.array([7, 255, 255]))
                blur = cv2.inRange(cv2.blur(mask, (35,35)), 30, 255)
                x, y, r = mask_to_circle(blur, 15)
                if x > -1000 and x < 2048 and y < 1536 and capture.transformed_depth[y,x] > 0:
                    data_points.append((capture.color_timestamp_usec, *cal.convert_2d_to_3d((x,y), capture.transformed_depth[y,x], pyk4a.CalibrationType.COLOR)))
                    #print(data_points[-1][1], -(data_points[-1][2]-340),data_points[-1][3]-265)
                    if(len(data_points) > MIN_POINTS):
                        if is_flying(data_points[-MIN_POINTS:]):
                            data_points = data_points[-MIN_POINTS:]
                            solution = calculate_angle(np.array(data_points))
                            if solution is not None:
                                steps = motor_steps(*solution, motor1_cal, motor2_cal, motor3_cal)
                                motor_controller.move(serial_con, *steps)
                                t_final = k4a.get_capture().color_timestamp_usec
                                break
                    times.append((time.process_time_ns()-t1)/(10**6))
        # Sleep system and return home
        time.sleep(1)
        motor_controller.returnhome(serial_con)
        print('Max time:', max(times), 'ms\tAverage Time:', sum(times)//len(times), 'ms\tTotal Time:', int((t_final-data_points[0][0])/1000), 'ms')
        np.savetxt('last_toss.csv', np.array(data_points), delimiter=",")
    k4a.stop()
    serial_con.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Terminating Program...')
        sys.exit(0)
