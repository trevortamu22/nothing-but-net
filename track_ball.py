import cv2
import numpy as np

import pyk4a
from pyk4a import Config, PyK4A, Calibration
import time

import brute_force
import motor_controller

try:
    from wb_val import *
    auto_wb = int(auto_wb)
except ImportError:
    auto_wb = 2500

def mask_to_circle(img, min_radius):
    contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(capture, contours, -1, (255,255,255),3)
    #cv2.imshow("Contour", capture)
    if len(contours) > 0:
        center, radius = cv2.minEnclosingCircle(max(contours, key = cv2.contourArea))
        if radius >= min_radius:
            return int(center[0]), int(center[1]), int(radius)
    return -1000, -1000, -1000

def is_flying(pos):
    pos = np.array(pos)
    a = np.polyfit(pos[:,0], -pos[:,2], 2)[0]*10**9
    #print(pos.shape[0])
    return a < -4.35 and a > -5.45

def calculate_angle(pos):
    t = pos[:, 0]/(10**6)
    t = t - t[0]
    x = pos[:, 1]/1000
    y = pos[:, 2]/1000
    z = pos[:, 3]/1000
    x_coeff = np.polyfit(t[:limit], x[:limit], 1)
    y_coeff = np.polyfit(t[:limit], -y[:limit], 2)
    z_coeff = np.polyfit(t[:limit], z[:limit], 1)
    return brute_force.brute_force(x_coeff,y_coeff,z_coeff)


def main():
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
    print(auto_wb)
    # Set white balance for the color camera
    k4a.whitebalance = auto_wb
    #assert k4a.whitebalance == 4510

    # Define the calibration object to allow coordinate transformations
    cal = Calibration.from_raw(k4a.calibration_raw, depth_cam_mode, color_res)
    
    # Open Serial Connection to system
    serial_con = motor_controller.connect()
    
    # Declare large arrays in an attempt to manage memory allocation
    hsv = np.zeros((1080,1920,3))
    mask = np.zeros((1080,1920))
    blur = np.zeros((1080,1920))
    capture_color = np.zeros((1080,1920,3), dtype=np.uint8)
    

    # Adds 15-20 ms
    show_outline = False
    
    data_points = []
    all_data_points = []
    times = [0]
    in_flight = False
    min_points = 7
    last_time = 0
    #cuda_capture = cv2.cuda_GpuMat()
    #cuda_capture.upload(capture_color)
    
    # Main loop (runs until esc is pressed)
    while 1:
        t1 = time.process_time_ns()
        capture = k4a.get_capture()
        if np.any(capture.color):
            # Find circles   
            capture_color = (capture.color[:,:,:3])
            #np.copyto(capture_color, capture.color[:,:,:3], casting='no') # 0.25ms  
            #capture_color = np.array(capture.color[:,:,:3])

            #cuda_capture.upload(capture_color)
            #hsv = cv2.cuda.cvtColor(cuda_capture, cv2.COLOR_BGR2HSV).download() #31ms
            hsv = cv2.cvtColor(capture_color, cv2.COLOR_BGR2HSV)
            
            mask = cv2.inRange(hsv, np.array([3, 170, 80]), np.array([7, 255, 255]))
            #blur = cv2.blur(mask, (5,5))s
            blur = cv2.inRange(cv2.blur(mask, (35,35)), 30, 255)
            
            x, y, r = mask_to_circle(blur, 15)
            
            if x > -1000 and x < 2048 and y < 1536 and capture.transformed_depth[y,x] > 0:
                data_points.append((capture.color_timestamp_usec, *cal.convert_2d_to_3d((x,y), capture.transformed_depth[y,x], pyk4a.CalibrationType.COLOR)))
                if(not in_flight and len(data_points) > min_points):
                    in_flight = is_flying(data_points[-min_points:])
                    if in_flight:
                        data_points = data_points[-min_points:]
                        solution = calculate_angle(data_points[-min_points:])
                        if solution is not None:
                            steps = calc_motor_angles(solution) # This may change to a simple array call
                            motor_controller.move(serial_con, *steps)
                            break
                elif in_flight:
                    in_flight = is_flying(data_points)
                    if not in_flight and len(data_points) > 9: break
                if show_outline:
                    capture_color = np.array(capture_color)
                    if in_flight:
                        cv2.circle(capture_color, (x, y), r, (20,20,255), 3)
                    else:
                        cv2.circle(capture_color, (x, y), r, (20,255,20), 3)
                    cv2.circle(capture_color, (x, y), 2, (20,255,20), 3)
            times.append((time.process_time_ns()-t1)/(10**6))
            #if x > -1000 and x < 2048 and y < 1536 and capture.transformed_depth[y,x] == 0:
            #    capture_color = np.array(capture_color)
            #    cv2.circle(capture_color, (x, y), r, (20,20,255), 3)
            #    cv2.imshow("Main", cv2.resize(capture_color, (1024, 768)))
            #cv2.imshow("Mask", mask)
            #cv2.imshow("Blur", blur)#cv2.medianBlur(capture_color, 35))
            #cv2.imshow("Main", cv2.resize(capture_color, (1024, 768)))
             
            print(times[-1], sum(times)//len(times))
            key = cv2.waitKey(10)
            if key != -1:
                cv2.destroyAllWindows()
                break
            
            # Sleep system and return home
            time.sleep(3)
            motor_controller.returnhome(serial_con)
            
    #np.savetxt('output.csv', np.array(data_points[:-1]), delimiter=",")
    #np.savetxt('all_output.csv', np.array(all_data_points[:-1]), delimiter=",")
    k4a.stop()
    serial_con.close()


if __name__ == "__main__":
    main()