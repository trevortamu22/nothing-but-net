import serial
import time

print("start")
# Connect to Arduino

def connect():
    ser = serial.Serial('COM3', 9600, timeout=1)
    time.sleep(0.5)
    return ser

def move(ser, m1,m2,m3): 
    # Protect Motor Limits
    print('Moving to', m1, m2, m3)
    if m1 >= 500: m1 = 500
    if m2 >= 500: m2 = 500
    if m3 >= 500: m3 = 500
    
    # Send command to move motors
    ser.write(b'Move\n')
    ser.write(str(m1).encode() + b'\n')
    ser.write(str(m2).encode() + b'\n')
    ser.write(str(m3).encode() + b'\n')
    

def returnhome(ser):
# Send command to return to home
    #move(ser, -global_m1, -global_m2, -global_m3)
    ser.write(b'ReturnHome\n')
    time.sleep(1)







# ser.write(b'Move\n')
# ser.write(b'0\n')
# ser.write(b'0\n')
# ser.write(b'0\n')
#end_time=time.time()
#total_time=end_time-start_time
#print("total time", total_time)
# Close connection to Arduino


