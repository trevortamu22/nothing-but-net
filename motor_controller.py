import serial
import time

print("start")
# Connect to Arduino

def connect():
    return serial.Serial('COM3', 9600, timeout=1)

def move(ser, m1,m2,m3): 
# Send command to move motors
    ser.write(b'Move\n')
    ser.write(str(m1).encode() + b'\n')
    ser.write(str(m2).encode() + b'\n')
    ser.write(str(m3).encode() + b'\n')

def returnhome(ser):
# Send command to return to home
    ser.write(b'ReturnHome\n')







# ser.write(b'Move\n')
# ser.write(b'0\n')
# ser.write(b'0\n')
# ser.write(b'0\n')
#end_time=time.time()
#total_time=end_time-start_time
#print("total time", total_time)
# Close connection to Arduino


