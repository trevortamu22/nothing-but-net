import serial
import time

print("start")
# Connect to Arduino

def movemotor (m1,m2,m3): 
    ser = serial.Serial('COM3', 9600, timeout=1)
    time.sleep(0.1) # wait for connection to establish
# Send command to move motors
    ser.write(b'Move\n')
    ser.write(str(m1).encode() + b'\n')
    ser.write(str(m2).encode() + b'\n')
    ser.write(str(m3).encode() + b'\n')
    ser.close()

def returnhome():
# Send command to return to home
    ser = serial.Serial('COM3', 9600, timeout=1)
    time.sleep(0.1) # wait for connection to establish
    ser.write(b'ReturnHome\n')
    ser.close()








# ser.write(b'Move\n')
# ser.write(b'0\n')
# ser.write(b'0\n')
# ser.write(b'0\n')
#end_time=time.time()
#total_time=end_time-start_time
#print("total time", total_time)
# Close connection to Arduino


