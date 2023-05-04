''' Allows communication with the arduino/mechanical system through the use of simply python functions
    -Trevor Sides 05/03/23'''

import serial
import time

# MAKE SURE THIS AGREES WITH THE ADRUINO IDE
COM = 'COM3'

# Connect to the arduino over the given COM port
def connect():
    try:
        ser = serial.Serial(COM, 9600, timeout=1)
        print('Arduino Connected on', COM)
    except:
        print('Connection to the Arduino Failed!\nDid you check the COM port in the Arduino IDE?')
        return None
    return ser


# Send a move command to all 3 motors
def move(ser, m1,m2,m3): 
    # Protect Motor Limits
    print('Moving to', m1, m2, m3)
    if m1 >= 500: m1 = 500
    if m2 >= 500: m2 = 500
    if m3 >= 500: m3 = 500
    if ser is not None:
        # Send command to move motors
        ser.write(b'Move\n')
        ser.write(str(m1).encode() + b'\n')
        ser.write(str(m2).encode() + b'\n')
        ser.write(str(m3).encode() + b'\n')


# Request that the motors and backboard return to the default home postion
def returnhome(ser):
    # Send command to return to home
    print('Returning Home!')
    if ser is not None:
        ser.write(b'ReturnHome\n')
        time.sleep(1)
