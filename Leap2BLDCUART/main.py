# this simple code connects a leap motion to Tinymovr controller
# Window code

import time, json,os
import Leap
import random
from numpy import *
import serial

# sampling rate (s)
dt = 0.01
# index yaw angle (along x-axis of the leap motion)
z = 0.0
# flag to notify if there is a hand
isFinger = False
# ID of the COM port
port = "COM5"

# instanciate Leap Motion and Serial Port
ser = serial.Serial(port, 115200, timeout=0.005)
leapController = Leap.Controller()

# calibrate the motor
time.sleep(1)
print(".Q")
ser.write(".Q\n")
time.sleep(10)

# activate the closed loop
print(".A")
ser.write(".A\n")

while True:
    # get the sensors
    # time
    t = time.clock()
    # leap
    isFinger = False
    frame = leapController.frame()
    for fin in frame.fingers:
        if fin.hand.is_left and fin.type == fin.TYPE_INDEX:
            z = int(2*4096*fin.direction.x/pi)
            isFinger = True

    # send the command
    if isFinger:
        ser.write(".P"+str(z)+"\n")
        print(z)
    else:
        ser.write(".I0\n")

    # wait approximately until the next top
    time.sleep(dt)