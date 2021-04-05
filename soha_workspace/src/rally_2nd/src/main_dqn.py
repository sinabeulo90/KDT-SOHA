#!/usr/bin/env python

import cv2, time
# from pyzbar import pyzbar
from ros_module import *
from dqn_drive import *
# from parking import *
#from ultrasonic_drive import *
# from yolo_drive import *
# from go_backward import *

rm = ROS("team_name")

angle = 0
speed = 20
mode = ""

obj = {
    "DQNQR":DQN(),
    #"PARKQR":PARK(),
    #"ULTRASONICQR":ULTRASONIC(),
    #"YOLOQR":YOLO(),
    # "BACKWARDQR":BACKWARD()
}

mode = "DQNQR"

while rm.get_shutdown():
    
    if mode == "DQNQR":
        obj[mode].set_data([
            rm.ultrasonic_data["L"],
            rm.ultrasonic_data["FL"],
            rm.ultrasonic_data["FM"],
            rm.ultrasonic_data["FR"],
            rm.ultrasonic_data["R"],
        ])


    angle, speed = obj[mode].get_motor() 
    rm.set_motor(angle, speed)
    
    
    
