#!/usr/bin/env python

import cv2, time
from pyzbar import pyzbar
from ros_module import *
#from dqn_drive import *
from parking import *
#from ultrasonic_drive import *
#from yolo_drive import *
from go_backward import *

rm = ROS("team_name")

angle = 0
speed = 15
mode = ""
cnt=0
total_count=0

obj = {
    #"DQNQR":DQN([256, 256], ""),
    #"PARKQR":PARK(),
    #"ULTRASONICQR":ULTRASONIC(),
    #"YOLOQR":YOLO(),
    "BACKWARDQR":BACKWARD()
}


while rm.get_shutdown():
    camera_image = rm.get_camera_image_data()
    
    if not camera_image.size == (640*480*3):
        continue

    qrcodes = pyzbar.decode(camera_image.copy())
    for qtcode in qrcodes:
	'''
	(x,y,w,h) = qrcode.rect
        cv2.rectangle(camera_image,(x,y), (x+w , y+h),(0,0,255),2)

        text=qrcode.data
        cv2.putText(camera_image,text,(x, y-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,0,255),2)
        print(text)

    cv2.imshow('frame', camera_image) 
    cv2.waitKey(10)
    '''
        qr_string = qtcode.data

	print(qr_string)


        if qr_string=="algorithm drive_avoid_obstacle":
            mode="ULTRASONICQR"

        elif qr_string=="2 algorithm drive_turn_back":
            mode="BACKWARDQR"
        '''
        elif qr_string=="dqn dqn_drive_start":
            mode="DQNQR"
        elif qr_string=="dqn dqn_drive_end":
            mode="YOLOQR"
        elif qr_string=="algorithm drive_to_parking_lot_1":
            mode="PARKQR"
        '''

    if mode == "":
            rm.set_motor(0, 0)
            continue

    obj[mode].set_motor(angle, speed)
    ''' 
    if mode == "DQNQR":
        obj[mode].set_data([])
    elif mode == "PARKQR":
        obj[mode].set_data([rm.get_ar_tags_datas])
    elif mode == "YOLOQR":
        obj[mode].set_data([])
    '''
    if mode == "ULTRASONICQR":
        obj[mode].set_data([rm.get_ultrasonic_data()])
    
    elif mode == "BACKWARDQR":
        #obj[mode].set_data(10)
	if total_count<=6:
	    print("cnt :"+str(cnt))
	    if cnt<=1:
            	
	    	obj[mode].set_data(False)
		cnt+=1
	    elif cnt>=2:
	    	#flag=True
	    	obj[mode].set_data(True)
	    	cnt=0
            
	else:
	    break
	    
        total_count+=1
	
	#else:
	    #mode="DQNQR"

    else:
        rm.set_motor(0, 0)
        continue

    angle, speed = obj[mode].get_motor()
    print(angle, speed) 
    rm.set_motor(angle, speed)
    
    
    
