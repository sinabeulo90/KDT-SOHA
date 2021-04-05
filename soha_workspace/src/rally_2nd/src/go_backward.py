#!/usr/bin/env python

import time

class BACKWARD:
    angle = 0
    speed = 0

    def __init__(self):
        pass

    def set_motor(self, angle, speed):
        self.angle = angle
        self.speed = speed
	

    def set_data(self, data):
	print(data)
	#for i in range(5):
     	
        for j in range(20):
            self.set_motor(0,0)
            time.sleep(0.1)
	    
        if data==False:
	    #for j in range(20):
             #   self.set_motor(0,0)
              #  time.sleep(0.1)

            for j in range(3):
 	    	print(j)
		
	    	self.set_motor(30, -15)
	    	time.sleep(0.1)
	    return 0
	    
	if data:
            for j in range(3):
		print(j)
            	self.set_motor(-30,15)      
            	time.sleep(0.1)
	    
	    return 0
	
    def get_motor(self):
        return self.angle, self.speed

