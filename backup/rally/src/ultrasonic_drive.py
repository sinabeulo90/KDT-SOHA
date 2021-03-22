#!/usr/bin/env python
import discreatefilter
import numpy as np
import time

class ULTRASONIC:
    angle = 0
    speed = 0
    
    def __init__(self):
        # threshold
        self.thresh_side_away = 120
        self.thresh_side = 20
        self.thresh_fornt = 20

        self.thresh_bearing = 30

        self.flag = 0
        self.flag_back = False

        ## LPF
        freq = 20 * 1000
        f_cut = 500
        self.df_ultra = discreatefilter.filter(f_cut, freq, title='ultra')

    def set_motor(self, angle, speed):
        self.angle = angle
        self.motor = speed

    def set_speed_back(self, data):
        for i in [2, 1, 3]:    
            if data[i] < self.thresh_fornt or self.flag_back == True:
                print("checking")
                self.angle = 0
                self.flag_back = True
                self.flag += 1

                if self.flag < 5:
                    self.motor = 0
                    time.sleep(0.1)
                    return 0    
                else:
                    self.motor = -20
                    time.sleep(0.1)
                    break
                
        if self.flag > 10:
            self.flag = 0
            self.flag_back = False
            self.motor = - self.motor
        return 0

    def set_data(self, data):
        print(data)
        data = data[0]
        v_left = data[0] + data[1] * 1.407
        v_right = data[4] + data[3] * 1.407
        bearing = - v_left + v_right
        ## LPF
        bearing = int(self.df_ultra.LPF(np.array([bearing])))
        
        for i in [0, 4]:    
            if data[i] < self.thresh_side - 40:
                bearing = -(i-2)*self.thresh_bearing

        self.angle = 0
        if bearing < 0 :
            self.angle = -50
        elif bearing > 0:
            self.angle = 50

        self.set_speed_back(data)

    def get_motor(self):
        return self.angle, self.motor
