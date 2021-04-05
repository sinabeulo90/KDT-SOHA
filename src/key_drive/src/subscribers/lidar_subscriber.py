#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import numpy as np

from sensor_msgs.msg import LaserScan


class LidarSubscriber():
    def __init__(self, name="/scan"):
        # rospy subscriber
        self.sub_lidar = rospy.Subscriber(name, LaserScan, self._callback, queue_size=1)


    def get(self):
        return False, None
            

    def _callback(self, msg):
        pass