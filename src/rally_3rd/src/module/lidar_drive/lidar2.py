#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import numpy as np
import cv2 as cv
from module.utils.trackbar import Trackbar

def rint(point):
    return np.rint(point).astype(np.int32)

# 파라미터 폴더 경로
dir_path = os.path.dirname(os.path.join(__file__))
module_path = os.path.dirname(dir_path)
src_path = os.path.dirname(module_path)
# params_dir = os.path.join(src_path, "params", "image_processing")

# tb_degree = Trackbar(os.path.join(params_dir, "degree"), "degree", debug=True)
tb_degree = Trackbar(winname="degree", debug=True)

def show_lidar(lidar_info):
    background = np.zeros((500, 500, 3))
    delta = tb_degree.getValue("degree", "default", 0, 0, len(lidar_info)) * 360.0

    for i, dist in enumerate(lidar_info):
        if dist == 0:
            continue
        theta = 1.0 * (i + delta) % len(lidar_info) * 360.0
        dist *= 1000
        pt = tuple(rint([250 + dist * np.cos(theta), 250 + dist * np.sin(theta)]))
        cv.circle(background, pt, 3, (0, 0, 255))

    cv.imshow("background", background)
    cv.waitKey(1)