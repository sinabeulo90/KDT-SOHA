#!/usr/bin/env python
#-*- coding: utf-8 -*-

import yaml, os
import numpy as np
import cv2 as cv
import rospy

from image_processing import processing
from sliding_window import sliding_window
from scipy.optimize import least_squares

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from xycar_motor.msg import xycar_motor

from collections import deque
from datetime import datetime
from utils.calculate import get_linear_function
from utils.filter import Filter


# filter_left = Filter(f_cut=3000, freq=10000, num_data=3)
# filter_right = Filter(f_cut=3000, freq=10000, num_data=3)
filter_deg = LowPassFilter(0.75)


def rint(point):
    return np.rint(point).astype(np.int32)

script_dir = os.path.dirname(__file__)
yaml_file_path = os.path.join(script_dir, "params", "calibration", "usb_cam.yaml")

with open(yaml_file_path) as f:
    data = yaml.load(f)

width, height = data["image_width"], data["image_height"]

cameraMatrix = data["camera_matrix"]["data"]
rows, cols = data["camera_matrix"]["rows"], data["camera_matrix"]["cols"]
cameraMatrix = np.array(cameraMatrix).reshape((rows, cols))

distCoeffs = data["distortion_coefficients"]["data"]
rows, cols = data["distortion_coefficients"]["rows"], data["distortion_coefficients"]["cols"]
distCoeffs = np.array(distCoeffs).reshape((rows, cols))

imageSize = (width, height)
newImgSize = (width, height)

newCameraMatrix, calibrated_ROI = cv.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, newImgSize)

image_dir_path = os.path.join(script_dir, "image")

if not os.path.isdir(image_dir_path):
    os.mkdir(image_dir_path)


def calibrate_image(frame):
    tf_image = cv.undistort(frame, cameraMatrix, distCoeffs, None, newCameraMatrix)
    roi_x, roi_y, roi_width, roi_height = calibrated_ROI
    tf_image = tf_image[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width].copy()
    return cv.resize(tf_image, (frame.shape[1], frame.shape[0]))


bridge = CvBridge()

is_update = False
image_data = None


# dq = deque(maxlen=100)
dq2 = deque(maxlen=100)

def callback_camera(data):
    global image_data, is_update
    image_data = bridge.imgmsg_to_cv2(data, "bgr8")
    is_update = True
    # dq.append(datetime.now())
    # if len(dq) == dq.maxlen:
    #     print(dq[-1] - dq[0]) # 3초에 100장


motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
rospy.Subscriber("usb_cam/image_raw", Image, callback_camera)

rospy.init_node("drive")
rate = rospy.Rate(10)
steering_deg = 0

filter_coef_left = None
filter_coef_right = None

try:
    while not rospy.is_shutdown():
        if not is_update:
            continue

        frame = image_data

        def routine(frame, POS=None):
            global steering_deg, filter_coef_left, filter_coef_right
            frame = calibrate_image(frame)

            processed_frame, explain1 = processing(frame)
                
            processed_height, processed_width = processed_frame.shape[:2]
            processed_height2 = processed_height//3*2

            choosen_left, choosen_right, explain2 = sliding_window(processed_frame)

            func_coef_left = get_linear_function(choosen_left)
            func_coef_right = get_linear_function(choosen_right)

            if func_coef_left and func_coef_right:
                for coef in [func_coef_left, func_coef_right, (func_coef_left + func_coef_right)/2]:
                    func = np.poly1d(coef)
                new_ys = np.linspace(0, processed_frame.shape[0], num=processed_frame.shape[0], endpoint=True)
                new_xs = func(new_ys)

                for x, y in zip(new_xs, new_ys):
                    if 0 < y < processed_frame.shape[0] and 0 < x < processed_frame.shape[1]:
                        x, y = int(x), int(y)
                        cv.circle(explain2, (x, y), 3, (0, 255, 255), -1)

            heading_src = (processed_width//2, processed_height)
            heading_dst = (func(processed_height2), processed_height2)
            heading_sub =  (heading_src[0] - heading_dst[0], heading_src[1] - heading_dst[1])

            heading_rad = np.arctan2(heading_sub[1], heading_sub[0])
            heading_deg = np.degrees(heading_rad)

            # heading_deg = filter_deg.LPF(np.array([heading_deg]))[0]
            heading_deg = filter_deg2.get_lpf(heading_deg)
            steering_deg = heading_deg - 90

            heading_rad = np.radians(heading_deg)

            gradient = np.tan(heading_rad)

            heading_dst = ((processed_height2 - processed_height)/gradient + processed_width//2, processed_height2)


            heading_src = tuple(rint(heading_src))
            heading_dst = tuple(rint(heading_dst))

            cv.circle(explain2, heading_src, 3, (0, 0, 255), -1)
            cv.circle(explain2, heading_dst, 3, (0, 0, 255), -1)
            cv.line(explain2, heading_src, heading_dst, (0, 0, 255), 1)
            
            else:
                # heading_rad = np.radians(heading_deg)
                # gradient = np.tan(heading_rad) + 1e-10
                
                # heading_src = (processed_width//2, processed_height)
                # heading_dst = ((processed_height2 - heading_src[1])/gradient + heading_src[0], processed_height2)

                # heading_src = tuple(rint(heading_src))
                # heading_dst = tuple(rint(heading_dst))

                # cv.circle(explain2, heading_src, 3, (0, 0, 255), -1)
                # cv.circle(explain2, heading_dst, 3, (0, 0, 255), -1)
                # cv.line(explain2, heading_src, heading_dst, (0, 0, 255), 1)
            
                if func_coef_left:
                    func = np.poly1d(func_coef_left)
                    new_ys = np.linspace(0, processed_frame.shape[0], num=processed_frame.shape[0], endpoint=True)
                    new_xs = func(new_ys)

                    left_src = (func(processed_height), processed_height)
                    left_dst = (func(processed_height2), processed_height2)
                    left_sub =  (left_src[0] - left_dst[0], left_src[1] - left_dst[1])

                    left_rad = np.arctan2(left_sub[1], left_sub[0])
                    heading_deg = np.degrees(left_rad)

                    heading_deg = filter_deg2.get_lpf(heading_deg)
                    steering_deg = heading_deg - 90

                    heading_rad = np.radians(heading_deg)

                    gradient = np.tan(heading_rad) + 1e-10
                    heading_src = (processed_width//2, processed_height)
                    heading_dst = ((processed_height2 - processed_height)/gradient + processed_width//2, processed_height2)

                    heading_src = tuple(rint(heading_src))
                    heading_dst = tuple(rint(heading_dst))

                    cv.circle(explain2, heading_src, 3, (0, 0, 255), -1)
                    cv.circle(explain2, heading_dst, 3, (0, 0, 255), -1)
                    cv.line(explain2, heading_src, heading_dst, (0, 0, 255), 1)
                
                elif func_coef_right:
                    func = np.poly1d(func_coef_right)
                    new_ys = np.linspace(0, processed_frame.shape[0], num=processed_frame.shape[0], endpoint=True)
                    new_xs = func(new_ys)

                    right_src = (func(processed_height), processed_height)
                    right_dst = (func(processed_height2), processed_height2)
                    right_sub =  (right_src[0] - right_dst[0], right_src[1] - right_dst[1])

                    right_rad = np.arctan2(right_sub[1], right_sub[0])
                    heading_deg = np.degrees(right_rad)

                    heading_deg = filter_deg2.get_lpf(heading_deg)
                    steering_deg = heading_deg - 90

                    heading_rad = np.radians(heading_deg)

                    gradient = np.tan(heading_rad) + 1e-10

                    heading_src = (processed_width//2, processed_height)
                    heading_dst = ((processed_height2 - processed_height)/gradient + processed_width//2, processed_height2)

                    heading_src = tuple(rint(heading_src))
                    heading_dst = tuple(rint(heading_dst))

                    cv.circle(explain2, heading_src, 3, (0, 0, 255), -1)
                    cv.circle(explain2, heading_dst, 3, (0, 0, 255), -1)
                    cv.line(explain2, heading_src, heading_dst, (0, 0, 255), 1)
                else:
                    new_ys = []
                    new_xs = []


            for x, y in zip(new_xs, new_ys):
                if 0 < y < processed_frame.shape[0] and 0 < x < processed_frame.shape[1]:
                    x, y = int(x), int(y)
                    cv.circle(explain2, (x, y), 3, (255, 255, 0), -1)

            # Merge Explain
            vertical_line = np.zeros((explain1.shape[0], 5, 3), dtype=np.uint8)
            explain_merge = np.hstack((explain2, vertical_line, explain1))

            return explain_merge, steering_deg

        explain_frame, steering_deg = routine(frame)
        is_update = False

        message = xycar_motor()
        message.angle = steering_deg * 1.7
        message.speed = 25

        motor_pub.publish(message)
        
        dq2.append(datetime.now())
        if len(dq2) == dq2.maxlen:
            print("processed", dq2[-1] - dq2[0])

        cv.imshow("explain", explain_frame)
        cv.waitKey(1)

        # Rendering
        # cv.imshow("explain", explain_frame)
        # key = cv.waitKeyEx(1)
        # if key == ord('q'):
        #     break
        rate.sleep()

finally:
    # cv.destroyAllWindows()
    pass
