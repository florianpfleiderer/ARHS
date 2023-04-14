#!/usr/bin/env python
import sys

import rospy
import copy
from std_msgs.msg import String
from sensor_msgs.msg import Image

import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

class ColorCalibrator:

    def __init__(self):
        rospy.init_node("color_calibrator_node")
        rospy.loginfo("Initialised ColorCalibrator")

        self.bridge = CvBridge()
        self.image = None
        self.image_sub = rospy.Subscriber("robot1/kinect/rgb/image_raw", Image, self.camera_cb)
        
        
    def camera_cb(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

    def show_imgs(self, img1, img2):
        if self.image is not None:
            cv2.imshow('Color Mask', img1)
            cv2.imshow('Center Detector', img2)
            cv2.waitKey(10)
        else:
            rospy.loginfo("No image to show yet")
    
    def show_center_hsv(self, img):
        img_cpy = img.copy()
        img_hsv = cv2.cvtColor(img_cpy, cv2.COLOR_BGR2HSV)
        h, w, _ = img_hsv.shape
        center_x = w//2
        center_y = (h//4)*3
        pixel_center = img_hsv[center_y, center_x]
        cv2.circle(img_cpy, (center_x, center_y), 5, (255, 0, 0), 2)
        rospy.loginfo(pixel_center)
        return img_cpy
    
    def get_img_mask(self, img):
        h_u = cv2.getTrackbarPos('H_U', 'Color Mask')
        s_u = cv2.getTrackbarPos('S_U', 'Color Mask')
        v_u = cv2.getTrackbarPos('V_U', 'Color Mask')
        h_l = cv2.getTrackbarPos('H_L', 'Color Mask')
        s_l = cv2.getTrackbarPos('S_L', 'Color Mask')
        v_l = cv2.getTrackbarPos('V_L', 'Color Mask')
        
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img_hsv, np.array([[[h_l, s_l, v_l]]]), np.array([[[h_u, s_u, v_u]]]))
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask =  cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.bitwise_and(img, img, mask=mask)
        
    def init_trackbars(self):
        def nothing(x):
            pass
        
        cv2.namedWindow('Color Mask')
        cv2.createTrackbar('H_U', 'Color Mask', 179, 179, nothing)
        cv2.createTrackbar('S_U', 'Color Mask', 255, 255, nothing)
        cv2.createTrackbar('V_U', 'Color Mask', 255, 255, nothing)
        cv2.createTrackbar('H_L', 'Color Mask', 0, 179, nothing)
        cv2.createTrackbar('S_L', 'Color Mask', 0, 255, nothing)
        cv2.createTrackbar('V_L', 'Color Mask', 0, 255, nothing)
        
        

if __name__ == '__main__':
    color_calibrator = ColorCalibrator()
    color_calibrator.init_trackbars()
    loop_rate = rospy.Rate(10)
    
    rospy.loginfo("Starting loop")
    while not rospy.is_shutdown():
        if color_calibrator.image is not None:
                cur_img = copy.deepcopy(color_calibrator.image)
                img_center = color_calibrator.show_center_hsv(cur_img)
                img_mask = color_calibrator.get_img_mask(cur_img)
                color_calibrator.show_imgs(img_mask, img_center)
                
        loop_rate.sleep()
        