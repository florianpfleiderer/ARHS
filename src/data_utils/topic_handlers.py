#!/usr/bin/env python

import os
import rospy
import copy
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from globals.globals import *
from player.msg import FieldComponents, FieldComponent

class SubscriberWrapper:
    def __init__(self, topic, data_class):
        self.subscriber = rospy.Subscriber(topic, data_class, self.callback_func, queue_size=500)
        self.topic = topic
        self.data_class = data_class
        self.data = None

    def callback_func(self, msg):
        self.data = msg

    def copy_data(self):
        return copy.deepcopy(self.data)
 
    def is_valid(self):
        if self.data is None:
            rospy.logerr(f"{self.topic} has invalid data (class {self.data_class}!")
            return False
        return True

class ImageSubscriber(SubscriberWrapper):
    def __init__(self, imgtopic, imgtype):
        super().__init__(imgtopic, Image)
        self.imgtype = imgtype
        self.bridge = CvBridge()

    def callback_func(self, msg):
        try:
            self.data = self.bridge.imgmsg_to_cv2(msg, self.imgtype)
        except CvBridgeError as e:
            rospy.logerr(e)
            return

class RGBSubscriber(ImageSubscriber):
    def __init__(self):
        super().__init__(NAMESPACE + IMAGE_PATH, "bgr8")

class DepthSubscriber(ImageSubscriber):
    def __init__(self):
        super().__init__(NAMESPACE + DEPTH_PATH, "32FC1")

class LaserSubscriber(SubscriberWrapper):
    def __init__(self):
        super().__init__(NAMESPACE + LASER_PATH, LaserScan)

class FieldComponentsSubscriber(SubscriberWrapper):
    def __init__(self):
        super().__init__("/player/field_components", FieldComponents)

    def callback_func(self, msg: FieldComponents):
        self.data = msg.field_components

class TargetComponentSubscriber(SubscriberWrapper):
    def __init__(self):
        super().__init__("/player/target_component", FieldComponent)            

class FieldComponentsPublisher(rospy.Publisher):
    def __init__(self):
        super().__init__("/player/field_components", FieldComponents, queue_size=50)

class FieldDimensionsSubscriber(SubscriberWrapper):
    def __init__(self):
        super().__init__("/player/field_dimensions", FieldDimensions)

class FieldDimensionsPublisher(rospy.Publisher):
    def __init__(self):
        super().__init__("/player/field_dimensions", FieldDimensions, queue_size=10)

class TargetComponentPublisher(rospy.Publisher):
    def __init__(self):
        super().__init__("/player/target_component", FieldComponent, queue_size=10)

class VelocityPublisher(rospy.Publisher):
    def __init__(self):
        super().__init__(NAMESPACE + "cmd_vel", Twist, queue_size=10)



if __name__ == "__main__":
    img_sub = ImageSubscriber("robot1/kinect/rgb/image_raw", "bgr8")
    laser_sub = LaserSubscriber()
    laser_sub.topic = "robot1/front_laser/scan"
    
    def img_cb(msg):
        if msg is None:
            rospy.logerr("img is none!")
            return
        cv2.imshow("raw", CvBridge().imgmsg_to_cv2(msg, "bgr8"))

    raw_img_sub = rospy.Subscriber("robot1/kinect/rgb/image_raw", Image, img_cb, queue_size=500)
    
    while not rospy.is_shutdown():
        #if img_sub.is_valid():
        #    cv2.imshow("image", img_sub.copy_data())

        #if laser_sub.is_valid():
        #    cv2.imshow("laser", laser_scan_to_image(laser_sub.copy_data()))

        cv2.waitKey(10)
        time.sleep(0.5)
