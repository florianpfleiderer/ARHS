#!/usr/bin/env python

import rospy
import copy
from cv_bridge import CvBridge, CvBridgeError
from data_utils.data_validation import *
from visualization.screen_components import *
from sensor_msgs.msg import Image, LaserScan
import time

class ImageSubscriber:
    def __init__(self, name, imgtopic, imgtype):
        self._imgtopic = imgtopic
        self._imgtype = imgtype
        self.name = name

        self._bridge = CvBridge()
        self._imgsub = rospy.Subscriber(self._imgtopic, Image, self.img_cb)
        self._image = None

        self.v = Validator()


    def img_cb(self, msg):
        try:
            self._image = self._bridge.imgmsg_to_cv2(msg, self._imgtype)
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
    def get_image(self) -> Image:
        if self.is_valid():
            return copy.deepcopy(self._image)

    def is_valid(self) -> bool:
        return self.v.guard_none(self._image)

    # def show_image(self):
    #     self.viewer.show(self._image)

    # def draw_objects(self, objects):
    #     self.viewer.draw_objects(objects)

class LaserSubscriber:
    def __init__(self, name, topic):
        self._topic = topic
        self.name = name

        self._laser_sub = rospy.Subscriber(topic, LaserScan, self.laser_cb)
        self._laser_scan = None

        self.v = Validator()

    def laser_cb(self, msg):
        self._laser_scan = msg

    def is_valid(self):
        return self.v.guard_none(self._laser_scan)
    
    def get_scan(self):
        return copy.deepcopy(self._laser_scan)


if __name__ == "__main__":
    rospy.init_node("test")
    ks = ImageSubscriber("kinect image", LOCAL_PLAYER + IMAGE_PATH, "bgr8")
    rate = rospy.Rate(12)
    print(LOCAL_PLAYER + IMAGE_PATH)

    while not rospy.is_shutdown():
        img = ks.get_image()
        if img is not None:
            cv2.imshow(ks.name, img)
            cv2.waitKey(10)
        
        rate.sleep()
