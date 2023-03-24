#!/usr/bin/env python
import sys

import rospy
import copy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge, CvBridgeError

# You don't have to use a class, but it keeps things together and means that you
# don't use global variables
class PlayNode:

    def __init__(self, window_name="Kinect image"):
        """
        :param message_slop: Messages with a header.stamp within message_slop
        seconds of each other will be considered to be synchronisable
        """
        rospy.init_node("robot_node")
        rospy.loginfo("Initialised PlayNode")

        self.bridge = CvBridge()
        self.image_window = window_name
        self.image = None
        self.laser = None

        self.velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
        self.image_sub = rospy.Subscriber("kinect/rgb/image_raw", Image, self.camera_cb)
        self.laser_sub = rospy.Subscriber("front_laser/scan", LaserScan, self.laser_cb)

    def laser_cb(self, msg):
        """This function is called every time a message is received on the
        front_laser/scan topic

        We just keep the self.laser member up to date
        """
        rospy.loginfo("Received new scan")
        self.laser = msg

    def camera_cb(self, msg):
        """This function is called every time a message is received on the
        front_camera/image_raw topic

        We convert the image to an opencv object and update the self.image member
        """
        rospy.loginfo("Received new image")

        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

    def show_img(self):
        """Need to do this separately because otherwise it can freeze
        """
        if self.image is not None:
            cv2.imshow(self.image_window, self.image)
            cv2.waitKey(10)
        else:
            rospy.loginfo("No image to show yet")

    def set_velocities(self, linear, angular):
        """Use this to set linear and angular velocities
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.velocity_pub.publish(msg)

if __name__ == '__main__':
    play_node = PlayNode()

    # You can keep the following in main, or put it into a PlayNode.run() function.

    # 10 Hz loop
    loop_rate = rospy.Rate(10)

    rospy.loginfo("Starting loop")
    while not rospy.is_shutdown():
        # We check that we have both laser and image data. Because these are
        # constantly being updated, we should always have the most recent
        # messages received
        if play_node.laser and play_node.image is not None:
            # Now, we can use the laser and image data to do something. Because
            # the objects in the PlayNode are constantly updated, we need to
            # make a deep copy so that the data doesn't change while we're doing
            # computations on it. This isn't very efficient, but we do it just
            # for demonstration purposes
            cur_laser = copy.deepcopy(play_node.laser)
            cur_image = copy.deepcopy(play_node.image)

            # Do something useful with laser and image here

            play_node.show_img()

        loop_rate.sleep()
