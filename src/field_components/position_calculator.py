#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from player.msg import PolarVector2
import time
import math
from vector_utils import *

class PositionCalculator:
    def __init__(self):
        self.velocity_subscriber = rospy.Subscriber("robot1/cmd_vel", Twist, self.velocity_scan_callback)
        self.target_subscriber = rospy.Subscriber("player/target_dist", PolarVector2, self.target_dist_callback)
        self.velocity = Twist()
        self.target_dist = PolarVector2()

        self.calc_time = time.time()

    def velocity_scan_callback(self, message):
        self.velocity = message

    def target_dist_callback(self, message):
        self.target_dist = message

    def calculate_target_position(self):
        now = time.time()
        delta_t = now - self.calc_time
        self.calc_time = now

        delta_r = self.velocity.linear.x * delta_t
        delta_theta = self.velocity.angular.z * delta_t * 180 / math.pi

        cartesian_delta = polar_to_cartesian(PolarVector2(delta_r, delta_theta))
        cartesian_target = polar_to_cartesian(self.target_dist)

        target_position = cartesian_to_polar(cartesian_distance(cartesian_delta, cartesian_target))

        return target_position
