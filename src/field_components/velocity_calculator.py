#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from player.msg import PolarVector2

MAX_ANGULAR_SPEED = 5
MAX_LINEAR_SPEED = 0.5
TARGET_SIZE = 0.04
REPELLING_FORCE_THRESHOLD = 1
REPELLING_FORCE_MULTIPLIER = -1

SCAN_MIN_ANGLE = -100
SCAN_MAX_ANGLE = 100

class VelocityCalculator:
    def __init__(self):
        self.target_subscriber = rospy.Subscriber("player/target_dist", PolarVector2, self.target_callback)
        self.laser_scan_subscriber = rospy.Subscriber("robot1/front_laser/scan", LaserScan, self.laser_scan_callback)

        self.target_distance = PolarVector2()
        self.laser_scan = LaserScan()

    def laser_scan_callback(self, message):
        self.laser_scan = message

    def target_callback(self, message):
        self.target_distance = message

    def get_force(self, target_rel_pos, multiplier=1, min_distance=-1, max_distance=-1):
        result = PolarVector2()

        rospy.loginfo(f"force for {target_rel_pos}")

        if target_rel_pos.r <= min_distance >= 0:
            return result
        
        if target_rel_pos.r > max_distance >= 0:
            return result
        
        target_theta_rad = target_rel_pos.theta * math.pi / 180

        result.r = max(math.cos(target_theta_rad), 0)
        result.theta = target_theta_rad

        result.r *= multiplier
        result.theta *= multiplier

        if abs(result.r) > MAX_LINEAR_SPEED:
            result.r = MAX_LINEAR_SPEED * result.r / abs(result.r)

        if abs(result.theta) > MAX_ANGULAR_SPEED:
            result.theta = MAX_ANGULAR_SPEED * result.theta / abs(result.theta)

        return result

    def get_attracting_force(self, target_rel_pos):
        return self.get_force(target_rel_pos, multiplier=1, min_distance=TARGET_SIZE)    
        
    def get_repelling_force(self, target_rel_pos):
        return self.get_force(target_rel_pos, multiplier=REPELLING_FORCE_MULTIPLIER/(target_rel_pos.r + TARGET_SIZE), max_distance=REPELLING_FORCE_THRESHOLD)

    def calculate_input_velocity(self):
        velocity = Twist()

        vel_increment = self.get_attracting_force(self.target_distance)

        velocity.linear.x = vel_increment.r
        velocity.angular.z = vel_increment.theta
        rospy.loginfo(f"attracting force: {vel_increment}")


        # rospy.loginfo(f"scan size {len(self.laser_scan.ranges)}")
        # for (index, laser_dist) in enumerate(self.laser_scan.ranges):
        #     current_theta = self.laser_scan.angle_increment * index * 180 / math.pi + 180

        #     if not (SCAN_MIN_ANGLE < current_theta < SCAN_MAX_ANGLE):
        #         continue
            
        #     vel_increment = self.get_repelling_force(PolarLocation(laser_dist, current_theta))
        #     rospy.loginfo(f"repelling force: {vel_increment}")

        #     velocity.linear.x += vel_increment.r
        #     velocity.angular.z += vel_increment.theta

        rospy.loginfo(f"final input velocity is: r={round(velocity.linear.x, 2)} theta={round(velocity.angular.z, 2)}")

        return velocity
