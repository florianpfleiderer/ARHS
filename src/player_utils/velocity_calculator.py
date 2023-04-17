#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from player.msg import *

from globals.globals import *

class VelocityCalculator:
    def __init__(self):
        self.target_subscriber = rospy.Subscriber("player/target_dist", PolarVector2, self.target_callback)
        self.field_components_subscriber = rospy.Subscriber("player/field_components", LaserScan, self.laser_scan_callback)

        self.target_distance = PolarVector2()
        self.field_components = FieldComponents()

    def laser_scan_callback(self, message):
        self.laser_scan = message

    def target_callback(self, message):
        self.target_distance = message

    def get_force(self, target_dist, multiplier=1, min_distance=-1, max_distance=-1):
        result = PolarVector2()

        if target_dist.r <= min_distance >= 0:
            return result
        
        if target_dist.r > max_distance >= 0:
            return result
        
        target_theta_rad = target_dist.theta * math.pi / 180

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
        return self.get_force(target_rel_pos, multiplier=REPELLING_FORCE_MULTIPLIER/(target_rel_pos.r + REPELLING_FORCE_SIZE), max_distance=REPELLING_FORCE_THRESHOLD)

    def calculate_input_velocity(self):
        velocity = Twist()

        vel_increment = self.get_attracting_force(self.target_distance)

        velocity.linear.x = vel_increment.r
        velocity.angular.z = vel_increment.theta
        rospy.loginfo(f"attracting force: {vel_increment}")


        rospy.loginfo(f"scan size {len(self.field_components.field_components)}")

        for (index, field_component) in enumerate(self.field_components.field_components):            
            vel_increment = self.get_repelling_force(field_component.player_distance)

            velocity.linear.x += vel_increment.r
            velocity.angular.z += vel_increment.theta

        rospy.loginfo(f"final input velocity is: r={round(velocity.linear.x, 2)} theta={round(velocity.angular.z, 2)}")

        return velocity
