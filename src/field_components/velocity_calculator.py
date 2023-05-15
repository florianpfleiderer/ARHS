#!/usr/bin/env python
from math_utils.vector_utils import *
from field_components.field_components import *
from typing import List
from geometry_msgs.msg import Twist

class VelocityCalculator:
    def get_input_velocity(self, field_objects: List[FieldObject], target_object: FieldObject):
        attracting = TupleVector3()
        repelling = TupleVector3()

        attracting += target_object.distance

        for obj in field_objects:
            if obj is not target_object and obj.distance.length() < REPELLING_FORCE_THRESHOLD:
                repelling += 1 / (obj.distance + 0.2)

        if attracting.length() < TARGET_REACHED_R_THRESHOLD + TARGET_SIZE:
            return Twist()

        rospy.loginfo(f"attracting force: {attracting.length(): .2f}, repelling force: {repelling.length(): .2f}")

        result_force = (attracting * ATTRACTION_FACTOR).convert(Coordinate.CYLINDRICAL)# - repelling * REPULSION_FACTOR).convert(Coordinate.CYLINDRICAL)
        
        out_linear = min(result_force[0] * cosd(result_force[1]), MAX_LINEAR_SPEED)
        out_angular = min(result_force[1], MAX_ANGULAR_SPEED)
        
        if result_force[0] - MAX_LINEAR_SPEED > 0.5:
            rospy.logwarn(f"Max linear speed exceeded by: {result_force[0] - MAX_LINEAR_SPEED: .2f}!")

        if result_force[1] - MAX_ANGULAR_SPEED > 0.5:
            rospy.logwarn(f"Max angular speed exceeded by: {result_force[1] - MAX_ANGULAR_SPEED: .2f}!")

        out_velocity = Twist()
        out_velocity.linear.x = out_linear
        out_velocity.angular.z = out_angular * math.pi/180

        rospy.loginfo(f"out speed: linear {out_linear: .2f} m/s angular {out_angular: .2f} rad/s")
        return out_velocity