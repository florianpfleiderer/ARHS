#!/usr/bin/env python

import rospy
from player.msg import MoveToDestinationAction, MoveToDestinationGoal, MoveToDestinationResult
from geometry_msgs.msg import Twist
from math_utils.vector_utils import *
from actionlib import SimpleActionServer
from globals.globals import *
from field_components.field_components import *
from typing import List
from data_utils.topic_handlers import *
from field_components.velocity_calculator import *

class MoveToDestinationServer:
    def __init__(self):
        self.server = SimpleActionServer("move_to_destination", MoveToDestinationAction, self.execute, False)
        self.server.start()
        
        self.vel_pub = VelocityPublisher()
        self.field_component_sub = FieldComponentsSubscriber()
        self.target_pub = TargetComponentPublisher()

        self.vel_calc = VelocityCalculator()

    def execute(self, goal: MoveToDestinationGoal):
        rospy.loginfo("executing state MOVE_TO_DESTINATION")

        result = MoveToDestinationResult(False)

        target_component = goal.target_component

        if target_component is None or target_component.type == "":
            rospy.logwarn("Empty target for move to destination server!")
            self.server.set_aborted(result)
            return
        
        target_object: FieldObject = FieldObject.from_field_component(target_component)

        while target_object.distance.length() > TARGET_REACHED_R_THRESHOLD:
            field_components = self.field_component_sub.data
            self.target_pub.publish(target_component)

            if field_components is None or field_components == []:
                rospy.logwarn("Empty field components for move to destination server!")
                self.server.set_aborted(result)
                return

            field_objects: List[FieldObject] = [FieldObject.from_field_component(fc) for fc in field_components]
            out_velocity = self.vel_calc.get_input_velocity(field_objects, target_object)
            self.vel_pub.publish(out_velocity)

            rospy.loginfo(f"target: {target_object} distance: {target_object.distance.length()}")
            time.sleep(0.5)

        self.server.set_aborted(result)
    
if __name__ == "__main__":
    rospy.init_node("move_to_destination_server")
    server = MoveToDestinationServer()
    rospy.spin()