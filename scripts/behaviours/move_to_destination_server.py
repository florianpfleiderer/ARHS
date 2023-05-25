#!/usr/bin/env python
'''MoveToDestination state

This State receives a FieldComponent as a goal and returns if goal is reached.

goals:
    FieldComponent - the component to move to

outcomes:
    target_reached - if the target is reached
    target_lost - if the target is lost

feedback:
    string - current state of the state machine
'''

import time
import rospy
from player.msg import MoveToDestinationAction, MoveToDestinationGoal,\
     MoveToDestinationResult
from geometry_msgs.msg import Twist
from math_utils.vector_utils import Vector3
from actionlib import SimpleActionServer
from globals.globals import TARGET_REACHED_R_THRESHOLD
from field_components.field_components import FieldObject
from typing import List
from data_utils.topic_handlers import FieldComponentsSubscriber, TargetComponentPublisher,\
    VelocityPublisher
from field_components.velocity_calculator import VelocityCalculator

class MoveToDestinationServer:
    '''this simple action server moves to a component given as the goal.

    '''
    def __init__(self):
        self.server = SimpleActionServer("move_to_destination", MoveToDestinationAction,\
                                         self.execute, False)
        self.server.start()

        self.vel_pub = VelocityPublisher()
        self.field_component_sub = FieldComponentsSubscriber()
        self.target_pub = TargetComponentPublisher()

        self.vel_calc = VelocityCalculator()

    def check_preempt(self):
        '''check if the action has been preempted'''
        if self.server.is_preempt_requested():
            rospy.loginfo("preempted")
            self.server.set_preempted()
            return True
        return False

    def execute(self, goal: MoveToDestinationGoal):
        '''execute the state MOVE_TO_DESTINATION'''
        rospy.loginfo("executing state MOVE_TO_DESTINATION")

        result = MoveToDestinationResult('target_reached', 'target_lost')

        target_component = goal.target_component

        if target_component is None or target_component.type == "":
            rospy.logwarn("Empty target for move to destination server!")
            self.server.set_aborted(result)
            return

        # target_object: FieldObject = FieldObject.from_field_component(target_component)

        # while target_object.distance.length() > TARGET_REACHED_R_THRESHOLD:
        #     field_components = self.field_component_sub.data
        #     self.target_pub.publish(target_component)

        #     if field_components is None or field_components == []:
        #         rospy.logwarn("Empty field components for move to destination server!")
        #         self.server.set_aborted(result)
        #         return

        #     field_objects: List[FieldObject] = ([FieldObject.from_field_component(fc) 
        #                                          for fc in field_components])
        #     out_velocity = self.vel_calc.get_input_velocity(field_objects, target_object)
        #     self.vel_pub.publish(out_velocity)

        #     rospy.loginfo(f"target: {target_object} distance: {target_object.distance.length()}")
        #     time.sleep(0.5)
        time.sleep(3)
        self.server.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("move_to_destination_server")
    server = MoveToDestinationServer()
    rospy.spin()