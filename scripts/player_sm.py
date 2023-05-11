#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from player.msg import *
import smach
from smach_ros import SimpleActionState, IntrospectionServer


class LocomotionSM:
    def __init__(self):
        self.sm = smach.StateMachine(outcomes=["succeeded", "preempted", "aborted"])

        with self.sm:
            self.sm.add("FIND_DESTINATION",
                        SimpleActionState("find_destination",
                                          FindDestinationAction,
                                          goal_cb=self.find_destination_goal_cb),
                        transitions={"succeeded": "MOVE_TO_DESTINATION",
                                     "preempted": "FIND_DESTINATION",
                                     "aborted": "FIND_DESTINATION"},
                        remapping={"field_components": "field_components"})
              
            self.sm.add("MOVE_TO_DESTINATION",
                        SimpleActionState("move_to_destination",
                                          MoveToDestinationAction,
                                          goal_cb=self.move_to_destination_goal_cb),
                        transitions={"succeeded": "FIND_DESTINATION",
                                     "preempted": "FIND_DESTINATION",
                                     "aborted": "FIND_DESTINATION"},
                        remapping={"field_components": "field_components",
                                   "destination_index": "destination_index"})

        self.field_components_sub = rospy.Subscriber("player/field_components", FieldComponents, self.field_components_cb)
        self.field_components = []

    def field_components_cb(self, msg):
        self.field_components = msg.field_components

    def find_destination_goal_cb(self, userdata, goal):
        goal.field_components = self.field_components
        return goal
    
    def move_to_destination_goal_cb(self, userdata, goal):
        goal.field_components = self.field_components
        goal.destination_index = userdata.destination_index
        return goal
    
    def execute(self):
        return self.sm.execute()


if __name__ == "__main__":
    rospy.init_node("player_sm")
    sm = LocomotionSM()    
    sis = IntrospectionServer("player_state_machine", sm.sm, "/SM_ROOT")
    sis.start()

    rospy.loginfo("started player state machine")
    outcome = sm.execute()

    rospy.spin()
    sis.stop()
