#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from player.msg import *
import smach
from smach_ros import SimpleActionState, IntrospectionServer
import random
from geometry_msgs.msg import Vector3


class LocomotionSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "preempted", "aborted"])

        with self:
            self.add("FIND_DESTINATION",
                        SimpleActionState("find_destination",
                                          FindDestinationAction),
                        transitions={"succeeded": "MOVE_TO_DESTINATION",
                                     "preempted": "FIND_DESTINATION",
                                     "aborted": "FIND_DESTINATION"})
              
            self.add("MOVE_TO_DESTINATION",
                        SimpleActionState("move_to_destination",
                                          MoveToDestinationAction),
                        transitions={"succeeded": "FIND_DESTINATION",
                                     "preempted": "FIND_DESTINATION",
                                     "aborted": "MOVE_TO_DESTINATION"})
    
class TestPublisher:
    def __init__(self):
        self.pub = rospy.Publisher("player/field_components", FieldComponents, queue_size=500)
        self.rate = rospy.Rate(5)

    def run(self):
        fcs = FieldComponents()
        for i in range(5):
            val = random.random() * 100
            vec = Vector3(val, val, val)
            fcs.field_components.append(FieldComponent("MAGENTA", "unknown", vec, vec))
            
        self.pub.publish(fcs)
        print("published")
            

if __name__ == "__main__":
    rospy.init_node("player_sm")
    sm = LocomotionSM()    
    sis = IntrospectionServer("player_state_machine", sm, "/SM_ROOT")
    sis.start()

    rospy.loginfo("started player state machine")
    outcome = sm.execute()

    sis.stop()
