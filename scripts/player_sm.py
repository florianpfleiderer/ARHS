#!/usr/bin/env python

import rospy
from player.msg import *
import smach
from smach_ros import SimpleActionState, IntrospectionServer
import random
from geometry_msgs.msg import Vector3
from globals.tick import CallbackTicker
from globals.globals import *
from data_utils.topic_handlers import *


class LocomotionSM():
    def __init__(self):
        self.sm = smach.StateMachine(outcomes=["succeeded", "preempted", "aborted"])
        self.sm.userdata.target_component = FieldComponent()

        
        with self.sm:
            smach.StateMachine.add("FIND_DESTINATION",
                        SimpleActionState("find_destination",
                                          FindDestinationAction,
                                          goal=FindDestinationGoal(),
                                          result_slots=["target_component"]),
                        transitions={"succeeded": "MOVE_TO_DESTINATION",
                                     "preempted": "FIND_DESTINATION",
                                     "aborted": "FIND_DESTINATION"},
                        remapping={"target_component": "target_component"})
              
            smach.StateMachine.add("MOVE_TO_DESTINATION",
                        SimpleActionState("move_to_destination",
                                          MoveToDestinationAction,
                                          goal_slots=["target_component"]),
                        transitions={"succeeded": "FIND_DESTINATION",
                                     "preempted": "FIND_DESTINATION",
                                     "aborted": "MOVE_TO_DESTINATION"})
            
    def execute(self):
        return self.sm.execute()
    
class TestPublisher:
    def __init__(self):
        self.pub = FieldComponentsPublisher
        self.fcs = []
        for i in range(5):
            val = random.random() * 100
            vec = Vector3(val, val, val)
            self.fcs.append(FieldComponent("MAGENTA", "GenericObject", vec, vec))

    def run(self):
        self.pub.publish(self.fcs)
        print("published")
            

if __name__ == "__main__":
    rospy.init_node("player_sm")
    loc_sm = LocomotionSM()    
    sis = IntrospectionServer("player_state_machine", loc_sm.sm, "/SM_ROOT")
    sis.start()

    rospy.loginfo("started player state machine")

    # tp = TestPublisher()
    # ticker = CallbackTicker(TICK_RATE, tp.run)
    # ticker.start_thread()
    
    outcome = loc_sm.execute()

    rospy.spin()
    sis.stop()
