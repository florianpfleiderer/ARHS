#!/usr/bin/env python
'''Locomotion state machine
'''
import random
import rospy
import smach
from smach_ros import SimpleActionState, IntrospectionServer
from player.msg import FindDestinationAction, MoveToDestinationAction, FindDestinationGoal, GetGameSetupAction, ReleasePuckAction
from geometry_msgs.msg import Vector3
from data_utils.topic_handlers import FieldComponentsPublisher
from field_components.field_components import FieldComponent


class LocomotionSM():
    '''This state machine moves the robot to a destination and then finds a new destination.

    '''
    def __init__(self):
        self.sm = smach.StateMachine(outcomes=["succeeded", "preempted", "aborted"])
        self.sm.userdata.target_component = FieldComponent()
        


        with self.sm:
            smach.StateMachine.add("GET_GAME_SETUP",
                        SimpleActionState("get_game_setup",
                                          GetGameSetupAction,
                                          result_slots=["target_component"]),
                        transitions={"succeeded": "FIND_DESTINATION",
                                     "preempted": "GET_GAME_SETUP",
                                     "aborted": "GET_GAME_SETUP"},
                        remapping={"target_component": "target"})
            
            smach.StateMachine.add("FIND_DESTINATION",
                        SimpleActionState("find_destination",
                                          FindDestinationAction,
                                          goal=FindDestinationGoal(),
                                          goal_slots=["target"],
                                          result_slots=["target_component"]),
                        transitions={"succeeded": "MOVE_TO_DESTINATION",
                                     "preempted": "FIND_DESTINATION",
                                     "aborted": "FIND_DESTINATION"},
                        remapping={"target_component": "target_component"})

            smach.StateMachine.add("MOVE_TO_DESTINATION",
                        SimpleActionState("move_to_destination",
                                          MoveToDestinationAction,
                                          goal_slots=["target_component"],
                                          result_slots=['target_reached','target_lost']),
                        transitions={"succeeded": "RELEASE_PUCK",
                                     "preempted": "FIND_DESTINATION",
                                     "aborted": "MOVE_TO_DESTINATION"})
            
            smach.StateMachine.add("RELEASE_PUCK",
                        SimpleActionState("release_puck",
                                          ReleasePuckAction,
                                          result_slots=["target_component"]),
                        transitions={"succeeded": "FIND_DESTINATION",
                                     "preempted": "RELEASE_PUCK",
                                     "aborted": "FIND_DESTINATION"},
                        remapping={"target_component": "target_component"})

    def execute(self):
        '''execute the state machine'''
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
    
    # ref_com.send_name()
    # ref_com.wait_for_game()
    
    
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
