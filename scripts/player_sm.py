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

    userdata:
        target_component: FieldComponent
        target_color: string

    '''
    def __init__(self):
        self.sm = smach.StateMachine(outcomes=["succeeded", "preempted", "aborted"])
        self.sm.userdata.target_component = FieldComponent()
        


        with self.sm:

            @smach.cb_interface(outcomes=["goal_reached", "puck_reached"])
            def move_to_dest_cb(userdata, status, result):
                rospy.loginfo("move to destination result: {}".format(result))
                if result.target_type_reached == "YellowGoal" or result.target_type_reached == "BlueGoal":
                    return "goal_reached"
                elif result.target_type_reached == "YellowPuck" or result.target_type_reached == "BluePuck":
                    return "puck_reached"
                else:
                    return "aborted"
            
            smach.StateMachine.add("GET_GAME_SETUP",
                        SimpleActionState("get_game_setup",
                                          GetGameSetupAction,
                                          result_slots=["target_type"]),
                        transitions={"succeeded": "FIND_DESTINATION"},
                        remapping={"target_type": "target_type"})
            
            smach.StateMachine.add("FIND_DESTINATION",
                        SimpleActionState("find_destination",
                                          FindDestinationAction,
                                          goal=FindDestinationGoal(),
                                          goal_slots=["target_type"],
                                          result_slots=["target_component"]),
                        transitions={"succeeded": "MOVE_TO_DESTINATION",
                                     # "preempted": "FIND_DESTINATION",
                                     "aborted": "FIND_DESTINATION"},
                        remapping={"target_component": "target_component"})

            smach.StateMachine.add("MOVE_TO_DESTINATION",
                        SimpleActionState("move_to_destination",
                                          MoveToDestinationAction,
                                          goal_slots=["target_component"],
                                          result_cb=move_to_dest_cb),
                        transitions={"goal_reached": "RELEASE_PUCK",
                                     "puck_reached": "FIND_DESTINATION",
                                     "aborted": "FIND_DESTINATION"},
                        remapping={"target_type_reached": "target_type"})
            
            smach.StateMachine.add("RELEASE_PUCK",
                        SimpleActionState("release_puck",
                                          ReleasePuckAction,
                                          goal_slots=["target_type"],
                                          result_slots=["target_type"]),
                        transitions={"succeeded": "FIND_DESTINATION",
                                     # "preempted": "RELEASE_PUCK",
                                     "aborted": "FIND_DESTINATION"},
                        remapping={"target_type": "target_type"})

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
