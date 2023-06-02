#!/usr/bin/env python
'''FindDestination state

This State receives a FieldComponent as a goal and returns it as a result.

goals:
    FieldComponent - the component to find
    
outcomes:
    FieldComponent - the found component
    string - error message if no component is found
    
feedback:
    string - current state of the state machine
    
'''

import random
import time
import rospy
from actionlib import SimpleActionServer
from field_components.field_components import Field
from player.msg import FindDestinationAction, FindDestinationGoal, FindDestinationResult

class FindDestinationServer:
    '''this simple action server subscribes to field_component msg and returns
    the searched component as a result
    
    For this to work, the object detector must publish field components at all times.
    The FieldComponentsSubscriber class is used to subscribe to the topic.
    '''
    def __init__(self):
        self.server = SimpleActionServer("find_destination", FindDestinationAction,\
                                         self.execute, False)
        self.server.start()
        self.field = Field()
        self.field_component_list = [Field.get_objects_by_class(type) for type in ['YellowPuck', 'BluePuck', 'YellowGoal', 'BlueGoal']]

    def check_preempt(self):
        '''check if the action has been preempted'''
        if self.server.is_preempt_requested():
            rospy.loginfo("preempted")
            self.server.set_preempted()
            return True
        return False

    def execute(self, goal: FindDestinationGoal):
        '''execute the state FIND_DESTINATION'''
        rospy.loginfo("executing state FIND_DESTINATION")
        result = FindDestinationResult()

        field_components = self.field_component_list.data
        if field_components is None or len(field_components) == 0:
            rospy.logwarn("Empty field components for find destination!")
            self.server.set_aborted(result)
            time.sleep(1)
            return

        print(len(field_components))
        target = field_components[random.randint(0, len(field_components) - 1)]
        result.target_component = target
        time.sleep(3)
        rospy.loginfo(f"target acquired: {target}")
        self.server.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("find_destination_server")
    server = FindDestinationServer()
    rospy.spin()