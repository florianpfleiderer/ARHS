#!/usr/bin/env python
import rospy
import random
from player.msg import FindDestinationAction, FindDestinationGoal, FindDestinationResult, FieldComponent
from actionlib import SimpleActionServer
from geometry_msgs.msg import Vector3
from data_utils.topic_handlers import *

class FindDestinationServer:
    def __init__(self):
        self.server = SimpleActionServer("find_destination", FindDestinationAction, self.execute, False)
        self.server.start()
        self.field_component_sub = FieldComponentsSubscriber()

    def execute(self, goal: FindDestinationGoal):
        rospy.loginfo("executing state FIND_DESTINATION")
        result = FindDestinationResult()

        field_components = self.field_component_sub.data
        if field_components is None or len(field_components) == 0:
            rospy.logwarn("Empty field components for find destination!")
            self.server.set_aborted(result)
            time.sleep(1)
            return
        
        print(len(field_components))
        target = field_components[random.randint(0, len(field_components) - 1)]
        result.target_component = target
        rospy.loginfo(f"target acquired: {target}")

        self.server.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("find_destination_server")
    server = FindDestinationServer()
    rospy.spin()