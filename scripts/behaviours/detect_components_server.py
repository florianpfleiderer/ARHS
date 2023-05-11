#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from test_player.msg import DetectComponentsAction, DetectComponentsResult

def DetectComponents(inputs):
    components = []

    components.append(FieldComponent(1, PolarVector2(1, 10), 0.2))

    return components

class DetectComponentsActionServer:
    def __init__(self):
        rospy.loginfo("Initialising DetectComponentsActionServer")
        self.server = SimpleActionServer("detect_components", DetectComponentsAction, self.execute, False)
        self.server.start()
        rospy.loginfo("DetectComponentsActionServer initialised")

    def execute(self, goal):
        rospy.loginfo("Executing DetectComponentsActionServer")
        result = DetectComponentsResult()
        
        result.field_components = DetectComponents(goal)
        
        self.server.set_succeeded(result)
        rospy.loginfo("DetectComponentsActionServer executed")

if __name__ == "__main__":
    rospy.init_node("detect_components_server")
    server = DetectComponentsActionServer()
    rospy.spin()