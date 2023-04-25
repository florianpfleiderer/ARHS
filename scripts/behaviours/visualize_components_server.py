#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from test_player.msg import VisualizeComponentsAction, VisualizeComponentsResult
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3

class VisualizeComponentsActionServer:
    def __init__(self):
        rospy.loginfo("Initialising VisualizeComponentsActionServer")
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=500)

        self.server = SimpleActionServer("visualize_components", VisualizeComponentsAction, self.execute, False)
        self.server.start()

        rospy.loginfo("VisualizeComponentsActionServer initialised")

    def make_marker(self, marker_type, scale, r, g, b, a=1.0):
        marker = Marker()
        marker.action = Marker.ADD
        marker.type = marker_type
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        return marker

    def execute(self, goal):
        rospy.loginfo("Executing VisualizeComponentsActionServer")
        result = VisualizeComponentsResult()

        marker = self.make_marker(Marker.CUBE, Vector3(1.0, 1.0, 1.0), 1, 0, 0)

        self.marker_pub.publish(marker)
        
        result.success = True
        
        self.server.set_succeeded(result)
        rospy.loginfo("VisualizeComponentsActionServer executed")

if __name__ == "__main__":
    rospy.init_node("visualize_components_server")
    server = VisualizeComponentsActionServer()
    rospy.spin()