#!/usr/bin/env python

import rospy
import random
from player.msg import *
import actionlib

class FindDestinationServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("find_destination", FindDestinationAction, self.execute, False)
        self.server.start()

        self.target_position_pub = rospy.Publisher("player/target_dist", PolarVector2, queue_size=500)
        
    def execute(self, goal):
        rospy.loginfo("executing state FIND_DESTINATION")

        if len(goal.field_components) == 0:
            self.server.set_aborted()
            return
        
        target_index = random.randint(0, len(goal.field_components) - 1)
        self.server.set_succeeded()
        return target_index

if __name__ == "__main__":
    rospy.init_node("find_destination")
    server = FindDestinationServer()
    rospy.spin()