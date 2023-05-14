#!/usr/bin/env python

import rospy
import random
from player.msg import *
import actionlib

class FindDestinationServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("find_destination", FindDestinationAction, self.execute, False)
        self.server.start()
        self.field_components_sub = rospy.Subscriber("player/field_components", FieldComponents, self.field_components_cb)
        self.field_components = []
        self.destination_index_pub = rospy.Publisher("player/destination_index", Destination, queue_size=500)
        
    def field_components_cb(self, msg: FieldComponents):
        self.field_components = msg.field_components

    def execute(self, goal):
        rospy.loginfo("executing state FIND_DESTINATION")

        if len(self.field_components) == 0:
            self.server.set_aborted()
            return
        
        print(len(self.field_components))
        destination_index = random.randint(0, len(self.field_components) - 1)
        self.destination_index_pub.publish(Destination(destination_index))
        self.server.set_succeeded()
        return goal

if __name__ == "__main__":
    rospy.init_node("find_destination")
    server = FindDestinationServer()
    rospy.spin()