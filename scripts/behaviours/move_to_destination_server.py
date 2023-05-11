#!/usr/bin/env python

import rospy
from player.msg import *
from geometry_msgs.msg import Twist
from math_utils.vector_utils import *
from actionlib import SimpleActionServer
from globals.globals import *

class MoveToDestinationServer:
    def __init__(self):
        self.server = SimpleActionServer("move_to_destination", MoveToDestinationAction, self.execute, False)
        self.server.start()

        self.vel_pub = rospy.Publisher("robot1/cmd_vel", Twist, queue_size=500)

    def execute(self, goal):
        rospy.loginfo("executing state MOVE_TO_DESTINATION")

        attracting = TupleVector3()
        repelling = TupleVector3()

        for i, component in enumerate(goal.field_components):
            dist = component.player_distance
            pos = TupleVector3((dist.x, dist.y, dist.z))
            if i == goal.destination_index:
                attracting.add(pos)

            else:
                repelling.add(pos)

        if attracting.lt(TupleVector3((TARGET_REACHED_R_THRESHOLD, TARGET_REACHED_THETA_THRESHOLD, 360))):
            self.vel_pub.publish(Twist())
            self.server.set_succeeded()
            return True

        result_force = attracting.factor(ATTRACTION_FACTOR).subtract(repelling.factor(REPULSION_FACTOR)).convert(Coordinate.CYLINDRICAL).value()
        
        out_velocity = Twist()
        out_velocity.linear.x = result_force[0]
        out_velocity.angular.z = result_force[1]

        self.vel_pub.publish(out_velocity)

        self.server.set_succeeded()

        return False
    
if __name__ == "__main__":
    rospy.init_node("move_to_destination")
    server = MoveToDestinationServer()
    rospy.spin()