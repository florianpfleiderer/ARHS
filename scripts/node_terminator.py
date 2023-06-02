#!/usr/bin/env python

import rospy

from ref_com.communication import check_game_status
from geometry_msgs.msg import Twist

import subprocess

class NodeTerminator:
    def __init__(self):
        self.velocity_pub =rospy.Publisher("cmd_vel", Twist, queue_size=1000)
        rospy.loginfo("Initialised node_terminator")
        rospy.loginfo("Waiting for destruction...")
        
    def set_velocities(self, linear, angular):
            """Use this to set linear and angular velocities
            """
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self.velocity_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("node_terminator")
    nt = NodeTerminator()
    while not rospy.is_shutdown():
        if check_game_status() is False:
            rospy.logwarn("KILLING ALL NODES")
            nt.set_velocities(0, 0)
            rospy.sleep(1)
            rospy.logwarn("...maybe ;)")
            #subprocess.call("rosnode kill --all", shell=True)
    rospy.spin()
    