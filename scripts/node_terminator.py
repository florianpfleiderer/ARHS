#!/usr/bin/env python

import rospy

from ref_com.communication import check_game_status
from data_utils.topic_handlers import VelocityPublisher
from geometry_msgs.msg import Twist

import subprocess

def set_velocities(linear, angular):
    velocity_pub = VelocityPublisher()
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    velocity_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("node_terminator")
    rospy.loginfo("Initialised node_terminator")
    rospy.loginfo("Waiting for destruction...")
    while not rospy.is_shutdown():
        result = check_game_status()
        rospy.logwarn(result)
        if result is False:
            rospy.logwarn("KILLING ALL NODES")
            set_velocities(0, 0)
            rospy.sleep(1)
            rospy.logwarn("...maybe ;)")
            #subprocess.call("rosnode kill --all", shell=True)
    