#!/usr/bin/env python

import rospy
from globals.globals import *

from std_msgs.msg import Bool, Empty
from referee.srv import TeamReady, SendColor, SendDimensions
from geometry_msgs.msg import Point

def wait_for_referee():
    rospy.logwarn('Waiting for referee to be ready...')
    rospy.wait_for_message('/waitForTeams', Empty)

def wait_for_start():
    rospy.logwarn('Waiting for game to start...')
    return rospy.wait_for_message('/gameControl', Bool)

def send_names(*names):
    rospy.wait_for_service('/TeamReady')
    team_ready_srv = rospy.ServiceProxy('/TeamReady', TeamReady)
    for name in names:
        if team_ready_srv(name).ok:
            return name

def send_color(name, color):
    rospy.wait_for_service('/SendColor')
    send_color_srv = rospy.ServiceProxy('/SendColor', SendColor)
    result = send_color_srv(name, color)
    if result.ok:
        return color
    else:
        return result.correctColor        

def send_field_dimension(name, width, length):
    rospy.wait_for_service('/SendDimensions')
    send_dimension_srv = rospy.ServiceProxy('/SendDimensions', SendDimensions)
    
    dimensions = Point()
    dimensions.x = length
    dimensions.y = width
    dimensions.z = 0
    rospy.logwarn(f'calculated dimensions={dimensions}')
    result = send_dimension_srv(name, dimensions)
    rospy.logwarn(f'{result.correctDimensions=}')
    return result.correctDimensions