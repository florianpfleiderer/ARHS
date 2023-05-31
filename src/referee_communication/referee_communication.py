#!/usr/bin/env python

import rospy
from globals.globals import *

from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Empty 
from referee.srv import TeamReady, SendColor, SendDimensions

def wait_for_game():
    rospy.loginfo('Waiting for game to start...')
    return rospy.wait_for_message('/gameControl', Bool)

def send_name():
    global TEAM_NAME
    
    rospy.wait_for_message('/waitForTeams', Empty)
    rospy.wait_for_service('/TeamReady')
    team_ready_srv = rospy.ServiceProxy('/TeamReady', TeamReady)
    
    name_pt = 0
    result = False
    while not result:
        name = TEAM_NAME[name_pt]
        result = team_ready_srv(name).ok
        name_pt += 1
    rospy.loginfo(f'{TEAM_NAME[name_pt]} accepted as team name.')
    rospy.loginfo('Place the robot on the field. (color -> referee)')
    TEAM_NAME = name
    
def send_color(color):
    rospy.wait_for_service('/SendColor')
    send_color_srv = rospy.ServiceProxy('/SendColor', SendColor)
    result = send_color_srv(TEAM_NAME, color)
    if not result.ok:
        return result.correctColor
    else:
        return color

def send_field_dimension(length, width):
    rospy.wait_for_service('/SendDimensions')
    send_dimension_srv = rospy.ServiceProxy('/SendDimensions', SendDimensions)
    
    dimension = Point()
    dimension.x = length
    dimension.y = width
    
    result = send_dimension_srv(TEAM_NAME, dimension)
    if not result.ok:
        return result.correctDimensions
    else:
        return dimension