#!/usr/bin/env python
from typing import List
import rospy
from player.msg import GetGameSetupAction, GetGameSetupResult
from actionlib import SimpleActionServer
from globals.globals import *
from ref_com.communication import *
from ref_com.utils import TeamColorUtil, LocaliserUtil
from data_utils.topic_handlers import VelocityPublisher, FieldDimensionsPublisher
from geometry_msgs.msg import Twist


class GetGameSetupServer:
    def __init__(self):
        self.server = SimpleActionServer("get_game_setup", GetGameSetupAction,\
                                         self.execute, False)
        self.server.start()
        rospy.loginfo("GetGameSetupServer initialised")
        self.teamname = None
        self.color = None
        self.dimensions = None
        self.velocity_pub = VelocityPublisher()
        self.fielddimensions_pub = FieldDimensionsPublisher()
        
    def set_velocities(self, linear, angular):
        """Use this to set linear and angular velocities
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.velocity_pub.publish(msg)

    def check_preempt(self):
        '''check if the action has been preempted'''
        if self.server.is_preempt_requested():
            rospy.loginfo("preempted")
            self.server.set_preempted()
            return True
        return False
    
    def execute(self, goal):
        wait_for_referee()
        self.teamname = send_names('RoBros', 'Los RosBros', 'Terminators')
        if SIMULATION_MODE:
            opponent_name = 'Bösewicht'
            send_names(opponent_name)
        
        if check_game_status() is True:
            pass
        
        #4 localise
        localiser_util = LocaliserUtil()
        self.dimensions = localiser_util.get_dimensions()
        self.dimensions = send_field_dimension(self.teamname, self.dimensions[0], self.dimensions[1])
        self.fielddimensions_pub.publish(self.dimensions[0], self.dimensions[1])
        
        #5 send team color
        team_color_util = TeamColorUtil()
        self.color = team_color_util.determine_color()
        self.color = send_color(self.teamname, self.color)
        
        #5.5 send opponent color
        if SIMULATION_MODE:
            opponent_color = "yellow" if self.color == "blue" else "blue"
            send_color(opponent_name, opponent_color)
        
        
        result = GetGameSetupResult()
        result.target_type = team_color_util.get_first_target().type
        rospy.logwarn(f'{result.target_type=}')
        self.server.set_succeeded(result)

if __name__ == "__main__":
    print('test')
    rospy.init_node("get_game_setup_server")
    server = GetGameSetupServer()
    rospy.spin()