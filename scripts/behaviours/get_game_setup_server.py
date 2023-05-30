#!/usr/bin/env python
from typing import List
import rospy
from player.msg import GetGameSetupAction, GetGameSetupResult
from actionlib import SimpleActionServer
from globals.globals import *
from data_utils.topic_handlers import *
from ref_com.communication import *
from ref_com.utils import TeamColorUtil, LocaliserUtil


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
        self.field_components_sub = FieldComponentsSubscriber()
        self.field_components: List[FieldComponent] = None
        

    def check_preempt(self):
        '''check if the action has been preempted'''
        if self.server.is_preempt_requested():
            rospy.loginfo("preempted")
            self.server.set_preempted()
            return True
        return False
    
    def execute(self, goal):
        result = GetGameSetupResult()
        rospy.logwarn('STARTUNG:::::::::::::::::::')
        wait_for_referee()
        self.teamname = send_names('RoBros', 'Los RosBros', 'Terminators')
        if SIMULATION_MODE:
            opponent_name = 'Bösewicht'
            send_names(opponent_name)
        wait_for_start()
        
        #4 localise
        localiser_util = LocaliserUtil()
        self.dimensions = localiser_util.get_dimensions()
        self.dimensions = send_field_dimension(self.teamname, self.dimensions[0], self.dimensions[1])
        
        #5 send team color
        team_color_util = TeamColorUtil()
        self.color = team_color_util.determine()
        self.color = send_color(self.teamname, self.color)
        team_color_util.get_first_target()
        
        #5.5 send opponent color
        if SIMULATION_MODE:
            opponent_color = "yellow" if self.color == "blue" else "blue"
            send_color(opponent_name, opponent_color)
        
        target = team_color_util.get_first_target()
        result.target_component = target
        rospy.logwarn(f"target acquired: {target}")
        self.server.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("get_game_setup_server")
    server = GetGameSetupServer()
    rospy.spin()