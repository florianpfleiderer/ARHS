#!/usr/bin/env python
import time
import rospy
from actionlib import SimpleActionServer
from player.msg import GetGameSetupAction, GetGameSetupResult


class GetGameSetupServer:
    def __init__(self):
        self.server = SimpleActionServer("get_game_setup", GetGameSetupAction,\
                                         self.execute, False)
        self.server.start()
        rospy.loginfo("GetGameSetupServer initialised")

    def check_preempt(self):
        '''check if the action has been preempted'''
        if self.server.is_preempt_requested():
            rospy.loginfo("preempted")
            self.server.set_preempted()
            return True
        return False

    def execute(self, goal):
        rospy.loginfo("executing state GET_GAME_SETUP")
        result = GetGameSetupResult()

        #self.server.set_aborted(result)
        #target = field_components[random.randint(0, len(field_components) - 1)]
        #result.target_component = target
        #rospy.loginfo(f"target acquired: {target}")
        time.sleep(2)
        self.server.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("get_game_setup_server")
    server = GetGameSetupServer()
    rospy.spin()