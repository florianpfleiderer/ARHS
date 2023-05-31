#!/usr/bin/env python
import time
import rospy
from player.msg import ReleasePuckAction, ReleasePuckResult
from actionlib import SimpleActionServer

class GetGameSetupServer:
    def __init__(self):
        self.server = SimpleActionServer("release_puck", ReleasePuckAction,\
                                         self.execute, False)
        self.server.start()

    def check_preempt(self):
        '''check if the action has been preempted'''
        if self.server.is_preempt_requested():
            rospy.loginfo("preempted")
            self.server.set_preempted()
            return True
        return False

    def execute(self, goal):
        rospy.loginfo("executing state RELEASE_PUCK")
        result = ReleasePuckResult()

        #self.server.set_aborted(result)
        #target = field_components[random.randint(0, len(field_components) - 1)]
        #result.target_component = target
        #rospy.loginfo(f"target acquired: {target}")
        time.sleep(3)
        self.server.set_succeeded(result)

if __name__ == "__main__":
    rospy.init_node("release_puck_server")
    server = GetGameSetupServer()
    rospy.spin()