#!/usr/bin/env python
import time
import rospy
from actionlib import SimpleActionServer
from player.msg import ReleasePuckAction, ReleasePuckGoal, ReleasePuckResult
from data_utils.topic_handlers import LaserSubscriber, VelocityPublisher
from geometry_msgs.msg import Twist
from globals.globals import BACKUP_THRESHOLD

class ReleasePuckServer:
    '''This server is responsible for the RELEASE_PUCK state.
    
    subscribes:
        /front_laser/scan
    
    publishes:
        /cmd_vel
        
    '''
    def __init__(self):
        self.server = SimpleActionServer("release_puck", ReleasePuckAction, self.execute, False)
        self.server.start()
        self.laser_sub = LaserSubscriber()
        self.vel_pub = VelocityPublisher()

    def set_velocities(self, linear, angular):
        """Use this to set linear and angular velocities
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def check_preempt(self):
        '''check if the action has been preempted'''
        if self.server.is_preempt_requested():
            rospy.loginfo("preempted")
            self.server.set_preempted()
            return True
        return False

    def execute(self, goal: ReleasePuckGoal):
        rospy.loginfo("executing state RELEASE_PUCK")
        result = ReleasePuckResult()

        laser_data = self.laser_sub.copy_data()

        while laser_data:
            if laser_data[len.laser_data / 2] < BACKUP_THRESHOLD:
                self.set_velocities(-0.5, 0)
            else:
                self.set_velocities(0, 0)
                result.target_color = goal.target_color
                self.server.set_succeeded(result)

        time.sleep(1)
        result.target_color = None
        self.server.set_aborted(result)

if __name__ == "__main__":
    rospy.init_node("release_puck_server")
    server = ReleasePuckServer()
    rospy.spin()