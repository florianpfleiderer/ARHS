#!/usr/bin/env python

import rospy
import random
import actionlib
import copy

from control_architectures.msg import RandomDriveAction, RandomDriveFeedback, RandomDriveResult
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RandomDriveServer:

    feedback = RandomDriveFeedback()
    
    def __init__(self):
        self.server = actionlib.SimpleActionServer("drive_randomly", RandomDriveAction, self.execute, False)
        self.server.start()
        rospy.loginfo("initialised server")
        
        self.laser = None
        
        self.velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
        self.laser_sub = rospy.Subscriber("front_laser/scan", LaserScan, self.laser_cb)
    
    def check_preempt(self):
        if self.server.is_preempt_requested():
            rospy.loginfo("I was preempted")
            self.server.set_preempted()
            return True
    
    def laser_cb(self, msg):
        """This function is called every time a message is received on the
        front_laser/scan topic

        We just keep the self.laser member up to date
        """
        self.laser = msg
        
    def set_velocities(self, linear, angular):
        """Use this to set linear and angular velocities
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.velocity_pub.publish(msg)
        
    def get_laserview(self, view_degree):
        if self.laser is not None:
                cur_laser = copy.deepcopy(self.laser)
                laser_deg = int(len(cur_laser.ranges)/360)
                return cur_laser.ranges[int(laser_deg*(180-view_degree/2)):int(laser_deg*(180+view_degree/2))]
        else:
            return None
    
    def execute(self, goal):
        result = RandomDriveResult()
        rate = rospy.Rate(4) # tick twice per second
        time_counter = 0
        drive_flag = False
        turn_flag = False
        random_time = 0
        drive_time = 4*7
        
        rospy.loginfo("execute RandomDriveServer")
        while not rospy.is_shutdown():
            if self.check_preempt():
                break
            
            laserview = self.get_laserview(50)
            if laserview is not None:
                if min(laserview) < 0.6:
                    result.obstacle_found = True
                    self.feedback.message = f'obstacle {min(laserview)}m away'
                    self.server.publish_feedback(self.feedback)
                    self.server.set_succeeded(result)
                    break
                
                if time_counter < drive_time:
                    if not drive_flag:
                        self.set_velocities(0.5, 0)
                        random_time = 2*random.randrange(1, 9)
                        drive_flag = True
                elif time_counter < drive_time+random_time:
                    drive_flag = False
                    if not turn_flag:
                        self.set_velocities(0, 0.5)
                        turn_flag = True
                else:
                    turn_flag = False
                    time_counter = 0
                    
                time_counter += 1

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("drive_randomly_server")
    server = RandomDriveServer()
    
    rospy.spin()
