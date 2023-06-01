#!/usr/bin/env python

import copy
import rospy
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from control_architectures.msg import AvoidObstacleAction, AvoidObstacleFeedback, AvoidObstacleResult

class AvoidObstacleServer:

    feedback = AvoidObstacleFeedback()
    
    def __init__(self):
        self.server = actionlib.SimpleActionServer("avoid_obstacle", AvoidObstacleAction, self.execute, False)
        self.server.start()
        rospy.loginfo("initialised server")
        
        #self.get_laser_information = rospy.ServiceProxy('get_laser_information', GetLaserInformation)
        #self.set_velocity = rospy.ServiceProxy('set_velocity', SetVelocity)
        
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
    
    def get_direction(self, laserview):
        left_view = laserview[:int(len(laserview)/2)]
        right_view = laserview[int(len(laserview)/2):]
        inf_left = 0
        val_left = 0
        inf_right = 0
        val_right = 0
        for point in left_view:
            if point == 'inf':
                inf_left += 1
            else:
                val_left += point
        
        for point in right_view:
            if point == 'inf':
                inf_right += 1
            else:
                val_right += point
        
        if inf_left == inf_right:
            if val_left < val_right:
                return -1
            else:
                return 1
        elif inf_left < inf_right:
            return -1
        else:
            return 1
            
            
        
    
    def execute(self, goal):
        self.set_velocities(0, 0)
        rospy.loginfo("got goal")
        result = AvoidObstacleResult()
        back_up_flag = False
        turning_flag = False

        #while True:
        #    rospy.loginfo(self.get_laser_information())
        
        while not rospy.is_shutdown():
            if self.check_preempt():
                break
            
            laserview = self.get_laserview(50)
            if laserview is not None:
                rospy.loginfo(min(laserview))
                rospy.loginfo(f'{turning_flag}')
                
                if min(laserview) <= 1.2 and not turning_flag:
                    self.set_velocities(0, 0.5*self.get_direction(laserview))
                    turning_flag = True
                elif min(laserview) > 1.2:
                    rospy.loginfo('enough space to drive')
                    self.set_velocities(0, 0)
                    turning_flag = False
                    result.message = "successfully avoided"
                    self.server.set_succeeded(result)
                    break
                    
                
        

if __name__ == '__main__':
    rospy.init_node("avoid_obstacle_server")
    server = AvoidObstacleServer()
    
    rospy.spin()