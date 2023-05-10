#!/usr/bin/env python
import rospy

from math import *

from typing import List

from globals.tick import CallbackTicker
from globals.globals import TICK_RATE
from math_utils.vector_utils import tup3_from_polarvector2
from player.msg import FieldComponents, FieldComponent, PolarVector2, ScreenPosition
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from field_components.field import Field
from field_components.field_components import FieldObject
from field_components.colors import Color

class LocaliserNode:
    def __init__(self):
        # self.pub = rospy.Publisher('field_components', FieldComponents, queue_size=10)
        rospy.init_node("localiser_node")
        rospy.loginfo("Initialised LocaliserNode")

        self.comp_sub = rospy.Subscriber('player/field_components', FieldComponents, self.callback)
        self.position_pub = rospy.Publisher('player/position', Pose, queue_size=10)
        self.velocity_pub = rospy.Publisher("robot1/cmd_vel", Twist, queue_size=10) #stores 10 old data points

        self.field = Field()
        self.objects: List[FieldComponent] = None
        self.goal_found: FieldObject = None

    def callback(self, data: FieldComponents):  
        self.objects = [FieldObject(Color.from_string(c.color_name), \
                                    c.type, tup3_from_polarvector2(c.player_distance) , None) \
                        for c in data.field_components]
    
    def set_velocities(self, linear, angular):
        """Use this to set linear and angular velocities
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.velocity_pub.publish(msg)


    def execute(self):
        '''Main loop of the localiser node.
        
        '''

        while not self.objects:
            self.set_velocities(0, 0.5)
            rospy.loginfo("No objects found, turning")
        
        rospy.loginfo("Objects found, stop turning")
        self.set_velocities(0, 0)
        self.field.set_objects(self.objects)
        
        while not self.field.check_poles():
            self.set_velocities(0, 0.5)
            rospy.sleep(0.5)
        
        rospy.loginfo("outer poles found, stop turning")
        self.set_velocities(0, 0)

        pos = self.field.calculate_robot_position()
        if(pos is None):
            rospy.logwarn("No position")
            return
        
        point = Point(pos[0], pos[1], 0)
        quaternion = Quaternion(0, 0, 1, 0)
        self.position_pub.publish(Pose(point, quaternion))
        rospy.loginfo(f'Position: {pos}')
                                         
if __name__ == '__main__':
    localiser = LocaliserNode()
    # localiser.run()
    # include_field_object()
    # rospy.spin()
    rospy.loginfo("Starting loop")
    ticker = CallbackTicker(TICK_RATE,
                            localiser.execute,
                            )

    while not rospy.is_shutdown():
        ticker.tick()

