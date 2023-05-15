#!/usr/bin/env python
import rospy

from math import *
import copy

from typing import List

from globals.tick import CallbackTicker
from globals.globals import *
from math_utils.vector_utils import TupleVector3
from math_utils.field_calculation_functions import get_position
from player.msg import FieldComponents, FieldComponent
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from field_components.field import Field
from field_components.field_components import FieldObject, Pole
from field_components.colors import Color
from sensor_msgs.msg import LaserScan

class LocaliserNode:
    def __init__(self):
        # self.pub = rospy.Publisher('field_components', FieldComponents, queue_size=10)
        rospy.init_node("localiser_node")
        rospy.loginfo("Initialised LocaliserNode")

        self.comp_sub = rospy.Subscriber('player/field_components', FieldComponents, self.callback)
        self.laser_sub = rospy.Subscriber("/robot1/front_laser/scan", LaserScan, self.laser_cb)
        self.position_pub = rospy.Publisher('player/position', Pose, queue_size=10)
        self.velocity_pub = rospy.Publisher("robot1/cmd_vel", Twist, queue_size=10) #stores 10 old data points

        self.field = Field()
        self.objects: List[FieldComponent] = None
        self.goal_found: FieldObject = None
        self.laser: LaserScan = None

    def callback(self, data: FieldComponents):  
        self.objects = [FieldObject(Color.from_string(c.color_name), \
                                    c.type, TupleVector3.from_vector3(c.player_distance) , None) \
                        for c in data.field_components]
        
    def laser_cb(self, msg: LaserScan):
        self.laser = msg
    
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

        # while len(self.field.poles) < 3:
        #     self.set_velocities(0, 0.2)
        #     self.field.set_objects(self.objects)
        #     rospy.loginfo("Not enough poles found, turning")
        
        # rospy.loginfo("Objects found, stop turning")
        # self.set_velocities(0, 0)
        
        # while not self.field.check_poles():
        #     self.set_velocities(0, 0.2)
        #     rospy.sleep(0.5)
        
        # rospy.loginfo("outer poles found, stop turning")
        # self.set_velocities(0, 0)
        while not rospy.is_shutdown():
            cur_objects = copy.deepcopy(self.objects)
            cur_laser = copy.deepcopy(self.laser)

            if cur_objects is None:
                rospy.logwarn("No objects")
                return
            
            if not self.field.set_objects(cur_objects):
                rospy.logwarn("Field Objects could not be set")
                return

            if not self.field.check_poles():
                return

            pos = self.field.calculate_robot_position()
            if(pos is None):
                rospy.logwarn("No position")
                return
            
            # TODO auto detect left or right
            outer_pole: Pole = self.field.outer_pole(False)
            mid_pole: Pole = self.field.poles[1]

            outer_pole.spherical_distance = self.field.pole_to_laser(outer_pole, cur_laser)
            mid_pole.spherical_distance = self.field.pole_to_laser(mid_pole, cur_laser)

            pos = get_position(mid_pole, outer_pole, None)

            point = Point(pos[0], pos[1], 0)
            quaternion = Quaternion(0, 0, 1, 0)
            self.position_pub.publish(Pose(point, quaternion))
            rospy.loginfo(f'Position: {pos}')
            rospy.sleep(1)
                                         
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

