#!/usr/bin/env python
import rospy

from math import *
import copy

from typing import List

from globals.tick import CallbackTicker
from globals.globals import *
from math_utils.vector_utils import tup3_from_polarvector2
from player.msg import FieldComponents, FieldComponent, PolarVector2, ScreenPosition
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
                                    c.type, tup3_from_polarvector2(c.player_distance) , None) \
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
            
            outer_pole: Pole = self.field.outer_pole(False)

            direction_rad = outer_pole.spherical_distance[2]*pi/180
            
            index = int(180 + direction_rad/cur_laser.angle_increment)

            outer_pole_distances = cur_laser.ranges[index - 5: index + 5]
            rospy.loginfo(f'Outer pole distance: {outer_pole_distances}')

            dist_min = outer_pole.spherical_distance[0] - 0.5
            dist_max = outer_pole.spherical_distance[0] + 0.5

            pole_indices_array: float = []

            for i in range(len(outer_pole_distances)):
                rospy.loginfo(f'Outer pole distance: {outer_pole_distances[i]}')
                if dist_min < outer_pole_distances[i] < dist_max:
                    pole_indices_array.append(i)

            pole_found_index = pole_indices_array[len(pole_indices_array) // 2]
            rospy.loginfo(f'Pole found index: {pole_found_index}')
            total_idx = index - 5 + pole_found_index
            
            rospy.loginfo(f'Pole indices: {pole_indices_array}')

            rospy.loginfo(f'Outer pole distance:' \
                          f'{cur_laser.ranges[total_idx]}\n' \
                          f'Angle to Pole rgb: ' \
                          f'{outer_pole.spherical_distance[2]}\n' \
                          f'Angle to Pole laser: ' \
                          f'{(total_idx - 180)*cur_laser.angle_increment*180/pi}')
            
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

