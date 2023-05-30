#!/usr/bin/env python

import rospy
from data_utils.topic_handlers import *
from geometry_msgs.msg import Twist
from typing import List
from field_components.field import Field
from copy import deepcopy
from globals.tick import CallbackTicker
from globals.globals import *
from math_utils.vector_utils import TupleVector3, Coordinate
from math_utils.field_calculation_functions import get_position
from player.msg import FieldComponents, FieldComponent
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from field_components.field import Field
from field_components.field_components import FieldObject, Pole
from field_components.colors import Color
from sensor_msgs.msg import LaserScan
from data_utils.topic_handlers import *

def set_velocities(linear, angular):
    velocity_pub = VelocityPublisher()
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    velocity_pub.publish(msg)
    
class TeamColorUtil:
    def __init__(self):
        self.yellow_pucks = []
        self.blue_pucks = []
        self.components_sub = FieldComponentsSubscriber()
        self.components: List[FieldComponent] = None
        self.teamcolor = None
    
    def _filter_pucks(self, components: List[FieldComponent]):
        for component in components:
            if component.type == 'YellowPuck':
                self.yellow_pucks.append(component)    
            elif component.type == 'BluePuck':
                self.blue_pucks.append(component)
                  
    def _average_distance(self, components: List[FieldComponent]):
        dist = 0
        for component in components:
            dist += TupleVector3.from_vector3(component.player_distance)
        return dist / len(components)
    
    def determine(self):
        set_velocities(0, 0.5)
        while not rospy.is_shutdown():
            components = deepcopy(self.components_sub.data)
            if components is None:
                rospy.logwarn("No field components")
                continue
            self._filter_pucks(components)
            if len(self.yellow_pucks) > 0 and len(self.blue_pucks) > 0:
                set_velocities(0, 0)
                break
        if self._average_distance(self.yellow_pucks) > self._average_distance(self.blue_pucks):
            self.teamcolor =  'blue'
        else:
            self.teamcolor = 'yellow'
        return self.teamcolor
    
    def get_first_target(self):
        if self.teamcolor is None:
            rospy.logwarn('Teamcolor must be determined to get first target.')
            return None
        else:
            return self.yellow_pucks[0] if self.teamcolor == 'yellow' else self.blue_pucks[0]
        
        
class LocaliserUtil:
    def __init__(self):
        self.comp_sub = FieldComponentsSubscriber()
        self.laser_sub = LaserSubscriber()
        self.position_pub = rospy.Publisher('player/position', Pose, queue_size=10)
        
        self.field = Field()
        self.position = None
        self.dimensions = None
        self.objects: List[FieldComponent] = None
        self.laser: LaserScan = None

    def get_position(self):
        pass
    
    def get_dimensions(self):
        set_velocities(0, 0.2)
        while not rospy.is_shutdown():
            if self.comp_sub.data is None:
                continue
            self.objects = [FieldObject(Color.from_string(c.color_name), c.type, TupleVector3.from_vector3(c.player_distance) , None) for c in self.comp_sub.data]
            
            self.laser = self.laser_sub.data
            
            
                 
            cur_objects = deepcopy(self.objects)
            cur_laser = deepcopy(self.laser)
            
            if cur_objects is None:
                rospy.logwarn("No objects")
                return

            if not self.field.set_objects(cur_objects):
                rospy.logwarn("Field Objects could not be set")
                return
            
            self.field.change_coordinates(Coordinate.SPHERICAL)

            if not self.field.check_poles():
                return

            pos = self.field.calculate_robot_position()
            if pos is None:
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
            unit = self.field.calculate_unit()
            rospy.logwarn(f'###########{unit=}')
            return 3*unit, 5*unit   # width, length
        
        

        