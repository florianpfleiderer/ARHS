#!/usr/bin/env python

import rospy
from data_utils.topic_handlers import VelocityPublisher, FieldComponentsSubscriber
from geometry_msgs.msg import Twist
from typing import List
from copy import deepcopy
from globals.globals import *
from math_utils.vector_utils import TupleVector3
from player.msg import FieldComponent
from geometry_msgs.msg import Twist
import field_components.field_components as fc

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
    
    def determine_color(self):
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
        self.field = fc.Field()
        
    def get_dimensions(self):
        while not rospy.is_shutdown():
            if self.field.field_component_sub.data is None:
                continue
            self.field.field_objects = deepcopy(self.field.field_component_sub.data)
            poles = self.field.get_objects_by_class('Pole')
            dimensions = self.field.calculate_dimensions(*poles)[0]
            return dimensions   # w, l
            
        
if __name__ == "main":
    f  = fc.Field()
    print("done")