#!/usr/bin/env python

import rospy
from data_utils.topic_handlers import FieldComponentsSubscriber
from geometry_msgs.msg import Twist
from typing import List
from copy import deepcopy
from globals.globals import *
from math_utils.vector_utils import TupleVector3
from player.msg import FieldComponent
from geometry_msgs.msg import Twist
from field_components.field_components import Field, FieldObject

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
        while not rospy.is_shutdown():
            components = deepcopy(self.components_sub.data)
            if components is None:
                continue
            self._filter_pucks(components)
            if len(self.yellow_pucks) > 0 and len(self.blue_pucks) > 0:
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
        self.field = Field()

    def get_dimensions(self):
        while not rospy.is_shutdown():
            if self.field.field_component_sub.data is None:
                continue
            self.field.field_objects = deepcopy(self.field.field_component_sub.data)
            poles = self.field.get_objects_by_class('Pole')
            poles = [FieldObject.from_field_component(pole) for pole in poles]
            dimensions = self.field.calculate_dimensions(*poles)
            if not dimensions:
                continue
            return dimensions   # w, l
            
        
if __name__ == "__main__":
    print('test')