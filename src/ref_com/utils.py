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
from field_components.field_components import Field

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
        self.velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
        self.components_sub = FieldComponentsSubscriber()
        self.components: List[FieldComponent] = None

    def rotate(self, speed):
            """Use this to set linear and angular velocities
            """
            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = speed
            self.velocity_pub.publish(msg)
    
    def remove_generic_objects(self):
        filtered_components = []
        for component in self.components:
            if component.type != 'GenericObject':
                filtered_components.append(component)
        self.components = filtered_components
                
    
    def get_dimensions(self):
        self.rotate(-0.2)
        while self.components is None:
            self.components = deepcopy(self.components_sub.data)
        while not rospy.is_shutdown():
            self.components = deepcopy(self.components_sub.data)
            self.remove_generic_objects()
            if len(self.components) == 0:
                break
            
        self.rotate(0.2)
        while not rospy.is_shutdown():
            ok = self.field.update()
            w = self.field.half_size[0] * 2
            l = self.field.half_size[1] * 2
            if w != 0 and l != 0:
                self.rotate(0)
                return w, l
            
            
        
if __name__ == "__main__":
    print('test')