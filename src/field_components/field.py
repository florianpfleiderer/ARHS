#!/usr/bin/env python
import rospy

from typing import List, Tuple
from math import pi

from globals.globals import *
from sensor_msgs.msg import LaserScan
from field_components.colors import Color
from math_utils.vector_utils import Coordinate
from referee_communication import referee_communication as ref_com
from math_utils.field_calculation_functions import cosine_theorem, get_position
from field_components.field_components import Pole, YellowPuck, BluePuck, YellowGoal, BlueGoal, FieldObject

class Field(object):
    '''Singleton class representing the field.
    It gets the field objects, divides into pucks and poles and calculates 
    the field dimensions.
    It also sets the absolute coordinates of the field objects 
    after the field dimensions are calculated.

    Attributes:
        _instance: singleton instance to make sure, only one instance of the
            class is created and if there is one, return it
        length: length of the field in meters
        width: width of the field in meters
        poles: list of poles
        pucks: list of pucks
        
    '''

    _instance = None
    epsilon = 0.1 # allowed error for the proportions of the poles (0.1=10%)

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(Field, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        self.length = None
        self.width = None
        self.poles: List[Pole] = []
        self.yellowPucks: List[YellowPuck] = []
        self.bluePucks: List[BluePuck] = []
        self.yellowGoal: YellowGoal = None
        self.blueGoal: BlueGoal = None

    def set_objects(self, field_objects: List[FieldObject]=None) -> bool:
        self.length = None
        self.width = None
        self.clear_objects()

        if field_objects:
            self.poles.extend([o for o in field_objects if o.type == "Pole"])
            self.yellowPucks.extend([o for o in field_objects if o.type == "YellowPuck"])
            self.bluePucks.extend([o for o in field_objects if o.type == "BluePuck"])
            self.yellowGoal = [o for o in field_objects if o.type == "YellowGoal"]
            self.blueGoal = [o for o in field_objects if o.type == "BlueGoal"]
            return True
        return False

    def change_coordinates(self, coordinates: Coordinate):
        '''Changes the distamce attribute to the new coordinate system.'''
        for pole in self.poles:
            pole.distance.coordinates = coordinates

    def calculate_robot_position(self) -> Tuple:
        ''' Calculates the robot position from the poles.
        
        The robot always starts in the red area of the field.
        The origin will be the right corner when looking towards the other goal.
        You can get the Robot Position depending on the Color of your goal and three Poles.
        After this, you can increment the position with the velocity data.
        
        poles.distance[0] = r
        poles.distance[2] = phi 
            (0 is the direction of robot, - is right, + is left)
        '''

        self.check_poles()

        distance_1_2 = cosine_theorem(self.poles[0], self.poles[1])
        distance_2_3 = cosine_theorem(self.poles[1], self.poles[2])
        distance_1_3 = cosine_theorem(self.poles[0], self.poles[2])

        #length = (distance_1_2 * 10 + distance_1_3 * 4) / 2
        #width = length * (3/5)
        #ref_com.send_field_dimension(length, width)
        
        self.set_field_objects_positions(distance_1_2, distance_2_3, distance_1_3)

        return get_position(self.poles[2], self.poles[1], self.poles[0])


    def set_field_objects_positions(self, distance_1_2, distance_2_3, distance_1_3):
        ''' Sets the absolute position of the field objects.

        The Poles given to this function are the outer most three poles at the 
        initial robot position. The robot always starts in the red area of the field.
        '''

        proportion_a_b = distance_2_3 / distance_1_2
        proportion_a_c = distance_1_3 / distance_1_2

        print(f'proportions: a_b: {proportion_a_b}, a_c: {proportion_a_c}')

        # set pole positions with y = 3
        if(proportion_a_b > 1.5 + 1.5*self.epsilon or proportion_a_c > 2.5 + 2.5*self.epsilon):
            rospy.logwarn('Pole ratios to large')
        elif proportion_a_b < 2/3 + 2/3*self.epsilon or proportion_a_c < 0.4 + 0.4*self.epsilon:
            rospy.loginfo('looking left')
            self.poles[0].position = (1.25, 3)
            self.poles[1].position = (0.5, 3)
            self.poles[2].position = (0, 3)
        elif proportion_a_b > 1.5 - 1.5*self.epsilon or proportion_a_c > 2.5 - 2.5*self.epsilon:
            # this means i am looking right
            rospy.loginfo('looking right')
            self.poles[0].position = (0, 0)
            self.poles[1].position = (0.5, 0)
            self.poles[2].position = (1.25, 0)

        # TODO: think about the case, when robot sees the right or left side of the field first

        # if abs(proportion_a_b - 2/3) < self.epsilon:
        #     self.poles[0].position = (5, 3)
        #     self.poles[1].position = (4.5, 3)
        #     self.poles[2].position = (3.75, 3)
        # elif abs(proportion_a_b - 3/5) < self.epsilon:
        #     self.poles[0].position = (4.5, 3)
        #     self.poles[1].position = (3.75, 3)
        #     self.poles[2].position = (2.5, 3)
        # elif abs(proportion_a_b - 1) < self.epsilon:
        #     self.poles[0].position = (3.75, 3)
        #     self.poles[1].position = (2.5, 3)
        #     self.poles[2].position = (1.25, 3)
        # elif abs(proportion_a_b - 5/3) < self.epsilon:
        #     self.poles[0].position = (2.5, 3)
        #     self.poles[1].position = (1.25, 3)
        #     self.poles[2].position = (0.5, 3)
        # elif abs(proportion_a_b - 3/2) < self.epsilon:
        #     self.poles[0].position = (1.25, 3)
        #     self.poles[1].position = (0.5, 3)
        #     self.poles[2].position = (0, 3)


    def clear_objects(self):
        self.poles.clear()
        self.yellowPucks.clear()
        self.bluePucks.clear()
        self.yellowGoal = None
        self.blueGoal = None

    def goal_found(self):
        if self.yellowGoal or self.blueGoal:
            return self.yellowGoal if self.yellowGoal else self.blueGoal
        else:
            return None

    def poles_found(self):
        return len(self.poles) > 3

    def sort_poles_by_angle_phi(self):
        # sort poles by spherical distance, so from right to left
        self.poles = sorted(self.poles, key=lambda pole: pole.distance[2])

        rospy.loginfo(f'poles: {[pole.type for pole in self.poles]}' \
                       f'{[pole.distance[2] for pole in self.poles]}')

    def check_poles(self) -> bool:
        if len(self.poles) < 3:
            rospy.logwarn('Not enough poles detected')
            # TODO function always returns this
            return False

        self.sort_poles_by_angle_phi()

        distance_1_2 = cosine_theorem(self.poles[0], self.poles[1])
        distance_2_3 = cosine_theorem(self.poles[1], self.poles[2])
        distance_1_3 = cosine_theorem(self.poles[0], self.poles[2])

        proportion_a_b = distance_2_3 / distance_1_2
        proportion_a_c = distance_1_3 / distance_1_2

        print(f'proportions: a_b: {proportion_a_b}, a_c: {proportion_a_c}')

        # check right proportions
        if(proportion_a_b > 1.5 + 1.5*self.epsilon or proportion_a_c > 2.5 + 2.5*self.epsilon):
            rospy.logwarn('Pole ratios to large')
            return False
        else:
            return True

    def outer_pole(self, direction: bool) -> Pole:
        self.sort_poles_by_angle_phi()
        if self.poles[0].position[0] == 0:
            # looking right
            return self.poles[0]
        else:
            # looking left
            return self.poles[2]

    def pole_to_laser(self,pole: Pole, cur_laser: LaserScan) -> Tuple[float, float, float]:
        ''' Transforms the pole positions to the laser scan frame. '''
        direction_rad = pole.distance[2]*pi/180

        index = int(len(cur_laser.ranges)/2 + direction_rad/cur_laser.angle_increment)

        dist_ind = int(len(cur_laser.ranges)*LASER_INDEX_MARGIN)

        outer_pole_distances = cur_laser.ranges[index - dist_ind: index + dist_ind]
        rospy.loginfo(f'Outer pole distance: {outer_pole_distances}')

        dist_min = pole.distance[0] - 0.5
        dist_max = pole.distance[0] + 0.5

        pole_indices_array: float = []

        # TODO: enumerate instead of indices
        for i in range(len(outer_pole_distances)):
            rospy.loginfo(f'Outer pole distance: {outer_pole_distances[i]}')
            if dist_min < outer_pole_distances[i] < dist_max:
                pole_indices_array.append(i)
        if pole_indices_array == []:
            rospy.logwarn("Pole indices Array empty")
            return

        rospy.loginfo(f'Pole indices: {pole_indices_array}')

        pole_found_index = pole_indices_array[len(pole_indices_array) // 2]
        rospy.loginfo(f'Pole found index: {pole_found_index}')
        total_idx = index - dist_ind + pole_found_index

        rospy.loginfo(f'Pole distance:' \
                        f'{cur_laser.ranges[total_idx]}\n' \
                        f'Angle to Pole rgb: ' \
                        f'{pole.distance[2]}\n' \
                        f'Angle to Pole laser: ' \
                        f'{(total_idx - len(cur_laser.ranges)/2)*cur_laser.angle_increment*180/pi}')

        return cur_laser.ranges[total_idx], None, (total_idx - len(cur_laser.ranges)/2)*cur_laser.angle_increment*180/pi
