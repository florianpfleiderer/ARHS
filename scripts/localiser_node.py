#!/usr/bin/env python
import rospy

import cv2
from math import *
import sys
import time
from typing import List

from globals.tick import CallbackTicker
from globals.globals import TICK_RATE
from math_utils.vector_utils import tup2_from_polarvector2
from player.msg import FieldComponents, FieldComponent, PolarVector2, ScreenPosition
from geometry_msgs.msg import Pose, Point, Quaternion
from field_components.field import Field
from field_components.field_components import FieldObject
from field_components.colors import Color

class LocaliserNode:
    def __init__(self):
        # self.pub = rospy.Publisher('field_components', FieldComponents, queue_size=10)
        rospy.init_node("localiser_node")
        rospy.loginfo("Initialised LocaliserNode")

        self.sub = rospy.Subscriber('player/field_components', FieldComponents, self.callback)
        self.pub = rospy.Publisher('player/position', Pose, queue_size=10)

        self.field = Field()
        self.objects: List[FieldComponent] = None

    def callback(self, data: FieldComponents):
        assert isinstance(data, FieldComponents)    
        self.objects = [FieldObject(Color.from_string(c.color_name), c.type, tup2_from_polarvector2(c.player_distance), None) for c in data.field_components]

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.objects is not None:
                self.field.set_objects(self.objects)
                pos = self.field.calculate_robot_position()
                rospy.loginfo(f'Position: {pos}')
            r.sleep()
                                         
if __name__ == '__main__':
    localiser = LocaliserNode()
    # localiser.run()
    # include_field_object()
    # rospy.spin()
    rospy.loginfo("Starting loop")
    ticker = CallbackTicker(TICK_RATE,
                            # localiser.run,
                            )

    while not rospy.is_shutdown():
        ticker.tick()

