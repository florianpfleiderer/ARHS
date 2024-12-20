#!/usr/bin/env python

import cv2
import rospy
import threading

class RospyTicker:
    def __init__(self, rate):
        self.rate = rospy.Rate(rate)
        self.delta_ms = int(1000 / rate)
        self.thread = threading.Thread(target=self.start)

    def start(self):
        while not rospy.is_shutdown():
            self.tick()

    def tick(self):
        self.tick_action()
        self.rate.sleep()

    def tick_action(self):
        pass

class CVTicker(RospyTicker):
    def __init__(self, rate):
        super().__init__(rate)
    
    def tick_action(self):
        cv2.waitKey(self.delta_ms)

class CallbackTicker(RospyTicker):
    def __init__(self, rate, *callback_functions):
        super().__init__(rate)
        self.callback_functions = list(callback_functions)

    def tick_action(self):
        for func in self.callback_functions:
            func()