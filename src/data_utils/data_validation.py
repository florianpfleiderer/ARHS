#!/usr/bin/env python

import rospy

class Validator:    
    def guard_none(self, object):
        valid = object is not None

        if not valid:
            rospy.logerr(f"{type(object)} {str(object)} is none!")

        return valid
