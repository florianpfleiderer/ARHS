#!/usr/bin/env python

import rospy
import smach

from smach_ros import SimpleActionState, IntrospectionServer

from control_architectures.msg import RandomDriveAction, AvoidObstacleAction


if __name__ == '__main__':
    rospy.init_node("smach_example")

    sm = smach.StateMachine(outcomes=["preempted", "aborted", "succeeded"])

    with sm:

        @smach.cb_interface(outcomes=["obstacle_found"])
        def random_drive_cb(userdata, status, result):
            rospy.loginfo("random drive result: {}".format(result))
            rospy.loginfo(result)
            if result.obstacle_found:
                return "obstacle_found"

        def avoid_obstacle_cb(userdata, status, result):
            rospy.loginfo("avoid obstacle result: {}".format(result))
            if result.message == "crashed":
                return "aborted"
            else:
                return "succeeded"

        smach.StateMachine.add("DRIVE_RANDOMLY",
                                SimpleActionState("drive_randomly",
                                                 RandomDriveAction,
                                                 result_cb=random_drive_cb),
                                transitions={"obstacle_found": "AVOID_OBSTACLE"})

        smach.StateMachine.add("AVOID_OBSTACLE",
                                SimpleActionState("avoid_obstacle",
                                                 AvoidObstacleAction,
                                                 result_cb=avoid_obstacle_cb),
                                transitions={"succeeded": "DRIVE_RANDOMLY",
                                            "aborted": "AVOID_OBSTACLE"})

    sis = IntrospectionServer("smach_example", sm, "/SM_ROOT")
    sis.start()

    outcome = sm.execute()
    rospy.spin()
    sis.stop()
