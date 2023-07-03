#!/usr/bin/env python
import rospy
import sys
from typing import List
import random
from globals.globals import *
from globals.tick import *
from field_components.field_components import Field, FieldObject, YellowPuck, BluePuck, YellowGoal, BlueGoal, Robot, Player
import data_utils.topic_handlers as topics
from field_components.object_detection import KinectDetector, LaserScanDetector, CLASSES
from visualization.screen_components import Screen
from visualization.screen_utils import draw_fov_bird_eye
import visualization.imgops as imgops
from player.msg import FieldComponent, FieldComponents
from geometry_msgs.msg import Vector3
from math_utils.vector_utils import TupleVector3, TupleRotator3

def fake_detection():
    pub = topics.FieldComponentsPublisher()
    rate = rospy.Rate(0.2)
    field = Field()
    field.field_objects["Pole"] = field.generate_poles(5, 3)
    field.field_objects["YellowPuck"] = [YellowPuck(TupleVector3((random.uniform(1.25, 2.5), random.uniform(0, 3), 0)), YellowPuck.default_half_size) for i in range(3)]
    field.field_objects["BluePuck"] = [BluePuck(TupleVector3((random.uniform(2.5, 3.75), random.uniform(0, 3), 0)), BluePuck.default_half_size) for i in range(3)]
    field.field_objects["YellowGoal"] = [YellowGoal(TupleVector3((0.75, 1.5, 0)), YellowGoal.default_half_size)]
    field.field_objects["BlueGoal"] = [BlueGoal(TupleVector3((4.25, 1.5, 0)), BlueGoal.default_half_size)]
    field.field_objects["Robot"] = [Robot(TupleVector3((random.uniform(3.75, 5), random.uniform(0, 3), 0)), Robot.default_half_size)]
    field.field_objects["Player"] = [Player(TupleVector3((random.uniform(0, 1.25), random.uniform(0, 3), 0)), Player.default_half_size)]
    field.angle_offset = TupleRotator3.random_xy(80)
    field.distance = field.field_objects["Player"][0].distance
    field_screen = Screen.FieldScreen("field", field)

    print("field angle ", field.angle_offset)
    print("field offset", field.distance)
    
    test_field = Field()
    test_field_screen = Screen.FieldScreen("test field", test_field)

    while not rospy.is_shutdown():
        field_screen.image = imgops.empty_image(field_screen.dimensions, (50, 50, 50))
        test_field_screen.image = imgops.empty_image(test_field_screen.dimensions, (50, 50, 50))

        for fo in field.field_objects.items():
            field_screen.draw_object(fo, False, False, False, False, True)
            field_screen.draw_object(field.get_player_relative_field_object(fo), False, True, False, False, False)
        field_screen.show_image()

        pub.publish([field.get_player_relative_field_object(fo).get_field_component() for fo in field.field_objects.items()])
        
        test_field.update()
        # test_field_screen.update()
        test_field.draw(test_field_screen, draw_cube = True)

        print("test field angle ", test_field.angle_offset)
        print("test field offset", test_field.distance)

        test_field_screen.show_image()
        rate.sleep()
        cv2.waitKey(50)

def true_detection():
    args = rospy.myargv(argv=sys.argv)
    testmode = args[1] if len(args) > 1 else False
    rospy.loginfo("Testmode: " + str(testmode))

    detect_classes = [CLASSES[key.lower()] for key in args[2:]] if len(args) > 2 else list(CLASSES.values())
    
    kinect_det = KinectDetector(testmode)
    laser_det = LaserScanDetector(testmode)
    top_screen = Screen.BirdEyeScreen("top_view")

    field_components_pub = topics.FieldComponentsPublisher()
    target_sub = topics.TargetComponentSubscriber()

    objects: List[FieldObject] = []

    field: Field = Field()
    field_screen = Screen.FieldScreen("field", field)

    screens: List[Screen] = [kinect_det.screen, laser_det.screen, laser_det.laser_screen_rgb, top_screen, field_screen]

    def init_detection_cycle():
        objects.clear()

        top_screen.image = imgops.empty_image(top_screen.dimensions)
        field_screen.image = imgops.empty_image(field_screen.dimensions, (50, 50, 50))
        draw_fov_bird_eye(KINECT_FOV, top_screen)
        draw_fov_bird_eye((SCAN_MAX_ANGLE * 2, 0), top_screen)

    def run_kinect_detection() -> List[FieldObject]:
        if kinect_det.is_valid_data():   
            found_objects = []

            for cls in detect_classes:
                kinect_det.detect(cls)
                found_objects.extend(kinect_det.detected_objects)

            screens.extend([kinect_det.screen])

            for obj in found_objects:
                kinect_det.screen.draw_object(obj, True, True, False, False, True)

            if testmode:    
                kinect_det.show_test_parameters()

            return found_objects
        else:
            rospy.loginfo("Waiting for kinect images to process...")

    def run_laser_detection() -> List[FieldObject]:       
        if laser_det.is_valid_data():
            found_objects = []

            laser_det.detect()
            found_objects.extend(laser_det.detected_objects)
              
            laser_det.laser_handler.draw_laser_points(laser_det.screen, laser_det.laser_screen_rgb)

            screens.extend([laser_det.screen, laser_det.laser_screen_rgb])

            for obj in found_objects:
                laser_det.screen.draw_object(obj, True, True, False, True, False)
                laser_det.laser_screen_rgb.draw_object(obj, True, True, False, False, True)

            if testmode:
                laser_det.show_test_parameters()

            return found_objects
        else:
            rospy.loginfo("Waiting for laser scan to process...")

    def draw_objects(objects, draw_text=True, draw_center=True, draw_icon=False, draw_rect=True, draw_cube=True, *screens: Screen):
        for screen in screens:
            for obj in objects:
                screen.draw_object(obj, draw_text, draw_center, draw_icon, draw_rect)

    def show_screens(*screens: Screen):
        for screen in screens:
            screen.show_image()

    def combine_detection():
        kinect_objects = run_kinect_detection()
        laser_objects = run_laser_detection()

        if kinect_objects is None or laser_objects is None:
            return

        combined_objects: List[FieldObject] = []

        laser_merged_indices = []
        for obj in kinect_objects:
            result_obj = obj
            for i, laser_obj in enumerate(laser_objects):
                if laser_obj.distance.angle(obj.distance) < 5 and laser_obj.distance.distance(obj.distance)/laser_obj.distance.length() < 0.1:
                    laser_merged_indices.append(i)
                    result_obj.distance = laser_obj.distance
                    # result_obj = result_obj.merge(laser_obj, return_type=type(obj))
                    print("merged")

            combined_objects.append(result_obj)

        for i, laser_obj in enumerate(laser_objects):
            if i not in laser_merged_indices:
                combined_objects.append(laser_obj)

        rospy.loginfo(f"Detected {len(combined_objects)} field components")
        
        if combined_objects and len(combined_objects) > 0:
            field_components_pub.publish(FieldComponents(
                [FieldComponent(o.color.__str__() , o.type, Vector3(*o.distance.tuple),
                                None ) for o in combined_objects]))

        field.update()
        field_screen.update()

        draw_objects(combined_objects, False, False, False, False, True, top_screen)
        draw_objects(combined_objects, False, False, False, False, True, field_screen)
        field.draw(field_screen)

    def kinect_warp_correction():
        kinect_objects = run_kinect_detection()
        laser_objects = run_laser_detection()

        if kinect_objects is None or laser_objects is None:
            return

        draw_objects(kinect_objects, False, False, False, True, top_screen)
        draw_objects(laser_objects, False, False, False, True, top_screen)

    def draw_target(screen: Screen):
        if target_sub.data is not None:
            screen.draw_object(FieldObject.from_field_component(target_sub.data), False, True)

    rospy.loginfo("Starting loop")
    ticker = CallbackTicker(TICK_RATE,
                            init_detection_cycle,
                            # lambda: draw_objects(run_kinect_detection, False, True, top_screen),
                            # lambda: draw_objects(run_laser_detection, False, True, top_screen),
                            combine_detection,
                            # kinect_warp_correction,
                            # lambda: draw_target(top_screen),
                            # lambda: show_screens(*screens),
                            # lambda: show_screens(top_screen),
                            lambda: show_screens(top_screen, field_screen)
                            )

    imgticker = CVTicker(TICK_RATE)

    while not rospy.is_shutdown():
        ticker.tick()
        imgticker.tick()


if __name__ == '__main__':
    rospy.init_node("object_detector")
    rospy.loginfo("Initialised ObjectDetector")

    fake_detection()