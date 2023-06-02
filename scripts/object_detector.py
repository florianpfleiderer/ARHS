#!/usr/bin/env python
import rospy

import cv2
from math import *
import sys
import time
from typing import List

from player.msg import FieldComponent, FieldComponents

from field_components.field_components import FieldObject, Pole, YellowPuck, BluePuck, YellowGoal, BlueGoal, Robot, Field
from visualization.screen_components import Screen, TrackbarParameter, TestImage
import visualization.imgops as imgops
from globals.globals import *
import math_utils.math_function_utils as mf
import data_utils.topic_handlers as topics
from globals.tick import *
import data_utils.laser_scan_utils as laser
from data_utils.laser_scan_utils import LaserScanHandler
from geometry_msgs.msg import Vector3


CLASSES = {'pole': Pole,
           'yellowpuck': YellowPuck,
           'bluepuck': BluePuck,
           'yellowgoal': YellowGoal,
           'bluegoal': BlueGoal,
           'robot': Robot}

class FieldDetector:
    def __init__(self, detection_function, screen: Screen):
        self.test_parameters = {}
        self.detection_function = detection_function
        self.avg_detection_time = 0
        self.detection_counter = 0
        self.counter_cap = 1000

        self.interval_start = 0
        self.printer_interval = 2

        self.detected_objects: List[FieldObject] = []
        self.screen: Screen = screen

    def detect(self, *args):
        self.detection_counter += 1

        detect_start_time = time.perf_counter()
        detection_result = self.detection_function(*args)
        detect_duration = time.perf_counter() - detect_start_time

        counter = min(self.counter_cap, self.detection_counter)
        self.avg_detection_time = (self.avg_detection_time * (counter - 1) + detect_duration) / counter

        if time.time() - self.interval_start >= self.printer_interval:
            # print(f"detection call {self.detection_counter}, detected {len(self.detected_objects)} average time (last {self.counter_cap}) {self.avg_detection_time}")
            self.interval_start = time.time()

        return detection_result

    def show_test_parameters(self):
        for param in list(self.test_parameters.values()):
            param.show()

    def add_test_parameters(self, *params):
        for param in params:
            try:
                name = param.name

            except AttributeError:
                name = str(type(param))

            self.test_parameters[name] = param

    def add_result(self, obj):
        self.detected_objects.append(obj)

class KinectDetector(FieldDetector):
    def __init__(self, testmode): 
        super().__init__(self.detect_field_objects, Screen.KinectScreen("kinect image"))    

        self.testmode = testmode
        
        self.rgb_sub = topics.RGBSubscriber()
        self.depth_sub = topics.DepthSubscriber()

        self.thresh_upper = TrackbarParameter(CANNY_THRESHOLD_UPPER, "upper", "kinect image")
        self.thresh_lower = TrackbarParameter(CANNY_THRESHOLD_LOWER, "lower", "kinect image")
        self.depth_offset = TrackbarParameter(KINECT_OFFSET[0], "laser depth offset", "laser rgb image", -50, 50, 0.01)
        self.lateral_offset = TrackbarParameter(KINECT_OFFSET[1], "laser lateral offset", "laser rgb image", -50, 50, 0.01)
        self.height_offset = TrackbarParameter(KINECT_OFFSET[2], "laser height offset", "laser rgb image", 0, 100, 0.01)

        # self.lens_correction_const = TrackbarParameter(0, "const", "top_view", -50, 50, 0.01)
        # self.lens_correction_lin = TrackbarParameter(0, "lin", "top_view", -50, 50, 0.01)
        # self.lens_correction_quad = TrackbarParameter(0.2, "quad", "top_view", 0, 100, 0.01)
        # self.lens_correction_quart = TrackbarParameter(0.6, "quart", "top_view", 0, 100, 0.01)
        self.lens_correction_ang = TrackbarParameter(0, "ang", "top_view", 0, 100, 0.01)

        img = imgops.empty_image(KINECT_DIMENSIONS)
        self.add_test_parameters(TestImage("depth_masked_image", img),
                                 TestImage("edges", img))
    
    def is_valid_data(self):
        return self.rgb_sub.is_valid() and self.depth_sub.is_valid()

    def detect_contours(self, base_class, rgb_image, depth_image):
        color_mask = imgops.mask_color(rgb_image, base_class.color)
        color_mask = imgops.denoise(color_mask, COLOR_MASK_SMOOTHING)
        depth_masked_image = imgops.convert_gray2bgr(imgops.apply_mask(color_mask, depth_image))
        depth_masked_image = imgops.denoise(depth_masked_image, COLOR_MASK_SMOOTHING, False)
        
        edges = imgops.edges(depth_masked_image, self.thresh_lower.get_value(self.testmode), self.thresh_upper.get_value(self.testmode))
        edges = imgops.denoise(edges, CANNY_SMOOTHING, False)
        
        self.test_parameters["depth_masked_image"].set_value(depth_masked_image)
        self.test_parameters["edges"].set_value(edges)
            
        contours = imgops.get_contours(edges)

        if contours == ((), None):
            return None
        
        contours = imgops.get_inner_contours(*contours)
        #cv2.drawContours(rgb_image, contours, -1, Color.YELLOW.default, 1)
           
        
        return contours

    def detect_field_objects(self, base_class: FieldObject):
        rgb_image = self.rgb_sub.copy_data()
        depth_image = self.depth_sub.copy_data()

        self.detected_objects.clear()
        self.screen.image = rgb_image
        contours = self.detect_contours(base_class, rgb_image, depth_image)
        if contours is None:
            return False

        rects = [cv2.boundingRect(c) for c in contours]
        rects = [rect for rect in rects if mf.check_range(rect[2] * rect[3], *base_class.area_detect_range) and \
                                           mf.check_range(rect[2] / rect[3], *base_class.ratio_detect_range)]     
        

        field_objects = []

        for rect in rects:
            x, y, w, h = rect
            image_w = depth_image.shape[1]    
            image_h = depth_image.shape[0] 
            cx = min(int(x + w/2), image_w)
            cy = min(int(y + h/2), image_h)
            r = depth_image[cy, cx] if SIMULATION_MODE else depth_image[cy, cx] / 1000

            # correction for lens warp
            edge_dist = (abs(cx/image_w - 1/2) + abs(cy/image_h - 1/2))
            r *= 1 + 0.14 * (edge_dist) ** 2 + 0.53 * (edge_dist) ** 4

            if mf.check_range(r, *KINECT_RANGE):
                fo = self.screen.create_field_object(rect, r, base_class)
                field_objects.append(fo)

        if base_class in [YellowGoal, BlueGoal, Robot] and len(field_objects) > 1:
            field_objects = [field_objects[0].merge(*field_objects[1:], return_type=base_class)]

        self.detected_objects.extend(field_objects)
        
        return True
    
class LaserScanDetector(FieldDetector):
    def __init__(self, testmode):
        super().__init__(self.detect_objects, Screen.LaserScreen("laser image"))

        self.laser_sub = topics.LaserSubscriber()
        self.rgb_sub = topics.RGBSubscriber()
        self.laser_handler = LaserScanHandler(self.laser_sub.copy_data())

        self.testmode = testmode
        self.depth_offset = TrackbarParameter(LASER_OFFSET[0], "laser depth offset", "laser rgb image", -50, 50, 0.01)
        self.lateral_offset = TrackbarParameter(LASER_OFFSET[1], "laser lateral offset", "laser rgb image", -50, 50, 0.01)
        self.height_offset = TrackbarParameter(LASER_OFFSET[2], "laser height offset", "laser rgb image", 0, 100, 0.01)

        self.laser_screen_rgb = Screen.KinectScreen("laser rgb image")

    def is_valid_data(self):
        return self.laser_sub.is_valid()    
    
    def detect_contours(self, laser_scan_handler: LaserScanHandler):
        edges = laser.detect_edges(laser_scan_handler)
        laser.draw_edges(self.laser_handler, edges, self.screen)
        object_ranges = laser.detect_contours(laser_scan_handler, edges)

        object_contours = [cont for cont in object_ranges if abs(cont.end_angle - cont.start_angle) < 10]

        return object_contours
       
    def detect_objects(self):
        laser_scan = self.laser_sub.copy_data()
        self.laser_handler.update(laser_scan)
        rgb_image = self.rgb_sub.copy_data()
        self.laser_screen_rgb.image = rgb_image

        laser_ranges = self.laser_handler.get_ranges()
        laser_ranges = laser.range_denoise(laser_ranges, 5)

        laser_image = imgops.laser_scan_to_image(laser_scan, self.screen.dimensions)
        self.screen.image = laser_image

        self.detected_objects.clear()

        object_ranges = self.detect_contours(self.laser_handler)

        for contour in object_ranges:
            fo = laser.object_from_contour(contour, self.laser_handler, self.screen)
            if fo is not None:
                self.add_result(fo)

        return True


if __name__ == '__main__':
    rospy.init_node("object_detector")
    rospy.loginfo("Initialised ObjectDetector")

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
        imgops.draw_fov_bird_eye(KINECT_FOV, top_screen)
        imgops.draw_fov_bird_eye((SCAN_MAX_ANGLE * 2, 0), top_screen)

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
                            #lambda: show_screens(*screens),
                            # lambda: show_screens(top_screen),
                            #lambda: show_screens(top_screen, field_screen, kinect_det.screen)
                            lambda: show_screens(kinect_det.screen)
                            )

    imgticker = CVTicker(TICK_RATE)

    while not rospy.is_shutdown():
        ticker.tick()
        imgticker.tick()
