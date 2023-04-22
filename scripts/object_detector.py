#!/usr/bin/env python
import rospy

import cv2
from math import *
import sys
import time

from player.msg import FieldComponent, PolarVector2

from field_components.field_components import *
from visualization.screen_components import *
import visualization.imgops as imgops
from globals.globals import *
from math_utils.math_function_utils import *
from data_utils.topic_handlers import ImageSubscriber, LaserSubscriber
from list_utils.filtering import *
from globals.tick import *
from data_utils.laser_scan_utils import *

CLASSES = {'pole': Pole,
           'yellowpuck': YellowPuck,
           'bluepuck': BluePuck,
           'yellowgoal': YellowGoal,
           'bluegoal': BlueGoal,
           'robot': Robot}

class Detector:
    def __init__(self, detection_function):
        self.test_parameters = {}
        self.detection_function = detection_function
        self.avg_detection_time = 0
        self.detection_counter = 0
        self.counter_cap = 1000

        self.interval_start = 0
        self.printer_interval = 2

        self.detected_objects = 0

    def detect(self, *args):
        self.detection_counter += 1

        detect_start_time = time.time()
        detection_result = self.detection_function(*args)
        detect_duration = time.time() - detect_start_time

        counter = min(self.counter_cap, self.detection_counter)
        self.avg_detection_time = (self.avg_detection_time * (counter - 1) + detect_duration) / counter

        if time.time() - self.interval_start >= self.printer_interval:
            print(f"detection call {self.detection_counter}, detected {self.detected_objects} average time (last {self.counter_cap}) {self.avg_detection_time}")
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

class KinectDetector(Detector):
    def __init__(self, testmode): 
        super().__init__(self.detect_field_objects)    

        self.testmode = testmode
        
        self.rgb_sub = ImageSubscriber("rgb image", LOCAL_PLAYER + "kinect/rgb/image_raw", "bgr8")
        self.depth_sub = ImageSubscriber("depth image", LOCAL_PLAYER + "kinect/depth/image_raw", "32FC1")

        self.thresh_upper = TrackbarParameter(CANNY_THRESHOLD_UPPER, "upper", "rgb image")
        self.thresh_lower = TrackbarParameter(CANNY_THRESHOLD_LOWER, "lower", "rgb image")
        # self.add_test_parameters(self.thresh_upper, self.thresh_lower)
    
    def is_valid_data(self):
        return self.rgb_sub.is_valid() and self.depth_sub.is_valid()

    def detect_field_objects(self, base_class):
        rgb_image = self.rgb_sub.get_image()
        depth_image = self.depth_sub.get_image()

        color_mask = imgops.mask_color(rgb_image, base_class.color)
        depth_masked_image = imgops.convert_gray2bgr(imgops.apply_mask(color_mask, depth_image))
        depth_masked_image = imgops.denoise(depth_masked_image, COLOR_MASK_SMOOTHING)

        edges = imgops.edges(depth_masked_image, self.thresh_lower.get_value(self.testmode), self.thresh_upper.get_value(self.testmode))
        edges = imgops.denoise(edges, CANNY_SMOOTHING, False)

        self.add_test_parameters(TestImage("rgb image", rgb_image),
                                 TestImage("depth_masked_image", depth_masked_image),
                                 TestImage("edges", edges))
            
        contours = imgops.get_contours(edges)
        if contours == ((), None):
            return []
        
        contours = imgops.get_inner_contours(*contours)

        cv2.drawContours(rgb_image, contours, -1, Color.YELLOW.default, 1)

        def filter_cb(contour):
            x, y, w, h = cv2.boundingRect(contour)
            result = check_range(w*h, *base_class.area_detect_range)
            result &= check_range(w/h, *base_class.ratio_detect_range)
            return result

        contours = filter_list(contours, filter_cb)

        screen_objects = [ScreenObject.from_rectangle_tuple(cv2.boundingRect(contour)) for contour in contours]

        if base_class in [YellowGoal, BlueGoal, Robot] and len(screen_objects) > 1:
            screen_objects = [screen_objects[0].merge(*screen_objects[1:])]

        distances = [screen_obj.get_field_vector(depth_image) for screen_obj in screen_objects]

        field_objects = [base_class(distance, screen_obj) for distance, screen_obj in zip(distances, screen_objects)]

        self.detected_objects = len(field_objects)
        return field_objects
    
class LaserScanDetector(Detector):
    def __init__(self, testmode):
        super().__init__(self.detect_objects)
        self.laser_sub = LaserSubscriber("laser scan", LOCAL_PLAYER + "front_laser/scan")
        self.rgb_sub = ImageSubscriber("rgb image", LOCAL_PLAYER + "kinect/rgb/image_raw", "bgr8")

        self.testmode = testmode

    def is_valid_data(self):
        return self.laser_sub.is_valid()
       
    def detect_objects(self):
        laser_scan = self.laser_sub.get_scan()
        rgb_image = self.rgb_sub.get_image()

        detected_objects = []

        laser_ranges = laser_scan.ranges
        laser_ranges = range_denoise(laser_ranges, 7)

        laser_image = imgops.laser_scan_to_image(laser_scan)
        laser_image = imgops.scale(laser_image, 3)
        laser_max_angle = TrackbarParameter(SCAN_MAX_ANGLE, "max angle", "laser image")

        self.add_test_parameters(TestImage("laser image", laser_image))

        edges = []
        max_scan_angle = laser_max_angle.get_value(self.testmode)
        for index in range(laser_index(max_scan_angle, laser_scan), laser_index(-max_scan_angle, laser_scan)):
            range_diff = laser_ranges[index] - laser_ranges[index - 1]
            if range_diff < -LASER_EDGE_THRESHOLD:
                edges.append((index, True))
            
            elif range_diff > LASER_EDGE_THRESHOLD:
                edges.append((index, False))

        object_ranges = {}
        open_indices = []
        for index, rising_edge in edges:
            if rising_edge:
                object_ranges[str(index)] = (laser_theta(index, laser_scan), -1)
                open_indices.append(index)

            else:
                if len(open_indices) > 0:   
                    start_index = open_indices.pop()
                    object_ranges[str(start_index)] = (object_ranges[str(start_index)][0], laser_theta(index, laser_scan))  
                else:
                    if len(object_ranges) > 0:
                        start_index = laser_index(list(object_ranges.values())[-1][1], laser_scan)
                    else:
                        start_index = laser_index(max_scan_angle, laser_scan)
                    object_ranges[str(start_index)] = (laser_theta(start_index, laser_scan), laser_theta(index, laser_scan))

        for index in open_indices:
            object_ranges[str(index)] = (object_ranges[str(index)][0], -max_scan_angle)

        def filter_cb(element):
            distances = (laser_ranges[laser_index(element[0], laser_scan)], laser_ranges[laser_index(element[1], laser_scan) - 1])
            dist_min = min(distances)
            dist_max = max(distances)
            depth = dist_max - dist_min
            width = 2 * dist_min * sind(abs(element[1] - element[0]) / 2)

            return depth ** 2 + width ** 2 < 0.5 ** 2
        
        object_widths = filter_list(object_ranges.values(), filter_cb)

        for max_theta, min_theta in object_widths:
            center_theta = (max_theta + min_theta) / 2
            center_index = laser_index(center_theta, laser_scan)

            d = laser_ranges[center_index]

            min_alpha = KINECT_ANGLE - atand(KINECT_HEIGHT / d)
            max_alpha = min_alpha + atand(0.3 / d)

            new_distance = PolarVector2(d, center_theta)
            screen_obj = ScreenObject(min_theta, max_theta, min_alpha, max_alpha)
            detected_objects.append(FieldObject.from_field_component(FieldComponent("ORANGE", "unknown", new_distance, screen_obj.properties)))

        self.add_test_parameters(TestImage("rgb image", rgb_image))
        self.detected_objects = len(detected_objects)

        return detected_objects


if __name__ == '__main__':
    rospy.init_node("object_detector_node")
    rospy.loginfo("Initialised ObjectDetector")

    args = rospy.myargv(argv=sys.argv)
    testmode = args[1] if len(args) > 1 else True
    detect_classes = [CLASSES[key.lower()] for key in args[2:]] if len(args) > 2 else list(CLASSES.values())

    kinect_det = KinectDetector(testmode)
    laser_det = LaserScanDetector(testmode)

    def run_kinect_detection():
        if kinect_det.is_valid_data():      
            found_objects = []

            for cls in detect_classes:
                found_objects.extend(kinect_det.detect(cls))
    
            for obj in found_objects:
                obj.draw(kinect_det.test_parameters["rgb image"].get_value(testmode))

            if testmode:    
                kinect_det.show_test_parameters()
        else:
            rospy.loginfo("Waiting for images to process...")

    def run_laser_detection():       
        if laser_det.is_valid_data():
            found_objects = laser_det.detect()

            for obj in found_objects:
                obj.draw(laser_det.test_parameters["rgb image"].get_value(testmode))
                obj.draw(laser_det.test_parameters["laser image"].get_value(testmode), (360, 50), ProjectionType.SPHERICAL)

            if testmode:
                laser_det.show_test_parameters()
        else:
            rospy.loginfo("Waiting for laser scan to process...")

    
    rospy.loginfo("Starting loop")
    ticker = CallbackTicker(TICK_RATE,
                            run_kinect_detection,
                            run_laser_detection
                            )
    
    imgticker = CVTicker(TICK_RATE)

    while not rospy.is_shutdown():
        ticker.tick()
        imgticker.tick()
