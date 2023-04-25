#!/usr/bin/env python
import rospy

import cv2
from math import *
import sys
import time

from player.msg import *

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

class FieldDetector:
    def __init__(self, detection_function, screen):
        self.test_parameters = {}
        self.detection_function = detection_function
        self.avg_detection_time = 0
        self.detection_counter = 0
        self.counter_cap = 1000

        self.interval_start = 0
        self.printer_interval = 2

        self.detected_objects = []
        self.screen = screen

    def detect(self, *args):
        self.detection_counter += 1

        detect_start_time = time.time()
        detection_result = self.detection_function(*args)
        detect_duration = time.time() - detect_start_time

        counter = min(self.counter_cap, self.detection_counter)
        self.avg_detection_time = (self.avg_detection_time * (counter - 1) + detect_duration) / counter

        if time.time() - self.interval_start >= self.printer_interval:
            print(f"detection call {self.detection_counter}, detected {len(self.detected_objects)} average time (last {self.counter_cap}) {self.avg_detection_time}")
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
        
        self.rgb_sub = ImageSubscriber("rgb image", LOCAL_PLAYER + "kinect/rgb/image_raw", "bgr8")
        self.depth_sub = ImageSubscriber("depth image", LOCAL_PLAYER + "kinect/depth/image_raw", "32FC1")

        self.thresh_upper = TrackbarParameter(CANNY_THRESHOLD_UPPER, "upper", "kinect image")
        self.thresh_lower = TrackbarParameter(CANNY_THRESHOLD_LOWER, "lower", "kinect image")
        self.lateral_offset = TrackbarParameter(KINECT_OFFSET[0], "laser lateral offset", "laser rgb image", lambda x: int(100 * x) + 50, lambda x: (x - 50) / 100)
        self.height_offset = TrackbarParameter(KINECT_OFFSET[1], "laser height offset", "laser rgb image", lambda x: int(100 * x), lambda x: x / 100)
        self.depth_offset = TrackbarParameter(KINECT_OFFSET[2], "laser depth offset", "laser rgb image", lambda x: int(100 * x), lambda x: x / 100)

        img = empty_image(KINECT_DIMENSIONS)
        self.add_test_parameters(TestImage("depth_masked_image", img),
                                 TestImage("edges", img))
    
    def is_valid_data(self):
        return self.rgb_sub.is_valid() and self.depth_sub.is_valid()

    def detect_contours(self, base_class, rgb_image, depth_image):
        color_mask = imgops.mask_color(rgb_image, base_class.color)
        depth_masked_image = imgops.convert_gray2bgr(imgops.apply_mask(color_mask, depth_image))
        depth_masked_image = imgops.denoise(depth_masked_image, COLOR_MASK_SMOOTHING)

        edges = imgops.edges(depth_masked_image, self.thresh_lower.get_value(self.testmode), self.thresh_upper.get_value(self.testmode))
        edges = imgops.denoise(edges, CANNY_SMOOTHING, False)

        self.test_parameters["depth_masked_image"].set_value(depth_masked_image)
        self.test_parameters["edges"].set_value(edges)
            
        contours = imgops.get_contours(edges)

        if contours == ((), None):
            return None
        
        contours = imgops.get_inner_contours(*contours)
        cv2.drawContours(rgb_image, contours, -1, Color.YELLOW.default, 1)

        return contours

    def detect_field_objects(self, base_class):
        rgb_image = self.rgb_sub.get_image()
        depth_image = self.depth_sub.get_image()

        self.detected_objects.clear()
        self.screen.image = rgb_image

        contours = self.detect_contours(base_class, rgb_image, depth_image)

        if contours is None:
            return False

        rects = [cv2.boundingRect(c) for c in contours]

        def filter_cb(rect):
            x, y, w, h = rect
            result = check_range(w*h, *base_class.area_detect_range)
            result &= check_range(w/h, *base_class.ratio_detect_range)
            return result
        rects = filter_list(rects, filter_cb)

        field_objects = []

        for rect in rects:
            x, y, w, h = rect
            cx = min(int(x + w/2), depth_image.shape[1])
            cy = min(int(y + h/2), depth_image.shape[0])
            r = depth_image[cy, cx]
            if check_range(r, *KINECT_RANGE):
                fo = self.screen.create_field_object(rect, r, base_class)
                field_objects.append(fo)

        if base_class in [YellowGoal, BlueGoal, Robot] and len(field_objects) > 1:
            field_objects = [field_objects[0].merge(*field_objects[1:], return_type=base_class)]

        self.detected_objects.extend(field_objects)

        return True
    
class LaserScanDetector(FieldDetector):
    def __init__(self, testmode):
        super().__init__(self.detect_objects, Screen.LaserScreen("laser image"))
        self.laser_sub = LaserSubscriber("laser scan", LOCAL_PLAYER + "front_laser/scan")
        self.rgb_sub = ImageSubscriber("laser rgb image", LOCAL_PLAYER + "kinect/rgb/image_raw", "bgr8")

        self.testmode = testmode
        self.depth_offset = TrackbarParameter(LASER_OFFSET[0], "laser depth offset", "laser rgb image", lambda x: int(100 * x), lambda x: x / 100)
        self.lateral_offset = TrackbarParameter(LASER_OFFSET[1], "laser lateral offset", "laser rgb image", lambda x: int(100 * x) + 50, lambda x: (x - 50) / 100)
        self.height_offset = TrackbarParameter(LASER_OFFSET[2], "laser height offset", "laser rgb image", lambda x: int(100 * x), lambda x: x / 100)

        self.laser_screen_rgb = Screen.KinectScreen("laser rgb image")

    def is_valid_data(self):
        return self.laser_sub.is_valid()
    
    def detect_contours(self, laser_scan, laser_ranges):
        object_ranges = detect_contours(laser_scan, laser_ranges)

        def filter_cb(element):
            return abs(element[1] - element[0]) < 10
        object_contours = filter_list(object_ranges, filter_cb)

        return object_contours
       
    def detect_objects(self):
        laser_scan = self.laser_sub.get_scan()
        rgb_image = self.rgb_sub.get_image()

        laser_ranges = limit_ranges(laser_scan.ranges, *LASER_RANGE)
        laser_ranges = range_denoise(laser_ranges, 5)

        laser_image = imgops.laser_scan_to_image(laser_scan)
        self.screen.image = laser_image

        self.detected_objects.clear()

        object_ranges = self.detect_contours(laser_scan, laser_ranges)

        for min_phi, max_phi in object_ranges:
            center_phi = (max_phi + min_phi) / 2
            center_index = laser_index(center_phi, laser_scan)

            d = laser_ranges[center_index]

            laser_height = self.height_offset.get_value(self.testmode)
            laser_height = LASER_OFFSET[2]
            
            max_theta = laser_theta(laser_scan, center_phi, laser_height)
            min_theta = max_theta - atand(0.3 / d)

            if check_range(d, *LASER_RANGE):
                rect = self.screen.get_rect(min_phi, max_phi, min_theta, max_theta) 
                fo = self.screen.create_field_object(rect, d, GenericObject)
                self.add_result(fo)

        self.laser_screen_rgb.image = rgb_image

        return True


if __name__ == '__main__':
    rospy.init_node("object_detector_node")
    rospy.loginfo("Initialised ObjectDetector")

    args = rospy.myargv(argv=sys.argv)
    testmode = args[1] if len(args) > 1 else True
    detect_classes = [CLASSES[key.lower()] for key in args[2:]] if len(args) > 2 else list(CLASSES.values())

    kinect_det = KinectDetector(testmode)
    laser_det = LaserScanDetector(testmode)

    field_components_pub = rospy.Publisher("player/field_components", FieldComponents, queue_size=500)

    found_objects = []

    def run_kinect_detection():
        if kinect_det.is_valid_data():    
            found_objects.clear()

            for cls in detect_classes:
                kinect_det.detect(cls)
                found_objects.extend(kinect_det.detected_objects)
    
            for obj in found_objects:
                kinect_det.screen.draw_object(obj)

            kinect_det.screen.show_image()

            if testmode:    
                kinect_det.show_test_parameters()

            return found_objects
        else:
            rospy.loginfo("Waiting for images to process...")

    def run_laser_detection():       
        if laser_det.is_valid_data():
            found_objects.clear()

            laser_det.detect()
            found_objects.extend(laser_det.detected_objects)
              
            draw_laser_points(laser_det.laser_sub.get_scan(), laser_det.screen, laser_det.laser_screen_rgb)
            
            for obj in found_objects:
                laser_det.screen.draw_object(obj)
                laser_det.laser_screen_rgb.draw_object(obj)

            # laser_det.screen.image = imgops.scale(laser_det.screen.image, 2)

            laser_det.screen.show_image()
            laser_det.laser_screen_rgb.show_image()


            if testmode:
                laser_det.show_test_parameters()

            return found_objects
        else:
            rospy.loginfo("Waiting for laser scan to process...")

    def combine_detection():
        # objects = run_kinect_detection()
        objects = run_laser_detection()
        if objects is not None and len(objects) > 0:
            field_components_pub.publish(FieldComponents(list([o.get_field_component() for o in objects])))

    
    rospy.loginfo("Starting loop")
    ticker = CallbackTicker(TICK_RATE,
                            run_kinect_detection,
                            run_laser_detection,
                            # combine_detection
                            )
    
    imgticker = CVTicker(TICK_RATE)

    while not rospy.is_shutdown():
        ticker.tick()
        imgticker.tick()
