#!/usr/bin/env python
import cv2
import time
from typing import List
from field_components.field_components import FieldObject, Pole, YellowPuck, BluePuck, YellowGoal, BlueGoal, Robot, Field
from visualization.screen_components import Screen, TrackbarParameter, TestImage
import visualization.imgops as imgops
from globals.globals import *
import math_utils.math_function_utils as mf
import data_utils.topic_handlers as topics
from globals.tick import *
import data_utils.laser_scan_utils as laser
from data_utils.laser_scan_utils import LaserScanHandler
from data_utils.topic_handlers import FieldComponentsPublisher


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

def run_detection():
    fcp = FieldComponentsPublisher()
    kin_det: FieldDetector = KinectDetector()
    laser_det = LaserScanDetector()
    field = Field()
    fsc = Screen.FieldScreen("field", field)
    kin_det.detect()
    laser_det.detect()
    kin_objects = kin_det.detected_objects
    laser_objects = laser_det.detected_objects

    merged_objects = []
    found_laser_objects = []
    for kin_obj in kin_objects:
        for laser_obj in laser_objects:
            if kin_obj.distance.distance_xy(laser_obj.distance) < 0.1:
                kin_obj.distance = laser_obj.distance
                found_laser_objects.append(laser_obj)
                break
        merged_objects.append(kin_obj)

    for laser_obj in laser_objects:
        if laser_obj not in found_laser_objects:
            merged_objects.append(laser_obj)

    fcp.publish(merged_objects)
    field.update()
    field.draw(fsc, False, False, True, False, False)

if __name__ =="__main__":
    tick = CallbackTicker(TICK_RATE,
                          run_detection,
                          lambda: cv2.waitKey(10))
    run_detection()