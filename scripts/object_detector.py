#!/usr/bin/env python
import rospy

import cv2
import numpy as np
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

OBJECTS = [('robot', 'red'),
           ('pole', 'green'),
           ('puck', 'blue'),
           ('puck', 'yellow'),
           ('goal', 'blue'),
           ('goal', 'yellow')]

#color_min, color_max
COLORS = {'green': ([55, 50, 50], [65, 255, 255]),
          'blue': ([115, 50, 50], [125, 255, 255]),
          'yellow': ([25, 50, 50], [35, 255, 255]),
          'red':([0, 50, 50], [5, 255, 255])}

#ratio_min, ratio_max
RATIOS = {'pole': [None, 0.4],
          'puck': [None, 0.6],
          'goal': [1.7, None],
          'robot': [None, None]}

class KinectDetector:
    def __init__(self):
        # #newest data
        # self.new_rgb_img = None
        # self.new_depth_raw = None
        
        # #copied data to work on
        # self.rgb_img = None
        # self.depth_raw = None
        # self.depth_img = None
        
        # #testmode
        # self.testmode = False
        # self.testcolor = []
        
        try:
            args = rospy.myargv(argv=sys.argv)
            if len(args) > 1:
                self.testmode = args[1]
            if len(args) > 2:
                self.testcolor = args[2:]
        except:
            rospy.loginfo('wrong args')
                
        # if self.testmode:
        #     self.init_trackbars()

        self.testmode = True
        
        self.rgb_sub = ImageSubscriber("rgb image", "robot1/kinect/rgb/image_raw", "bgr8")
        self.depth_sub = ImageSubscriber("depth image", "robot1/kinect/depth/image_raw", "32FC1")
    
    def is_valid_data(self):
        return self.rgb_sub.is_valid() and self.depth_sub.is_valid()

    # def copy_sensordata(self):
    #     self.rgb_img = self.rgb_sub.get_image()
    #     self.depth_raw = self.depth_sub.get_image()

    #     depth_norm = self.depth_raw / KINECT_MAX_RANGE
    #     depth_map = cv2.applyColorMap(np.uint8((depth_norm * 255)), cv2.COLORMAP_JET)
    #     self.depth_img = cv2.cvtColor(depth_map, cv2.COLOR_BGR2GRAY)
    #     # self.depth_img = cv2.normalize(src=self.depth_raw, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    
    # def show_imgs(self, scale=None):
    #     if scale is not None:
    #         self.rgb_img = cv2.resize(self.rgb_img, (0, 0), fx = scale, fy = scale, interpolation = cv2.INTER_BITS2)
    #     cv2.imshow('Object detector', self.rgb_img)
    #     if self.testmode:
    #         cv2.imshow('depth_raw', self.depth_raw)
    #         cv2.imshow('depth_img', self.depth_img)
    #     cv2.waitKey(10)
   
    # def color_mask(self, color):
    #     #filter color
    #     color_min, color_max = COLORS[color]
    #     hsv =  cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2HSV)
    #     mask = cv2.inRange(hsv, np.array([[color_min]]), np.array([[color_max]]))
    #     #reduce noise
    #     kernel = np.ones((COLOR_MASK_SMOOTHING, COLOR_MASK_SMOOTHING),np.uint8)
    #     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    #     mask =  cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    #     return mask
    
    # def depth_mask_gs(self, mask_color):
    #     #create greenscreen
    #     gs = np.empty_like(self.rgb_img)
    #     gs[:] = (0, 255, 0)
    #     #subtract color mask
    #     mask_color_invert = cv2.bitwise_not(mask_color)
    #     gs = cv2.bitwise_and(gs, gs, mask=mask_color_invert)
    #     #create depth mask
    #     mask_depth = cv2.bitwise_and(self.depth_img, self.depth_img, mask=mask_color)
    #     mask_depth = cv2.cvtColor(mask_depth, cv2.COLOR_GRAY2BGR)
    #     #add gs and depth mask
    #     mask_depth = cv2.add(gs, mask_depth)
    #     return mask_depth
    
    # def edge_detection(self, img):
    #     #TODO: maybe need to blur bevor canny in real life
    #     thresh_lower = CANNY_THRESHOLD_LOWER
    #     thresh_upper = CANNY_THRESHOLD_UPPER
    #     if self.testmode:
    #         thresh_upper = cv2.getTrackbarPos('upper', 'Object detector')  
    #         thresh_lower = cv2.getTrackbarPos('lower', 'Object detector')
    #     canny =  cv2.Canny(img, thresh_lower, thresh_upper)
    #     #reduce noise
    #     kernel = np.ones((CANNY_SMOOTHING, CANNY_SMOOTHING),np.uint8)
    #     canny = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)
    #     return canny
    
    # def get_contours(self, img, object=None):
        #TODO: there may be problems with getting the innner contours in real life
        
        # good_contours = []
        # contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        # try:
        #     #get inner contours
        #     hierarchy = hierarchy[0]
        #     for component in zip(contours, hierarchy):
        #         currentContour = component[0]
        #         currentHierarchy = component[1]
        #         if currentHierarchy[2] < 0: #if there is no child contour
        #             good_contours.append(currentContour)
        # except:
        #     pass
        # return good_contours

    def detect_field_objects(self, base_class):
        detection_start_time = time.time()

        rgb_image = self.rgb_sub.get_image()
        depth_image = self.depth_sub.get_image()

        rgb_image = imgops.denoise(rgb_image, COLOR_MASK_SMOOTHING)

        color_mask = imgops.mask_color(rgb_image, base_class.color)
        depth_masked = imgops.apply_mask(color_mask, depth_image)
        depth_masked_image = imgops.convert_gray2bgr(depth_masked)

        rgb_image = imgops.denoise(depth_masked_image, CANNY_SMOOTHING)

        thresh_upper = TrackbarParameter(CANNY_THRESHOLD_UPPER, "upper", "rgb image")
        thresh_lower = TrackbarParameter(CANNY_THRESHOLD_LOWER, "lower", "rgb image")

        edges = imgops.edges(depth_masked_image, thresh_lower.get_value(self.testmode), thresh_upper.get_value(self.testmode))

        contours = imgops.get_contours(edges)

        if contours == ((), None):
            return []
        
        contours = imgops.get_inner_contours(*contours)

        # show images if testmode is enabled
        if self.testmode:
            cv2.drawContours(rgb_image, contours, -1, Color.YELLOW.default, 1)
            cv2.imshow('depth_masked', depth_masked_image)
            cv2.imshow('edges', edges)
            cv2.imshow('contours', rgb_image)
            cv2.waitKey(10)

        def filter_cb(contour):
            x, y, w, h = cv2.boundingRect(contour)
            result = check_range(w*h, *base_class.area_detect_range)
            result &= check_range(w/h, *base_class.ratio_detect_range)

            return result

        contours = filter_list(contours, filter_cb)

        screen_objects = [ScreenObject(*cv2.boundingRect(contour)) for contour in contours]

        if type(base_class) in [YellowGoal, BlueGoal, Robot] and len(screen_objects) > 1:
            screen_objects = [screen_objects[0].merge(screen_objects[1:])]

        distances = [screen_obj.get_field_vector(depth_image) for screen_obj in screen_objects]

        field_objects = [base_class(distance, screen_obj) for distance, screen_obj in zip(distances, screen_objects)]

        return field_objects

    # def detect_object(self, type, color):        
    #     mask_color = self.color_mask(color)
    #     mask_depth = self.depth_mask_gs(mask_color)
    #     canny = self.edge_detection(mask_depth)
    #     contours = self.get_contours(canny, type)
        
    #     #for testmode
    #     if self.testmode:
    #         self.test_function(color, mask_color, mask_depth, canny, contours)
        
    #     area_min = AREA_MIN
    #     if color == 'yellow':   #other min area because of yellow parts of the robot
    #         area_min = AREA_YELLOW
        
    #     ratio_min, ratio_max = RATIOS[type]
    #     if ratio_min is None:
    #         ratio_min = 0
    #     if ratio_max is None:
    #         ratio_max = 999999
        
    #     found_objects = []
    #     for contour in contours:
    #         screen_obj = ScreenObject.from_tuple(cv2.boundingRect(contour))

    #         if screen_obj.get_area() < area_min:
    #             continue
    #         if not ratio_min <= screen_obj.get_ratio() <= ratio_max:
    #             continue
            
    #         distance = PolarVector2(screen_obj.get_field_distance(self.depth_raw), screen_obj.get_field_angle(self.depth_raw))
            
    #         field_object_properties = FieldComponent(color, type, distance, screen_obj)

    #         found_objects.append(FieldObject(field_object_properties))

    #     #for objects that exist only one time on the field -> combine seperatet detections
    #     if (type == 'goal' or type == 'robot') and len(found_objects) > 1:
    #         found_objects = [self.combine_objects(found_objects)]
        
    #     return found_objects
        
    # def detect_multiple_objects(self, objects):
    #     found_objects = []
    #     for object in objects:
    #         type, color = object
    #         found_objects = found_objects + self.detect_object(type, color)
    #     return found_objects

    # def combine_objects(self, objects):     
        screen_objects = [o.screen_obj for o in objects]
        x_min = min(o.x for o in screen_objects)
        y_min = min(o.y for o in screen_objects)
        x_max = max(o.x + o.w for o in screen_objects)
        y_max = max(o.y + o.h for o in screen_objects)
        
        h = y_max - y_min
        w = x_max - x_min
        
        new_screen_object = ScreenObject((x_min, y_min, w, h))

        distances = [o.get_field_distance(self.depth_raw) for o in screen_objects]
        distance = (min(distances) + max(distances))/2
        
        angle = new_screen_object.get_field_angle(self.depth_raw)
        
        field_object_properties = FieldComponent(objects[0].color_name, objects[0].type, PolarVector2(distance, angle), new_screen_object.properties)

        return FieldObject(field_object_properties)

    def test_function(self, color, mask_color, mask_depth, edges, contours):
        if color in self.testcolor:
            cv2.imshow(f'{color} mask', mask_color)
            cv2.imshow(f'{color} depth_mask', mask_depth)
            cv2.imshow(f'{color} edges', edges)
            contour_test = self.rgb_img.copy()
            cv2.drawContours(contour_test, contours, -1, (0, 255, 0), 1)
            cv2.imshow(f'{color} contours', contour_test)
                
    def init_trackbars(self):
        def nothing(x):
            pass
        cv2.namedWindow('Object detector')
        cv2.createTrackbar('upper', 'Object detector', CANNY_THRESHOLD_UPPER, 255, nothing)
        cv2.createTrackbar('lower', 'Object detector', CANNY_THRESHOLD_LOWER, 255, nothing)

class LaserScanDetector:
    def __init__(self):
        self.laser_sub = LaserSubscriber("laser scan", "robot1/front_laser/scan")
        self.laser_scan = None

        self.rgb_sub = ImageSubscriber("rgb image", "robot1/kinect/rgb/image_raw", "bgr8")
        self.rgb_image = None

    def is_valid_data(self):
        return self.laser_sub.is_valid() and self.rgb_sub.is_valid()
       
    def detect_objects(self):
        new_object = True
        min_theta = 0
        max_theta = 0

        previous_range = 0
        previous_theta = -180

        self.rgb_image = self.rgb_sub.get_image()
        self.laser_scan = self.laser_sub.get_scan()

        detected_objects = []

        for index, range in enumerate(self.laser_scan.ranges):
            current_theta = index * self.laser_scan.angle_increment

            if not LASER_MIN_ANGLE <= current_theta <= LASER_MAX_ANGLE:
                continue

            # if not self.laser_scan.range_min <= range <= self.laser_scan.range_max:
            #     continue

            if abs(previous_range - range) > LASER_EDGE_THRESHOLD:
                if new_object:
                    new_object = False
                    min_index = index
                    min_theta = current_theta

                else:
                    max_index = previous_index
                    max_theta = previous_theta

                    center_theta = (max_theta - min_theta) / 2
                    center_index = int((max_index - min_index) / 2)

                    imh = self.rgb_image.shape[0]
                    imw = self.rgb_image.shape[1]

                    d = self.laser_scan.ranges[min_index]
                    min_alpha = atand((KINECT_HEIGHT - LASER_HEIGHT - 0.2) / d)

                    d = self.laser_scan.ranges[max_index]
                    max_alpha = atand((KINECT_HEIGHT - LASER_HEIGHT - 0.2) / d)

                    d = self.laser_scan.ranges[center_index]

                    new_distance = PolarVector2(d, center_theta)
                    screen_obj = ScreenObject.from_angles(min_theta, min_alpha + KINECT_ANGLE, max_theta, max_alpha + KINECT_ANGLE, imw, imh, KINECT_FOV_X, KINECT_FOV_Y)
                    detected_objects.append(FieldObject(FieldComponent("?", "?", new_distance, screen_obj.properties)))

                    new_object = True

            previous_index = index
            previous_theta = current_theta
            previous_range = range

        return detected_objects
    
    # def visualize(self, field_objects):
    #     cv2.imshow("Laser Scan", self.rgb_image)

    #     for field_object in field_objects:
    #         field_object.draw(self.rgb_image)



if __name__ == '__main__':
    rospy.init_node("object_detector_node")
    rospy.loginfo("Initialised ObjectDetector")

    kinect_det = KinectDetector()
    laser_det = LaserScanDetector()

    loop_rate = rospy.Rate(10)

    rospy.loginfo("Starting loop")
    while not rospy.is_shutdown():
        if kinect_det.is_valid_data():          
            kinect_det.rgb_sub.show_image() 
            found_objects = kinect_det.detect_field_objects(Pole)

            kinect_det.rgb_sub.draw_objects(found_objects)     
            kinect_det.rgb_sub.show_image()
            # kinect_det.show_imgs()
        else:
            rospy.loginfo("Waiting for images to process...")

        # if laser_det.is_valid_data():
        #     found_objects = laser_det.detect_objects()
        #     print(laser_det.rgb_image.shape)

        #     laser_det.rgb_sub.show_image()

        #     laser_vis.image = laser_det.rgb_image
        #     laser_vis.show_image()
        #     laser_vis.draw_objects(found_objects)
        # else:
        #     rospy.loginfo("Waiting for images to process...")
        
        loop_rate.sleep()
