#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import copy
import sys
import math

from player.msg import FieldComponent, PolarVector2, ScreenPosition
from enum import Enum

#CONSTANTS
KINECT_FOV = 62
KINECT_TAN = math.tan(KINECT_FOV/2 * math.pi / 180)
KINECT_MAX_RANGE = 5.0

AREA_MIN = 800
AREA_YELLOW = 800
CANNY_THRESHOLD_UPPER = 40
CANNY_THRESHOLD_LOWER = 40
CANNY_SMOOTHING = 5
COLOR_MASK_SMOOTHING = 5

CV2_DEFAULT_FONT = cv2.FONT_HERSHEY_SIMPLEX
CV2_DEFAULT_FONT_SCALE = 0.25
CV2_DEFAULT_THICKNESS = 1


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

# enum for colors with default, min and max values
class Color(Enum):
    RED = ((0, 0, 255), (0, 50, 50), (5, 255, 255))
    GREEN = ((0, 255, 0), (55, 50, 50), (65, 255, 255))
    BLUE = ((255, 0, 0), (115, 50, 50), (125, 255, 255))
    YELLOW = ((0, 255, 255), (25, 50, 50), (35, 255, 255))
    ORANGE = ((0, 165, 255), (10, 50, 50), (20, 255, 255))

    def __init__(self, default, min, max):
        self.default = default
        self.min = min
        self.max = max

    def default(self):
        return self.default
    
    def min(self):
        return self.min

    def max(self):
        return self.max

    def get_range(self):
        return self.min, self.max

    def __str__(self) -> str:
        return self.name.lower()

class ScreenObject:
    def __init__(self, properties):
        if type(properties) is tuple:
            self.x, self.y, self.w, self.h = properties
            self.properties = ScreenPosition(x=self.x, y=self.y, w=self.w, h=self.h)

        else:
            self.properties = properties
            self.x = properties.x
            self.y = properties.y
            self.w = properties.w
            self.h = properties.h

    def get_center(self):
        cx = int(self.x + self.w/2)
        cy = int(self.y + self.h/2)
        return cx, cy

    def get_corner_points(self):
        return ((self.x, self.y), (self.x + self.w, self.y + self.h))
    
    def get_area(self):
        return self.w * self.h

    def get_ratio(self):
        return self.w / self.h
    
    def get_field_distance(self, depth_img):
        cx, cy = self.get_center()
        return depth_img[cy, cx]
    
    def get_field_angle(self, depth_img):
        cx = self.get_center()[0]
        w = depth_img.shape[1]
        return math.atan((1 - 2 * cx / w) * KINECT_TAN) * 180 / math.pi

    def draw_bounds(self, window):
        corners = self.get_corner_points()
        cv2.rectangle(window, corners[0], corners[1], Color.YELLOW.default, CV2_DEFAULT_THICKNESS)

    def draw_center(self, window):
        cv2.circle(window, self.get_center(), 2, Color.ORANGE.default, -CV2_DEFAULT_THICKNESS)

    def draw(self, window):
        self.draw_bounds(window)
        self.draw_center(window)

    def __str__(self) -> str:
        return f"({self.x}, {self.y}) {self.w}x{self.h}"

class FieldObject:
    def __init__(self, properties):
        if type(properties) is tuple:
            self.color_name, self.type, self.distance, self.screen_pos = properties

        else: 
            self.color_name = properties.color_name
            self.type = properties.type
            self.distance = properties.player_distance
            self.screen_pos = properties.screen_position

        self.screen_obj = ScreenObject(properties.screen_position)
    
    def draw_text(self, window):
        cv2.putText(window, str(self), (self.screen_pos.x, self.screen_pos.y-10),
                    CV2_DEFAULT_FONT, CV2_DEFAULT_FONT_SCALE,
                    Color.YELLOW.default, CV2_DEFAULT_THICKNESS, cv2.LINE_AA)

    def draw(self, window):
        self.screen_obj.draw(window)
        self.draw_text(window)

    def __str__(self) -> str:
        return f"{self.color_name} {self.type} {self.distance.r:.2f}m {self.distance.theta:.1f}d"

class ObjectDetector:
    def __init__(self):
        rospy.init_node("object_detector_node")
        rospy.loginfo("Initialised ObjectDetector")

        #newest data
        self.new_rgb_img = None
        self.new_depth_raw = None
        
        #copied data to work on
        self.rgb_img = None
        self.depth_raw = None
        self.depth_img = None
        
        #testmode
        self.testmode = False
        self.testcolor = []
        
        try:
            args = rospy.myargv(argv=sys.argv)
            if len(args) > 1:
                self.testmode = args[1]
            if len(args) > 2:
                self.testcolor = args[2:]
        except:
            rospy.loginfo('wrong args')
                
        if self.testmode:
            self.init_trackbars()
        
        #misc
        self.bridge = CvBridge()
        
        #subscriber
        self.new_rgb_img_sub = rospy.Subscriber("robot1/kinect/rgb/image_raw", Image, self.rgb_camera_cb)
        self.new_depth_raw_sub = rospy.Subscriber("robot1/kinect/depth/image_raw", Image, self.depth_camera_cb)
 
    def rgb_camera_cb(self, msg):
        try:
            self.new_rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
    
    def depth_camera_cb(self, msg):
        try:
            self.new_depth_raw = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
    
    def copy_sensordata(self):
        self.rgb_img = copy.deepcopy(self.new_rgb_img)
        self.depth_raw = copy.deepcopy(self.new_depth_raw)

        depth_norm = self.depth_raw / KINECT_MAX_RANGE
        depth_map = cv2.applyColorMap(np.uint8((depth_norm * 255)), cv2.COLORMAP_JET)
        self.depth_img = cv2.cvtColor(depth_map, cv2.COLOR_BGR2GRAY)
        # self.depth_img = cv2.normalize(src=self.depth_raw, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    
    def show_imgs(self, scale=None):
        if scale is not None:
            self.rgb_img = cv2.resize(self.rgb_img, (0, 0), fx = scale, fy = scale, interpolation = cv2.INTER_BITS2)
        cv2.imshow('Object detector', self.rgb_img)
        if self.testmode:
            cv2.imshow('depth_raw', self.depth_raw)
            cv2.imshow('depth_img', self.depth_img)
        cv2.waitKey(10)
   
    def color_mask(self, color):
        #filter color
        color_min, color_max = COLORS[color]
        hsv =  cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([[color_min]]), np.array([[color_max]]))
        #reduce noise
        kernel = np.ones((COLOR_MASK_SMOOTHING, COLOR_MASK_SMOOTHING),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask =  cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask
    
    def depth_mask_gs(self, mask_color):
        #create greenscreen
        gs = np.empty_like(self.rgb_img)
        gs[:] = (0, 255, 0)
        #subtract color mask
        mask_color_invert = cv2.bitwise_not(mask_color)
        gs = cv2.bitwise_and(gs, gs, mask=mask_color_invert)
        #create depth mask
        mask_depth = cv2.bitwise_and(self.depth_img, self.depth_img, mask=mask_color)
        mask_depth = cv2.cvtColor(mask_depth, cv2.COLOR_GRAY2BGR)
        #add gs and depth mask
        mask_depth = cv2.add(gs, mask_depth)
        return mask_depth
    
    def edge_detection(self, img):
        #TODO: maybe need to blur bevor canny in real life
        thresh_lower = CANNY_THRESHOLD_LOWER
        thresh_upper = CANNY_THRESHOLD_UPPER
        if self.testmode:
            thresh_upper = cv2.getTrackbarPos('upper', 'Object detector')  
            thresh_lower = cv2.getTrackbarPos('lower', 'Object detector')
        canny =  cv2.Canny(img, thresh_lower, thresh_upper)
        #reduce noise
        kernel = np.ones((CANNY_SMOOTHING, CANNY_SMOOTHING),np.uint8)
        canny = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)
        return canny
    
    def get_contours(self, img, object=None):
        #TODO: there may be problems with getting the innner contours in real life
        
        good_contours = []
        contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        try:
            #get inner contours
            hierarchy = hierarchy[0]
            for component in zip(contours, hierarchy):
                currentContour = component[0]
                currentHierarchy = component[1]
                if currentHierarchy[2] < 0: #if there is no child contour
                    good_contours.append(currentContour)
        except:
            pass
        return good_contours
                
    def detect_object(self, type, color):        
        mask_color = self.color_mask(color)
        mask_depth = self.depth_mask_gs(mask_color)
        canny = self.edge_detection(mask_depth)
        contours = self.get_contours(canny, type)
        
        #for testmode
        if self.testmode:
            self.test_function(color, mask_color, mask_depth, canny, contours)
        
        area_min = AREA_MIN
        if color == 'yellow':   #other min area because of yellow parts of the robot
            area_min = AREA_YELLOW
        
        ratio_min, ratio_max = RATIOS[type]
        if ratio_min is None:
            ratio_min = 0
        if ratio_max is None:
            ratio_max = 999999
        
        found_objects = []
        for contour in contours:
            screen_obj = ScreenObject(cv2.boundingRect(contour))

            if screen_obj.get_area() < area_min:
                continue
            if not ratio_min <= screen_obj.get_ratio() <= ratio_max:
                continue
            
            distance = PolarVector2(screen_obj.get_field_distance(self.depth_raw), screen_obj.get_field_angle(self.depth_raw))
            
            field_object_properties = FieldComponent(color, type, distance, screen_obj)

            found_objects.append(FieldObject(field_object_properties))

        #for objects that exist only one time on the field -> combine seperatet detections
        if (type == 'goal' or type == 'robot') and len(found_objects) > 1:
            found_objects = [self.combine_objects(found_objects)]
        
        return found_objects
        
    def detect_multiple_objects(self, objects):
        found_objects = []
        for object in objects:
            type, color = object
            found_objects = found_objects + self.detect_object(type, color)
        return found_objects

    def combine_objects(self, objects):     
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
      
    def visualize(self, field_objects):
        for field_object in field_objects:
            field_object.draw(self.rgb_img)

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
        
if __name__ == '__main__':
    od = ObjectDetector()
    loop_rate = rospy.Rate(10)

    rospy.loginfo("Starting loop")
    while not rospy.is_shutdown():
        if od.new_rgb_img is not None and od.new_depth_raw is not None:
            od.copy_sensordata()              
            found_objects = od.detect_multiple_objects(OBJECTS)
            od.visualize(found_objects)
            od.show_imgs()
        else:
            rospy.loginfo("Waiting for images to process...")
        
        loop_rate.sleep()
