#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import copy
import sys



#CONSTANTS
KINECT_FOV = 62
AREA_MIN = 400
AREA_YELLOW = 800
CANNY_THRESHOLD_UPPER = 25
CANNY_THRESHOLD_LOWER = 50

OBJECTS = [('robot', 'red'),
           ('pole', 'green'),
           ('puk', 'blue'),
           ('puk', 'yellow'),
           ('goal', 'blue'),
           ('goal', 'yellow')]

#color_min, color_max
COLORS = {'green': ([55, 50, 50], [65, 255, 255]),
          'blue': ([115, 50, 50], [125, 255, 255]),
          'yellow': ([25, 50, 50], [35, 255, 255]),
          'red':([0, 50, 50], [5, 255, 255])}

#ratio_min, ratio_max
RATIOS = {'pole': [None, 0.4],
          'puk': [None, 0.6],
          'goal': [1.7, None],
          'robot': [None, None]}


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
        self.depth_img = cv2.normalize(src=self.depth_raw, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    
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
        kernel = np.ones((5,5),np.uint8)
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
        kernel = np.ones((5,5),np.uint8)
        canny = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)
        return canny
    
    def get_contours(self, img, object=None):
        #TODO: there may be problems with getting the innner contours in real life
        
        good_contours = []
        contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        try:
            #for robot get biggest contour
            if object == 'robot':   
                good_contours.append(contours[0])
            else:
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
            x,y,w,h = cv2.boundingRect(contour)
            area = w*h
            ratio = w/h      
            if area < area_min:
                continue
            if ratio < ratio_min or ratio > ratio_max:
                continue  
            
            cx, cy = self.get_center(x, y, w, h)
            distance = self.get_distance(cx, cy)
            angle = self.get_direction(x)
            
            found_objects.append({'color': color,
                                  'type': type,
                                  'distance': distance,
                                  'angle': angle,
                                  'x': x,
                                  'y': y,
                                  'w': w,
                                  'h': h})
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
        x_min_list = []
        y_min_list = []
        x_max_list = []
        y_max_list = []
        distances = []
        
        for object in objects:
            x_min_list.append(object['x'])
            y_min_list.append(object['y'])
            x_max_list.append(object['x'] + object['w'])
            y_max_list.append(object['y'] + object['h'])
            distances.append(object['distance'])

        x_min = min(x_min_list)
        y_min = min(y_min_list)
        x_max = max(x_max_list)
        y_max = max(y_max_list)
        
        h = y_max - y_min
        w = x_max - x_min
        
        distance = (min(distances) + max(distances))/2
        
        cx, cy = self.get_center(x_min, y_min, w, h)
        angle = self.get_direction(cx)
        
        object = {'color': objects[0]['color'],
                  'type': objects[0]['type'],
                  'distance': distance,
                  'angle': angle,
                  'x': x_min,
                  'y': y_min,
                  'w': w,
                  'h': h}
        return object

    def get_center(self, x, y, w, h):
        cx = int(x + w/2)
        cy = int(y + h/2)
        return cx, cy
    
    def get_direction(self, x):
        h, w, _ = self.rgb_img.shape
        angle = 180 - KINECT_FOV/2 + (KINECT_FOV*x)/w
        return angle
    
    def get_distance(self, x, y):
        dist = self.depth_raw[y, x]   
        return dist
      
    def draw_rectangle(self, x, y, w, h):
        cv2.rectangle(self.rgb_img,(x,y),(x+w,y+h),(0 ,255, 255), 1)
    
    def draw_center(self, x, y, w, h):
        cx, cy = self.get_center(x, y, w, h)
        cv2.circle(self.rgb_img,(cx,cy), 2, (50, 125,255), -1)
    
    def draw_text(self, x, y, text):
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.35
        color = (0 ,255, 255)
        thickness = 1
        
        cv2.putText(self.rgb_img, text, (x, y-10), font, fontScale, color, thickness, cv2.LINE_AA)
    
    def visualize(self, objects):
        for object in objects:
            color = object['color']
            type = object['type']
            x = object['x']
            y = object['y']
            w = object['w']
            h = object['h']
            
            if type == 'pole':
                text = f"{object['type']} {object['distance']:.1f}m"
            else:
                text = f"{object['color']} {object['type']} {object['distance']:.1f}m"
            self.draw_rectangle(x, y, w, h)
            self.draw_center(x, y, w, h)
            self.draw_text(x, y, text)

    def test_function(self, color, mask_color, mask_depth, edges, contours):
        for testcolor in self.testcolor:
            if testcolor == color:
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
