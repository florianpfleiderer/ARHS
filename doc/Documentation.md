# Robohockey Documentation
**Authors:**</br>
Heinrich Fuhrmann 11940304 </br>
Thomas Schwabe 11918466</br>
Florian Pfleiderer 11771070</br>

**Version:** 
2023-04-14

## Milestone 2: Field Component Detection

### Field Component Detection Algorithm
1. Copy newest kinect image data to work on*(copy_sensordata)*
2. Create a mask based on the desired color *(color_mask)*</br>
   First the denoised image is converted to hsv colorspace. Then a binary mask is created by setting every pixel that is in the desired color range to *1* and everything else to *0*. Afterwards small black regions inside the mask get opened up and small white regions outside the mask get closed.
3. Combine the color mask with the depth image to create a depth mask *(depth_mask_gs)*</br>
   The combination with the depth image is needed to handle occlusion.
   A completely green image of the same size as the kinect image is created. The color mask gets cut out of this and the now missing parts are replace with the depth image. This approach with the green image is needed because otherwise objects that are to close or to fare away get lost in the normal black image.
4. Detect edges of the depth mask *(edge_detection)*</br>
   The edges of the depth mask are getting detected with canny edge detection and then closed up to detect the contours better in the next step.
5. Convert the detected edges into contours *(get_contours)*</br>
   To get the desired contours we used the *findContours* method of openCV on the detected contours. Because of possible occlusion only the inner contours are getting used. In case of occlusion the outer contour would be the one around the object in front and the one behind it together.
6. Decide which objects are detected *(detect_object)*</br>
   This method combines all methodes above starting with *color_mask()*. After the contours are detected the area and with/height-ratio of each is calculated based on the bounding rectangle. If the area is not to small and the ratio does fit the pre defined ratio of the desired object we know, in combination with the color, what type of field component it is. The distance to the object is measured to the centerpixel of the rectangle with the kinect and the angle to the object is also calculated to the centerpixel. If a robot or a goal of the same color was detected multiple times those objects get combined. This is the case when for example a puk is in front of the goal then it gets split into two parts. For the combining process the bounding rectangle of all those multiple detected objects are getting combined and the average of the farthest and nearest distance is taken as the new distance. The angle is calculated again to the new centerpixel of the combined rectangle.
7. All those steps are executed for every kind of field component and then the result is displayed on the screen by drawing a rectanlge around the detected objects with the object color, type and distance above.

### Field Component Detection Testmode and Color Calibration
For debuging and some calibration a **testmode** was implemented in *object_detector* which can be accessed by adding some arguments to the commandline when starting the node. The access the testmode you have to write following:</br>
 ```rosrun player object_detecor True <color1(optional)> <color2(optional)> ...```</br>
*True* activates the testmode. It adds trackbars for upper/lower threshold of canny edge detection and also displays the depth image of the kinect. By adding colors like *yellow*, *red* and so on after *True* every step of the image processing of the desired color is displayed.</br>
</br>
To help with **color calibration** a extra node *color_calibrator* was created. It helps to calibrate the hsv values by showing the denoised kinect image to the user underneath which some trackbars are placed. Those trackbars control the upper and lower hsv boundaries which define the resulting shown image. Also the hsv value inside the circle on the second windows is printed on the console.</br>


### State Machine

### Servers

## Issues
#### Algorithm:
The **first approach** of the detection algorithm was based on **template matching**. The idea was to take photos of every field component and use those to compare the preprocessed image from the kinect to. For preprocessing we used a binary color mask and combined it with the original image to get only the desired regions of the image. The problem there was that rotation and perspective in general of the object messed up this algorithm.</br>
To **solve the problems** we created contours of the template pictures when initialising the programm and then compared those to the detected contours in the image with **contour matching**. This approach worked fine for rotationally symmetrical objects (pole, puk) but not reliable enough for the others. Also the fact that the detection was based on previously taken pictures wasn't the best.</br>
The **third approach** is the one that ended up **working reliable** enough and it is the one used in our code.
