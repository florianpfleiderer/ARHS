# Robohockey Documentation M3
**Authors:**</br>
Heinrich Fuhrmann 11940304 </br>
Thomas Schwabe 11918466</br>
Florian Pfleiderer 11771070</br>

**Version:** 
2023-05-12

## Milestone 3: Field Dimension Detection and Localisation
### Localisation Algorithm
The Algorithm used for localisation is based on the specific field setup, which is a known environment.
It s based on the fact, that if you know the distance between two poles and divide by the distance of two other poles, the proportions tell you
which poles they are and therefore you can calculate your own position on the field.
Another important aspect for initial localisation is that the robot always starts in the red area, so you can detect the outer most pole, which also sets the origin of the coordinate system for localisation.
Once the three left or right outer most Poles are found, by checking the ratios between them and decide if you look left or right, the LaserScan data is accessed in the index range around where the object should be and this slice is searched for distances near the values from the depth camera. This ensures, that the Poles are found if the calculated rgb angle is slightly off(see issue3).
The next step would be to turn the robot left until three poles with the right ratios are found. This would result in a "check_poles" function returning True and thus, stopping the Robot motion (see issue5). Then the method from the paragraph above can be used.
The next Plan was to use a localisation method similar to rf2o (http://wiki.ros.org/rf2o_laser_odometry), but there was simply not enough time for our team to implement this.


### Object model
The FieldObject class is the centrepiece around which all of the detection and behaviour algorithms are built.
It has the following attributes:
        *color*: color of the object - the enum 'Color' stores default color value as well as min and max HSV values for color detection
        *type*: type of the object - string for representation
        *spherical_distance*: distance of the object in spherical coordinates in relation to the robot - directly converted from scan data
        *half_size*: half size of the object in spherical coordinates - spherical_distance + half_size would yield the bottom left 
            corner of the object in the FOV of the robot.
        *area_detect_range*: range of the area of the object in square pixels
        *ratio_detect_range*: range of the ratio of the object in pixels(w)/pixels(h)
        *position*: absolute position of the object in the field in x, y coordinates 

The FieldObject class has children that directly represent our known objects: YellowPuck, BluePuck, Pole, YellowGoal, BlueGoal, Robot, and a GenericObject class as well as a LaserPoint class for visualization of the raw laser_scan data. 
These attributes are stored in a FieldComponent message and published to the topic "player/field_components".
Any node that subscribes to the "player/field_components" topic should reconstruct the FieldObject from the given data.

### Issues 
**Issue1**
Spherical coordinates are in ISO 80000-2:2019 convention (used in physics, theta is angle from vertical z axis) and algorithm used mathematical convention.

**Issue2**
Calculated ratios between Pole distances where off by more than 10 Percent, so there was a need to implement another Ratio for better proof, which poles were found. Also with using different poles and taking medians, the calculations where off by quite a margin, so we decided to use the Laser Scan Data. 

**Issue3**
The angle calculated by the rgb image was slightly of to the real angle, partly because of the different camera position to the Lidar but also because of a small lens effect if the poles where on the edges of the image. To overcome this issue, it was planned to use a tf listener and transform the angle. As we had not enough time to implement this, we switched to a quick fix: slice some indices plus/minus the calculated camera angle of the LaserScan Data and search for similar depth ranges than the kinect shows. With this, we could locate the Poles and calculate our initial position.

**Issue4**
The Object Detection Algorithm based on Calibrated Colors did not work as in the previous Milestone, maybe because the sunlight even affects the colors slightly if the shades are closed. After recalculating, the real object detection worked again.

**Issue5**
Detecting the pole ratios when turning the robot. We did not have sufficient time to solve our issue with slow computing. The framerate still sometimes drops to values around 1fps, which sometimes makes the robot miss a pole completely and therefore not recognising the three searched poles. This party lies in the denoising of the RGB image before detection. The Problem here is that without denoising, the detection success rate drops significantly. Maybe we can find a different function or use a more efficient c++ function in a python wrapper.

### Contributions
- Heinrich Fuhrmann
    - vector calculations
    - State Machine implementation
- Thomas Schwabe
    - color calibration
    - implementation 
- Florian Pfleiderer
    - documentation
    - localisation algorithm and implementation
