# Robohockey Documentation
**Authors:**</br>
Heinrich Fuhrmann 11940304 </br>
Thomas Schwabe 11918466</br>
Florian Pfleiderer 11771070</br>

**Version:** 
2023-05-09

## Milestone 3: Field Dimension Detection and Localisation
### Localisation Algorithm
The Algorithm used for localisation is based on the specific field setup, which is a known environment.
It s based on the fact, that if you know the distance between two poles and divide by the distance of two other poles, the proportions tell you
which poles they are and therefore you can calculate your own position on the field.
Another important aspect for initial localisation is that the robot always starts in the red area, so you can detect the outer most pole, which also sets the origin of the coordinate system for localisation.
For further localisation, we can use the odometry data the robot receives through the cmd_vel topic. 

### Issues 
Spherical coordinates are in ISO 80000-2:2019 convention (used in physics, theta is angle from vertical z axis) and algorithm used mathematical convention.
Calculated ratios between Pole distances where off by more than 10 Percent, so there was a need to implement another Ratio for better proof, which poles were found. Also with using different poles and taking medians, the calculations where off by quite a margin, so we decided to use the Laser Scan Data.

### Contributions
- Heinrich Fuhrmann
    - Vector Calculations
    - state machine implementation
- Thomas Schwabe
    - color calibration
    - implementation 
- Florian Pfleiderer
    - documentation
    - localisation algorithm and implementation
