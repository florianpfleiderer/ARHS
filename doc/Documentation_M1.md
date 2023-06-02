# Robohockey Documentation M1
Author: Heinrich Fuhrmann 11940304

## Milestone 1: Movement and Navigation
Basic navigation of the robot is accomplished by using a *state machine* (SMACH) and the *repulsive field* method.
The state machine is located in /scripts/player_simple_sm.py and can be launched by calling the /launch/player_simple_sm.launch file.

Due to problems with creating a python package, all of the utility functions and modules were copied into the player_simple_sm.py file.

### State Machine
The state machine is written by using the SMACH module.
It is quite simple at this stage:
![Simple state machine](/2023-tokyo-town/doc/images/Robot_SM_M1.jpeg)

The state machine subscribes to and/or publishes on the topics "player/target_dist", "robot1/cmd_vel" and "robot1/LaserScan".
The namespaces will be modified at a later stage to assign the robot number dynamically.

### Utility Classes
    class VelocityCalculator

Here lies the repulsive field logic.
The class has a 
    get_force(self, target_dist, multiplier=1, min_distance=-1, max_distance=-1)
function that translates the given target_dist directly to a velocity input.

the input for *r* is calculated by $r_{target}cos(\theta)$, hindering the robot to drive away from the target if $\theta$ is large, i.e. the target is far to the side.

The target_dist should be given in polar coordinates (PolarVector2) with $\theta = 0$ meaning straight ahead. 
A negative multiplier will cause the force to point in the other direction, for moving away from an obstacle.

The laser scan is read directly from the topic and for each hit within range, the scan value is translated into a polar vector based on range and index.
The multiplier for the obstacle repulsive force is negative and indirectly proportional to the distance.
It is then added to the total input force for the robot.

Constants for thresholds and multipliers are defined at the top of the module for easy access.

Here is a diagram along with graphs for the repelling and attracting forces:
![calculate velocity graphs](/2023-tokyo-town/doc/images/calculate_velocity.jpeg)

    class PositionCalculator

This class should update "player/target_dist" according to "robot1/cmd_vel" and the time that has passed since the last update.
This odometric approach will probably be quite inaccurate in reality, so it should be replaced by an algorithm that uses the laser scan and the camera feed to locate itself within the playing field at a later stage.

    vector_utils.py

This module offers a few utility functions for vector calculations, and for converting between polar and cartesian coordinates.

## Issues
Most issues were faced while setting up ROS, SMACH and GitHub SSH.
Importing custom python packages is not working yet.

The idea of the design process was to keep functionalities separate.
At first, PositionCalculator, VelocityCalculator and the state machine each were a node that subscribed to and published on the topics described above and had a separate tick.
Mutual updating caused the modules to do calculations at an uncontrollable rate.

In the end, the only node left is the state machine. It defines the system tick and PositionCalculator and VelocityCalculator are only called once each tick. The state machine is the only node that can publish to topics.