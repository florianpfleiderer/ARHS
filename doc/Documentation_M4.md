# Robohockey Documentation M4
**Authors:**</br>
Heinrich Fuhrmann 11940304 </br>
Thomas Schwabe 11918466</br>
Florian Pfleiderer 11771070</br>

**Version:** 
2023-06-01

## Milestone 4: Puck manipulation, scoring, and referee communication
### General Overview
After Milestone 3, we discussed, that having everything working in nodes, and every node subscribing to their own topics is slowing down our process and making it difficult for us to follow our principle of decoupled, object orientated code, which is easy to maintain. 

So the first step was to implement smach again. These states should then be used to implement their functionality, but in contrast to our previous solution, the states work together to form a working system.
We then decided for a Field Class, that inherits from FieldObject, to store all of our Data and Locations. 
This Class would provide as "gateway" between our working logic and all the scanners (Kinect / Laser).

Also, the Field should provide the offset calculations between two "Point Clouds" to update and determine the positions on the field.

### SMACH
The State Machine, which is shown in smach_v02.png in images/, uses the states FIND_DESTINATION and MOVE_TO_DESTINATION both for the puck and the goal.
To achieve this, the goal message for FIND_DESTINATION is a string target_type, which can be "BluePuck", etc.
This is decided by the states GET_GAME_SETUP, RELEASE_PUCK, MOVE_TO_DESTINATION in their result statements.
#### *GetGameSetup*
This State is the starting point of our state machine. It handles the referee communication:
- detecting and sending our field color is done by evaluating the poles closest to the robot at the start
- calculating the field dimensions is done through the field class, which itself does these calculations when initialised

result: 
- string target_type 
    - type of object the next server has to find

transition:
- succeeded: FIND_DESTINATION

#### *FindDestination*
This State gets as a Goal the target to look for. The planned method is for the field class to have indices for every object on the field and this server supplies the next state with an index of the object to drive to. 
This is made possible by the Field Class, which also stores positions of all the objects.
- detecting the first component according to "target_type"
- getting the index of set object 
    
goal: 
- string target_type
    - type of object the next server has to find
    
result: 
- int_8 index
    - the index of the target_component

transition:
- succeeded: MOVE_TO_DESTINATION
- aborted: FIND_DESTINATION

#### *MoveToDestination*
The Plan for this state is to implement a repulsive field algorithm, as it is very effective.
Finding the Pucks and the Goal will be determined by the target_component, which will either be a Puck or Goal, which has its position stored in the field object. 
This state needs to transitions, either to RELEASE_PUCK or to FIND_DESTINATION, depending on the target_type.
- repulsive field for path planning
- deciding the next state depending on the target_type

goal:
- FieldComponent target_component - the target component to drive to
    
result: 
- target_color - the color of the target component when reached or None if not found

transition:
- puck_reached: FIND_DESTINATION -> target_type: GOAL
- goal_reached: RELEASE_PUCK
- aborted: MOVE_TO_DESTINATION

#### *ReleasePuck*
This state is used to release the puck. It is called after the robot has reached the goal and has driven into it.
- releasing the puck

goal:
- string target_type - the target component to drive to

result:
- target_color - the color of the target component when reached or None if not found

### Issues 
**Issue1** <br>
When implementing the State Machine, we did not now which data is passed best as userdata. First, a FieldComponent Type was used and sent through the states, but between find_destination and move_to_destination, just sending an field component does not work, because it is converted into a message class and then again converted into a python class by the move_to_destination server and thus, we cannot make sure, the same "object" is passed to the next server.
The last approach is to pass the index of the target in the objects array of the field class, and making sure, that the field class is consistent all the way through our state machine.

**Issue2** <br>
Poles Ratios: The Ration between the Distances between the outer-most three poles and the very next set of three poles is 0.6 and 0.7 respectively, so with some inaccuracy in the laser_scan, the poles can easily be falsely detected and this can - for example - result in a much smaller field if the inner poles are detected as if they are the outer most ones. We have not really found a way through this issue.

**Issue3** <br>
The object detection algorithm still lags a lot, especially when turning. This will be evaluated through timing different functions and detecting the bottlenecks of the code. For timing functions, the perf_counter() from the time module will be used.

**Issue4** <br>
Implementing two different outcomes for the move_to_destination_server. This varies depending if the target was a puck or goal. The Algorithm for driving there does not change, as you go as close to the puck until you touch it, which would be the middle of the goal, where you can easily drop the puck.
For this, a result_cb callback was implemented to reach the possible outcomes.

### Contributions
- Heinrich Fuhrmann
    - field class implementation
    - offset calculation and field update for position detection
- Thomas Schwabe
    - state machine setup and implementation
    - referee communication
    - end of game handling
- Florian Pfleiderer
    - documentation
    - state machine implementation and transitions
    - release puck, find destination, move to destination basic implementation

