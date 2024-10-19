# Robohockey Documentation
<!-- Author: Florian Pfleiderer 11771070</br>

**Version:** 
2023-04-13 -->

## Milestone 1
### Step 1 - Simple Algorithm
After getting up to speed with the ROS System itself, my approach to the first Milestone was to first implement an algorithm
using the LaserScan.ranges array to detect near obstacles.
The first Problem that i encountered was that the low values were quite random, and i found out, that discarding values below
range_min solved the issue.
I then chose a range of about 70 degrees in front of the robot from which the smallest value would be calculated and used. 
This made sure, that the robot is not reacting to obstacles on the side.
### Step 2 - Control Architecture
After having run the PlayerNode successfully, i wanted to implement the Smach Control Architecture.
I looked at the smach_test files in Github and implemented something similar into my Repository.
After the SM worked, i was unsure how to implement user data to calculate the needed stuff, but soon after i read about the SimpleActionServer, 
i thought of implementing user data somewhere there and not in the state machine.
i connected the server to the needed Topics, but on the rqt_graph i saw that the topics where standalone and not part of the /robot1 namespace.
I inspected player_node and alle the launch files to come to the conclusion, that the simplest way to solve my problem was to start 
the servers and smach test all from a single launchfile and set the namespace to robot1 in the beginning of the launchfile.
This solved my issues.
### Step 3 - refining the model
For a better overview and readability of the code, i added the folder functios and refactored the original code in the Simple Action Server, 
so it calls different functions instead of calculating everything inside the execute() method of the server. 
After this i added docstrings containing a short definition of what the described class / method / function does.
Letting the Robot drive aroudn for a few Minutes i realised, that i gets stuck in Corners switching bewteen avoid left and avoid right ations.
To solve this i added a counter and if it exceeded a certain value, the robot will drive backwards and right for 5 seconds.