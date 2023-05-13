**for copy and pasting often used commands**

## run teleop node
rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/robot1/cmd_vel

## export ROS_MASTER
export ROS_MASTER_URI=http://xmg-01:11311
export ROS_MASTER_URI=http://xmg-02:11311

## launch stuff
roslaunch hockeysimulator start_hockey_world.launch
roslaunch hockeysimulator init_hockey_world.launch
