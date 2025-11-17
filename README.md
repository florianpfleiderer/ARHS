# AHRS 

Porting to ROS 2 missing

# Setup commands 

Pull ROS Noetic image: `docker pull osrf/ros:noetic-desktop-full`

For GUI access: `xhost +local:root`



Run image and mount full workspace:
```
docker run -it \
  --gpus all \
  --name arhs_noetic \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/arhs_docker_ws:/root/arhs_ws \
  osrf/ros:noetic-desktop-full \
  bash
```

If your Device is using Nvidia Optimus, the nvidia GPU is unlikely to work within Docker: https://download.nvidia.com/XFree86/Linux-x86_64/535.54.03/README/primerenderoffload.html



Reenter the container: `docker start -ai arhs_noetic`

Inside the container:

```
# Source ROS 1 environment
source /opt/ros/noetic/setup.bash

# (One-time) install catkin_tools and git, rosdep, etc., if needed
apt update
apt install -y python3-catkin-tools git

rosdep init
rosdep update
```


# ROS Notes

## Installing and Configuring the Environment
Source Setup Files
```
source /opt/ros/<distro>/setup.bash
```
-> written in .bashrc

## create workspace
```
mkdir -p catkin_ws/src
cd catkin_ws
catkin init 
```
Using Catkin-Tools instead of catkin_make has many advantages including easier package managing

```
catkin build
```
will create devel and build folder
```
source devel/setup.bash 
```
needs to be run everytime (-> added to .bashrc)

To write source commands directly into bashrc:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /root/arhs_ws/devel/setup.bash" >> ~/.bashrc
```

## Navigating ROS
### rospack find <package_name>
ros + package
will the Path to package if found

### roscd <package>
ros + cd
changes your pwd to package directory

### rosls <package>
ros + ls
lists contents of that package

## building a package
not so important for now
Every Package contains build, devel, src Folders

Cmake Workflow:
```
mkdir build
cd build
cmake ..
make
```

## ROS Nodes
Nodes are the executables inside ROS Packages.
Use ROS client library to communicate with other Nodes.
Nodes can publish or subscribe to a _Topic_.
Nodes can also provide and use a _Service_.
Client libraries are rospy and roscpp.

### roscore
roscore = ros+core : master (provides name service for ROS) + rosout (stdout/stderr) + parameter server 
must be run at first when using ROS

### rosnode
rosnode = ros+node : ROS tool to get information about a node.
```
rosnode list
```
lists all nodes

```
rosnode info /<node_name>
```
more info about specific node eg.: where it publishes to

### rosrun
rosrun = ros+run : runs a node from a given package.
```
rosrun <node_name>
```
directly run a node

## ROS Topics
2 Nodes communicate over _ROS Topics_.
One Node _publishes_ and the other _subscribes_ to a certain Topic.
With **rqt_graph** (run from a new terminal window) these nodes an Topics are shown.

### rostopic
```
rostopic -h 
```
... to get a list of sub commands
#### rostopic echo </topic/name>
listens to one topic

#### rostopic list -v 
prints all available topics

## ROS Messages
#### rostopic type </topic>
retuns the type of a topic

### publish ROS Messages 
```
rostopic pub [topic] [msg_type] [args]
```

## ROS Services
Services allow nodes to send a _request_ and receive a _response_.
### rosservice
```
rosservice list         print information about active services
rosservice call         call the service with the provided args
rosservice type         print service type
rosservice find         find services by service type
rosservice uri          print service ROSRPC uri
```

### rosparam
store and manipulate data on the ROS Parameter Server
```
rosparam set            set parameter
rosparam get            get parameter
rosparam load           load parameters from file
rosparam dump           dump parameters to file
rosparam delete         delete parameter
rosparam list           list parameter names
```

## RQT
rqt provides different visual services 
eg rqt_graph, rqt_console & logger_level, rqt_plot

## Launch a Node
```
roslaunch
```
Launches a node as specified in the .launch file.
It is good Practice to have a launch dir with the .launch files although not necessary.
