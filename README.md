## beginner_tutorials
<p align="center">
<a href='https://opensource.org/licenses/MIT'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
</p>

## Overview
This is a beginner's project in which a package is created. The package has two nodes, one topic, and one service: node one - publisher (talker), node 2 - subscriber (listener), and topic (chatter). The talker publishes a string message on chatter and listener subscribes to chatter. The service change_text is used to modify the text message which is published by the talker node. 
Also, a launch file is present to launch both the talker and listener nodes together. It accepts an argument to change the frequecny of publishing the message by the node talker on the topic chatter.

## Dependencies
1. ROS Kinetic - to install ROS follow the [link](http://wiki.ros.org/kinetic/Installation)

2. catkin - to install catkin run the following command
```
sudo apt-get install ros-kinetic-catkin
```
3. Package Dependencies
 a. roscpp
 b. std_msgs

## Build
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/ajeetwankhede/beginner_tutorials.git
cd ..
cd ..
catkin_make
```

## Run the nodes together
To run the nodes together run the following commands
In your catkin workspace
```
cd ~/catkin_ws
catkin_make
source ./devel/setup.bash
```
Make sure that a roscore is up and running:
```
roscore
```
To run the publisher named "talker" and the subscriber named "listener" with an argument enter the following command in a new terminal
```
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch beginner_tutorials launch_file.launch frequency:=<value_type_double>
```

## Run the nodes separately
To run the nodes separately run the following commands
In your catkin workspace
```
cd ~/catkin_ws
catkin_make
source ./devel/setup.bash
```
Make sure that a roscore is up and running:
```
roscore
```
To run the publisher named "talker" enter the following command in a new terminal
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials talker
```

To run the subscriber named "listener" enter the following command in a new terminal
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials listener
```

## Use the service
To use the service to change the text of the message run the following command in a new terminal
```
cd ~/catkin_ws
source ./devel/setup.bash
rosservice call /change_text "new text"
```

## Visualize in rqt
To visualize the publish-subscribe relationships between the nodes as a graph run the following command in a new terminal
```
rosrun rqt_graph rqt_graph
```

## See the logging messages
To see the logging messages enter the following command in a new terminal
```
rosrun rqt_console rqt_console
```

## Stop the nodes
To stop, press Ctrl+C and then enter the following command to clean up the nodes from the rosnode list
```
rosnode cleanup
```
