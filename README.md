## beginner_tutorials
<p align="center">
<a href='https://opensource.org/licenses/MIT'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
</p>

## Overview
This is a beginner's project in which a ROS package is created. The package has two nodes, one topic, offers a service, and has two unit tests: node one - publisher (talker), node 2 - subscriber (listener), and topic (chatter). The talker publishes a string message on chatter and listener subscribes to chatter. Using a service change_text the string message could be changed. Talker node also has two unit tests for testing the service. It also displays various logging messages while running the node. It also broadcasts tf frame talk with a non-zero translation and rotation with respect to the world frame. To record the data that is being published, rosbag could be launched from a launch file within the package.

## Dependencies
1. ROS Kinetic - to install ROS follow the [link](http://wiki.ros.org/kinetic/Installation)

2. catkin - to install catkin run the following command
```
sudo apt-get install ros-kinetic-catkin
```
3. Package Dependencies
 a. roscpp
 b. std_msgs
 c. rostest
 d. tf

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

## Stop the nodes
To stop, press Ctrl+C and then enter the following command to clean up the nodes from the rosnode list
```
rosnode cleanup
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

## TF frames broadcasting verification
To see the boardcasting of TF frames by the talker node run the following command in a new terminal. Make sure the talker node is running.
```
rosrun tf tf_echo /world /talk
```
To verify the tf frames runtime run the following command in a new terminal. Make sure the talker node is running
```
rosrun rqt_tf_tree rqt_tf_tree
```
To generate a pdf containing tf frames tree using view_frames run the following command in a new terminal. Make sure the talker node is running.
```
rosrun tf view_frames
```
To view the pdf output run the following command.
```
evince frames.pdf
```

## Testing using rostest
To run the tests written for talker node run the following commands in a new terminal. It will show the output after ROS runs the tests.
```
cd ~/catkin_ws
source ./devel/setup.bash
catkin_make run_tests_beginner_tutorials
rostest beginner_tutorials talkerTest.launch
```

## Using rosbag to record and replay the messages
To start recoding the messages using rosbag run the following commands in a new terminal. Make sure talker node is running.
```
roslaunch beginner_tutorials launch_file.launch record:=true
```
The recording can be stopped by pressing Ctrl+C and the messages will be saved in results/recordedData.bag file. To disable the recording option while launching the launch_file run the following command.
```
roslaunch beginner_tutorials launch_file.launch record:=false
```

To play the recorded messages run the following command in a new terminal. To ensure that it is publishing messages also keep listenser node running.
```
cd ~/catkin_ws/src/beginner_tutorials/results
rosbag play recordedData.bag
```
