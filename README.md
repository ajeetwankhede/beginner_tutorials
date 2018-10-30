## Beginner_tutorials description
Beginner tutorial for creating a ROS package to publish custom string message

## Build
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ cd src/
$ git clone --recursive https://github.com/ajeetwankhede/beginner_tutorials.git
$ cd ..
$ catkin_make

## Run
To run the nodes separately run the following commands
# In your catkin workspace
$ cd ~/catkin_ws
$ catkin_make
$ source ./devel/setup.bash

# Make sure that a roscore is up and running:
$ roscore

# To run the publisher named "talker" enter the following command in a new terminal
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials talker

# To run the subscriber named "listener" enter the following command in a new terminal
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials listener

To stop, press Ctrl+c
