## beginner_tutorials
<p align="center">
<a href='https://opensource.org/licenses/MIT'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
</p>

## Overview
This is a beginner's project in which a package is created. The package has two nodes and one topic: node one - publisher (talker), node 2 - subscriber (listener), and topic (chatter). The talker publishes a string message on chatter and listener subscribes to chatter.

## Dependencies
ROS Kinetic - to install ROS follow the [link](http://wiki.ros.org/kinetic/Installation)
catkin - to install catkin run the following command
```

sudo apt-get install ros-kinetic-catkin
```
## Build
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/ajeetwankhede/beginner_tutorials.git
cd ..
catkin_make
```

## Run
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

To stop, press Ctrl+c
