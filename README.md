# Turtlebot_ENPM808X
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Overview
This program is an example of a basic implementation of walker example algorithm much like the Roomba robot. The robot being used is a turtlebot. The turtlebot moves forward till it detects an obstacle in a predefined range. When it detects an obstacle, it rotates itself till the obstacle is out of range. There is an added functionality of recording a bag file which records every topic except the /camera/* topic. This bag file can be found in the results section.

# Software
This program is running on a device running Ubuntu 16.04 and ROS Kinetic.
* To install ROS kinetic, use this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* If you don't have catkin installed use this [link](http://wiki.ros.org/catkin)

In addition to the above dependencies, you need to have the turtlebot_gazebo simulation installed. It can be installed typing the following lines in the terminal:
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```

If you think you already have this installed or you want to check if the above step has worked, you can type the following in the terminal:
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```

# Build and Compile
This code has to run in a catkin workspace. If you don't have a catkin workspace created use the following command to create one:
```
mkdir -p ~/catkin_ws/src
```
If you already have a catkin workspace or have created one using the above line then:
```
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/Chinj17/turtlebot_ENPM808X.git
cd ..
catkin_make
source devel/setup.bash
```

# Running the code
After following the above mentioned steps and successfully creating catkin workspace, you can run the code by following these steps in a terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch turtlebot_ENPM808X roomba.launch
```

# Error checks

**cppcheck**
```
cd <path to directory>
cppcheck --std=c++11 $(find . -name \*.cpp -or -name \*.h | grep -vE -e "^./build/" -e "^./results/") &> cppcheck.txt
```
**Google C++ standards**
```
cd <path to directory>
cpplint $(find . -name \*.cpp | grep -vE -e "^./build/" -e "^./results") &> cpplint.txt
```
