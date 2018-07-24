# FURIguide_ws
ROS Workspace for Spring 2018 ASU FURI Campus Guide Robot  
Entire workspace tracked as a git repo for ease of use.  
Targets ROS Lunar and ROS Kinetic.

## External Dependencies
* apriltags-ros
* opencv3
* ros-cv-camera
* gazebo-ros
* tf2, tf2_geometry_msgs

## Running
1. run `git submodule init` and `git submodule update` to pull the apriltags_ros repository in
2. initialize your ROS environment (usually `./opt/ros/lunar/setup.bash`)
3. run `catkin_make`
