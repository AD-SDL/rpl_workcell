#!/bin/bash

source /opt/ros/humble/install.bash
ROS_WS=~/wei_ws

mkdir -p $ROS_WS
mkdir -p $ROS_WS/src
cd $ROS_WS/src


##PF400
git clone https://github.com/AD-SDL/pf400_module

##Cameras
git clone https://github.com/AD-SDL/camera_module

##WEI
git clone https://github.com/AD-SDL/wei_ros
git clone https://github.com/AD-SDL/rpl_wei


cd ..
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build