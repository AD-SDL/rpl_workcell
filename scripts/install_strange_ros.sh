#!/usr/bin/env bash

source /opt/ros/humble/install.bash
ROS_WS=~/wei_ws

mkdir -p $ROS_WS
mkdir -p $ROS_WS/src
cd $ROS_WS/src


##PF400
git clone https://github.com/AD-SDL/pf400_module

##WEI
git clone https://github.com/AD-SDL/wei_ros
git clone https://github.com/AD-SDL/wei

## Camera Module
git clone https://github.com/AD-SDL/camera_module.git
cd camera_module/camera_module_client
pip install .


cd ..
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build


cd $ROS_WS/src/wei
pip3 install -r requirements/requirements.txt
pip3 install -e .
cd $ROS_WS
