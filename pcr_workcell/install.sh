#!bin/bash

ROS_WS=~/wei_ws_week3

mkdir -p $ROS_WS
mkdir -p $ROS_WS/src
cd $ROS_WS/src
git clone https://github.com/AD-SDL/azenta_module
git clone https://github.com/AD-SDL/hudson_module
git clone https://github.com/AD-SDL/pf400_module
git clone https://github.com/AD-SDL/ot2_driver
git clone https://github.com/AD-SDl/ot2_driver_ros # to be merged with driver later
git clone https://github.com/AD-SDL/wei_ros
git clone https://github.com/AD-SDL/rpl_wei
cd $ROS_WS/src/rpl_wei
pip3 install -r requirements/requirements.txt
pip3 install -e .
cd $ROS_WS/src
git clone https://github.com/AD-SDL/rpl_workcell
cd $ROS_WS
colcon build

