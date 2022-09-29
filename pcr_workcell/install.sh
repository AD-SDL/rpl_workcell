#!bin/bash

ROS_WS=~/wei_ws

mkdir -p $ROS_WS
mkdir -p $ROS_WS/src
cd $ROS_WS/src
git clone https://github.com/AD-SDL/azenta_module
git clone https://github.com/AD-SDL/hudson_module
git clone https://github.com/AD-SDL/pf400_module
git clone --recurse-submodules https://github.com/AD-SDl/ot2_module 
git clone https://github.com/AD-SDL/wei_ros
git clone https://github.com/AD-SDL/rpl_wei
cd ..
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build


cd $ROS_WS/src/rpl_wei
pip3 install -r requirements/requirements.txt
pip3 install -e .
cd $ROS_WS

