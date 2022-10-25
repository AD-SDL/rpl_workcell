#!/bin/bash


source /opt/ros/humble/install.bash
ROS_WS=~/wei_ws

mkdir -p $ROS_WS
mkdir -p $ROS_WS/src
cd $ROS_WS/src

##Peeler/Sealer
git clone https://github.com/AD-SDL/azenta_module

##Sciclops
git clone https://github.com/AD-SDL/sciclops_module

##PF400
git clone https://github.com/AD-SDL/pf400_module

##OT2
git clone https://github.com/AD-SDl/ot2_module
git clone https://github.com/AD-SDl/ot2_driver 
pip3 install -r ot2_driver/requirements.txt

##WEI
git clone https://github.com/AD-SDL/wei_ros
git clone https://github.com/AD-SDL/rpl_wei

##Thermocicler
sudo apt install mono-devel
pip3 install pythonnet
git clone https://github.com/AD-SDL/biometra_module

cd ..
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build


cd $ROS_WS/src/rpl_wei
pip3 install -r requirements/requirements.txt
pip3 install -e .
cd $ROS_WS

