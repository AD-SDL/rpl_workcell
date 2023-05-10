#!/bin/bash

source /opt/ros/humble/install.bash
ROS_WS=~/wei_ws

mkdir -p $ROS_WS
mkdir -p $ROS_WS/src
SRC_FOLDER=$ROS_WS/src 


##Peeler
cd $SRC_FOLDER
git clone https://github.com/AD-SDL/a4s_sealer_module
cd a4s_sealer_module/a4s_sealer_driver
pip install . 

##Sealer
cd $SRC_FOLDER
git clone https://github.com/AD-SDL/brooks_xpeel_module
cd brooks_xpeel_module
git pull
git checkout -b main
cd brooks_xpeel_driver
pip install . 

##Sciclops
cd $SRC_FOLDER
git clone https://github.com/AD-SDL/platecrane_module

##Cameras
git clone https://github.com/AD-SDL/camera_module

##OT2
git clone https://github.com/AD-SDl/ot2_module
git clone https://github.com/AD-SDl/ot2_driver 
pip3 install -r ot2_driver/requirements.txt

##Liconic
git clone https://github.com/AD-SDl/liconic_module
# pip3 install liconic_module/liconic_driver

##WEI
git clone https://github.com/AD-SDL/wei_ros
# git clone https://github.com/AD-SDL/rpl_wei

##Thermocicler
# sudo apt install mono-devel
# pip3 install pythonnet
# git clone https://github.com/AD-SDL/biometra_module

cd ..

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build

