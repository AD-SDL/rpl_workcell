source /opt/ros/humble/install.bash
ROS_WS=~/wei_ws

mkdir -p $ROS_WS
mkdir -p $ROS_WS/src
cd $ROS_WS/src

echo test

##Peeler
git clone https://github.com/AD-SDL/a4s_sealer_module
cd a4s_sealer_module/a4s_sealer_driver
pip install .

##Sealer
git clone https://github.com/AD-SDL/brooks_xpeel_module
cd brooks_xpeel_module/brooks_xpeel_driver
pip install .

##Sciclops
git clone https://github.com/AD-SDL/sciclops_module

##Cameras
git clone https://github.com/AD-SDL/camera_module

##OT2
git clone https://github.com/AD-SDl/ot2_module
git clone https://github.com/AD-SDl/ot2_driver
pip3 install -r ot2_driver/requirements.txt

##WEI
git clone https://github.com/AD-SDL/wei_ros

cd ..
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build
