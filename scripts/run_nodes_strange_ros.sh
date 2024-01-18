#!/usr/bin/env bash

session="nodes"
tmux new-session -d -s $session
tmux set -g mouse on

window=0
tmux new-window -t $session:$window -n 'pf400'
tmux rename-window -t $session:$window 'pf400'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch pf400_client pf400_client.launch.py'  C-m

window=1
tmux new-window -t $session:$window -n 'camera_module'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch camera_module_client camera_publisher.launch.py camera_name:=camera_module camera_number:=9' C-m

window=2
tmux new-window -t $session:$window -n 'cam_over'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch camera_module_client camera_publisher.launch.py camera_name:=camera_all camera_number:=3' C-m

window=3
tmux new-window -t $session:$window -n 'cam_hudson'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch camera_module_client camera_publisher.launch.py camera_name:=camera_hudson camera_number:=15' C-m

window=4
tmux new-window -t $session:$window -n 'cam_pf400_left'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch camera_module_client camera_publisher.launch.py camera_name:=camera_pf400_left camera_number:=12' C-m

window=5
tmux new-window -t $session:$window -n 'cam_pf400_right'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch camera_module_client camera_publisher.launch.py camera_name:=camera_pf400_right camera_number:=7' C-m


window=6
tmux new-window -t $session:$window -n 'cam_thermo'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch camera_module_client camera_publisher.launch.py camera_name:=camera_thermo camera_number:=0' C-m


tmux attach-session -t $session
