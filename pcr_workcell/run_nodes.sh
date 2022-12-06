#!/bin/bash

session="nodes"
tmux new-session -d -s $session
tmux set -g mouse on

window=0
tmux new-window -t $session:$window -n 'pf400'
tmux rename-window -t $session:$window 'pf400'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch pf400_client pf400_client.launch.py'  C-m

window=1
tmux new-window -t $session:$window -n 'sealerpeeler'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch sp_module_client sp_module.launch.py' C-m

window=2
tmux new-window -t $session:$window -n 'sciclops'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch sciclops_module_client sciclops_module.launch.py' C-m

window=3
tmux new-window -t $session:$window -n 'ot2_alpha'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'export robot_ip=192.168.50.29' C-m #this need to become an arg on the roslaunch
tmux send-keys -t $session:$window 'export robot_name=ot2_pcr_alpha' C-m #this need to become an arg on the roslaunch
tmux send-keys -t $session:$window 'ros2 launch ot2_module_client ot2_module.launch.py' C-m

window=4
tmux new-window -t $session:$window -n 'ot2_beta'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'export robot_ip=192.168.50.197' C-m #this need to become an arg on the roslaunch
tmux send-keys -t $session:$window 'export robot_name=ot2_growth_beta' C-m #this need to become an arg on the roslaunch
tmux send-keys -t $session:$window 'ros2 launch ot2_module_client ot2_module.launch.py' C-m

window=5
tmux new-window -t $session:$window -n 'biometra'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch biometra_client biometra_client.launch.py' C-m

window=5
tmux new-window -t $session:$window -n 'protocol_window'

tmux attach-session -t $session

