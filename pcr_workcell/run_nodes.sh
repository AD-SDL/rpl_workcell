#!/bin/bash

session="nodes"
tmux new-session -d -s $session

window=0
tmux new-window -t $session:$window -n 'pf400_module'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch pf400_client pf400_client.launch.py' C-m

window=2
tmux rename-window -t $session:$window -n 'sp_module'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch sp_module_client sp_module.launch.py' C-m

window=2
tmux new-window -t $session:$window -n 'sciclops_module'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch sciclops_module_client sciclops_module.launch.py' C-m

window=3
tmux new-window -t $session:$window -n 'ot2_module'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'export robot_ip=192.168.50.197' C-m #this need to become an arg on the roslaunch
tmux send-keys -t $session:$window 'export robot_name=ot2_pcr_alpha' C-m #this need to become an arg on the roslaunch
tmux send-keys -t $session:$window 'ros2 launch ot2_module_client ot2_module.launch.py' C-m

# window=4
# tmux new-window -t $session:$window -n 'biometra_module'
#tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
# tmux send-keys -t $session:$window 'ros2 launch biometra_module_client biometra_module.launch.py' C-m

tmux attach-session -t $session

