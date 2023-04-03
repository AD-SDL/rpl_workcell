#!/bin/bash

session="nodes"
tmux new-session -d -s $session
tmux set -g mouse on

# window=0
# tmux new-window -t $session:$window -n 'sealerpeeler_camera'
# tmux rename-window -t $session:$window 'sealerpeeler_camera'
# tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
# tmux send-keys -t $session:$window 'ros2 launch camera_module_client camera_publisher.launch.py camera_name:=camera_sp camera_number:=1' C-m

# window=1
# tmux new-window -t $session:$window -n 'sealer'
# tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
# tmux send-keys -t $session:$window 'ros2 launch a4s_sealer_client a4s_sealer_client.launch.py sealer_port:=/dev/ttyUSB1' C-m

# window=2
# tmux new-window -t $session:$window -n 'peeler'
# tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
# tmux send-keys -t $session:$window 'ros2 launch brooks_peeler_client brooks_peeler_client.launch.py peeler_port:/dev/ttyUSB5' C-m

# window=3
# tmux new-window -t $session:$window -n 'sciclops'
# tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
# tmux send-keys -t $session:$window 'ros2 launch sciclops_module_client sciclops_module.launch.py' C-m

# window=4
# tmux new-window -t $session:$window -n 'ot2_alpha'
# tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
# tmux send-keys -t $session:$window 'ros2 launch ot2_module_client ot2_module.launch.py ip:=146.137.240.100 robot_name:=ot2_pcr_alpha' C-m

window=5
tmux new-window -t $session:$window -n 'liconic'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch liconic_client liconic_client.launch.py' C-m

window=6
tmux new-window -t $session:$window -n 'biometra'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch biometra_client biometra_client.launch.py' C-m

tmux attach-session -t $session

