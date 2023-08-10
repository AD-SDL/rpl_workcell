#!/bin/bash

session="nodes"
tmux new-session -d -s $session
tmux set -g mouse on

# window=0
# tmux new-window -t $session:$window -n 'sealerpeeler_camera'
# tmux rename-window -t $session:$window 'sealerpeeler_camera'
# tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
#tmux send-keys -t $session:$window 'ros2 launch camera_module_client camera_publisher.launch.py camera_name:=camera_sp camera_number:=6' C-m

window=1
tmux new-window -t $session:$window -n 'sealer'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window  '~/wei_ws/src/ot2_module/ot2_module_client/ot2_module_client' C-m
tmux send-keys -t $session:$window  'python3 -m ot2_rest_client --node_name="ot2_cp_gamma" --ip="146.137.240.102"' C-m

window=2
tmux new-window -t $session:$window -n 'peeler'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch brooks_peeler_client brooks_peeler_client.launch.py peeler_port:=/dev/ttyUSB1' C-m

window=3
tmux new-window -t $session:$window -n 'sciclops'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window  'cd ~/wei_ws/src/platecrane_module/sciclops_module_client/sciclops_module_client'
tmux send-keys -t $session:$window 'uvicorn sciclops_rest_client:app --host 'parker.cels.anl.gov' --port=2000' C-m

window=4
tmux new-window -t $session:$window -n 'ot2_alpha'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window  'cd ~/wei_ws/src/ot2_module/ot2_module_client/ot2_module_client' C-m
tmux send-keys -t $session:$window  'python3 -m ot2_rest_client --node_name="ot2_cp_alpha" --ip="146.137.240.101" --port=2003' C-m

window=5
tmux new-window -t $session:$window -n 'ot2_beta'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window  'cd ~/wei_ws/src/ot2_module/ot2_module_client/ot2_module_client' C-m
tmux send-keys -t $session:$window  'python3 -m ot2_rest_client --node_name="ot2_cp_beta" --ip="146.137.240.100" --port=2002' C-m

window=6
tmux new-window -t $session:$window -n 'ot2_gamma'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window  'cd ~/wei_ws/src/ot2_module/ot2_module_client/ot2_module_client' C-m
tmux send-keys -t $session:$window  'python3 -m ot2_rest_client --node_name="ot2_cp_gamma" --ip="146.137.240.102" --port=2001' C-m


tmux attach-session -t $session

