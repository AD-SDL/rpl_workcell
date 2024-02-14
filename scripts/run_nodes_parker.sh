#!/bin/bash

session="nodes"
tmux new-session -d -s $session
tmux set -g mouse on




window=0
tmux new-window -t $session:$window -n 'sealer'
tmux send-keys -t $session:$window  'cd ~/workspace/a4s_sealer_module/scripts' C-m
tmux send-keys -t $session:$window  "python3 -m a4s_sealer_rest_client  --host="parker.cels.anl.gov"  --port=2000" C-m

window=1
tmux new-window -t $session:$window -n 'peeler'
tmux send-keys -t $session:$window  'cd ~/workspace/brooks_xpeel_module/scripts' C-m
tmux send-keys -t $session:$window   'python3 -m brooks_xpeel_rest_client  --host="parker.cels.anl.gov" --port=2001' C-m

window=2
tmux new-window -t $session:$window -n 'sciclops'
tmux send-keys -t $session:$window  'cd ~/workspace/hudson_platecrane_module/scripts' C-m
tmux send-keys -t $session:$window  'python3 -m sciclops_rest_client  --host="parker.cels.anl.gov" --port=2002' C-m

window=3
tmux new-window -t $session:$window -n 'ot2_alpha'
tmux send-keys -t $session:$window  'cd ~/workspace/ot2_module/scripts/' C-m
tmux send-keys -t $session:$window  'python3 -m ot2_rest_client --alias="ot2_pcr_alpha" --host="parker.cels.anl.gov" --ot2_ip="146.137.240.101" --port=2003' C-m

window=4
tmux new-window -t $session:$window -n 'ot2_beta'
tmux send-keys -t $session:$window  'cd ~/workspace/ot2_module/scripts/' C-m
tmux send-keys -t $session:$window  'python3 -m ot2_rest_client --alias="ot2_gc_beta" --host="parker.cels.anl.gov" --ot2_ip="146.137.240.100" --port=2004' C-m

window=5
tmux new-window -t $session:$window -n 'ot2_gamma'
tmux send-keys -t $session:$window  'cd ~/workspace/ot2_module/scripts/' C-m
tmux send-keys -t $session:$window  'python3 -m ot2_rest_client --alias="ot2_cp_gamma"  --host="parker.cels.anl.gov"  --ot2_ip="146.137.240.102" --port=2005' C-m

window=6
tmux new-window -t $session:$window -n 'PF400'
tmux send-keys -t $session:$window 'cd ~/workspace/pf400_module/scripts' C-m
tmux send-keys -t $session:$window 'python3 pf400_rest_client.py --host="parker.cels.anl.gov" --port 3000 --alias pf400' C-m

window=7
tmux new-window -t $session:$window -n 'camera_module'
tmux send-keys -t $session:$window 'cd ~/workspace/camera_module/scripts' C-m
tmux send-keys -t $session:$window 'python3 camera_rest_client.py --port=3001 --host="parker.cels.anl.gov" --alias camera_module --camera_url=0' C-m


tmux attach-session -t $session

