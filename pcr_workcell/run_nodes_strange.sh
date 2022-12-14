#!/bin/bash

session="nodes"
tmux new-session -d -s $session
tmux set -g mouse on

window=0
tmux new-window -t $session:$window -n 'pf400'
tmux rename-window -t $session:$window 'pf400'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch pf400_client pf400_client.launch.py'  C-m

tmux attach-session -t $session

