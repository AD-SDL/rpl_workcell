#!/usr/bin/env bash

session="RPL_Workcell"
folder="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )/.."
tmux new-session -d -s $session
tmux set -g mouse on

window=0
tmux rename-window -t $session:$window 'redis'
tmux send-keys -t $session:$window 'cd ' $folder C-m
# Start the redis server, or ping if it's already up
if [ "$(redis-cli ping)" != "PONG" ]; then
	tmux send-keys -t $session:$window 'redis-server' C-m
fi

window=1
tmux new-window -t $session:$window -n 'engine'
tmux send-keys -t $session:$window 'cd ' $folder C-m
#tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'python3 -m wei.engine --workcell ./workcell/rpl_modular_workcell.yaml' C-m

window=2
tmux new-window -t $session:$window -n 'server'
tmux send-keys -t $session:$window 'cd ' $folder C-m
tmux send-keys -t $session:$window 'python3 -m wei.server --workcell ./workcell/rpl_modular_workcell.yaml' C-m

window=3
tmux new-window -t $session:$window -n 'colorpicker'
tmux send-keys -t $session:$window 'cd ' $folder C-m
#tmux send-keys -t $session:$window 'python3 ./color_picker_app/color_picker_application.py' C-m

tmux attach-session -t $session

