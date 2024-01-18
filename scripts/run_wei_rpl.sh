#!/usr/bin/env bash

session="WEI"
folder="~/workspace/wei"
tmux new-session -d -s $session
tmux set -g mouse on

window=0
tmux rename-window -t $session:$window -n 'redis'
tmux send-keys -t $session:$window 'cd ' $folder C-m
# Start the redis server, or ping if it's already up
if [ "$(redis-cli ping)" != "PONG" ]; then
	tmux send-keys -t $session:$window 'envsubst < $folder/../redis.conf | redis-server -' C-m
fi

window=1
tmux new-window -t $session:$window -n 'worker'
tmux send-keys -t $session:$window 'cd ' $folder C-m
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'python3 -m wei.engine --workcell ~/workspace/rpl_workcell/workcell/rpl_modular_workcell.yaml' C-m

window=2
tmux new-window -t $session:$window -n 'server'
tmux send-keys -t $session:$window 'cd ' $folder C-m
tmux send-keys -t $session:$window 'python3 -m wei.server --workcell ~/workspace/rpl_workcell/workcell/rpl_modular_workcell.yaml' C-m

tmux attach-session -t $session

