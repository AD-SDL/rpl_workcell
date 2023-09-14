#!/bin/bash                                                                                                                                                                                     
                                                                                                                                                                                                
session="nodes"                                                                                                                                                                                 
tmux new-session -d -s $session                                                                                                                                                                 
tmux set -g mouse on                                                                                                                                                                            
                                                                                                                                                                                                


window=0
tmux new-window -t $session:$window -n 'PF400'
tmux send-keys -t $session:$window 'cd ~/workspace/pf400_module/scripts' C-m
tmux send-keys -t $session:$window 'python3 pf400_rest_client.py --host=strange.cels.anl.gov --port 3000 --alias pf400' C-m

window=1
tmux new-window -t $session:$window -n 'camera_module'
tmux send-keys -t $session:$window 'cd ~/workspace/camera_module/scripts' C-m
tmux send-keys -t $session:$window 'python3 camera_rest_client.py --port=3001 --host=strange.cels.anl.gov --alias camera_module --camera_url=0' C-m

window=2
tmux new-window -t $session:$window -n 'sciclops_camera'
tmux send-keys -t $session:$window 'cd ~/workspace/camera_module/scripts' C-m
tmux send-keys -t $session:$window 'python3 camera_rest_client.py --port=3002 --host=strange.cels.anl.gov --alias sciclops_camera --camera_url="rtsp://admin:123@rplcam4.cels.anl.gov:8554/profile0"' C-m

window=3
tmux new-window -t $session:$window -n 'overall_camera'
tmux send-keys -t $session:$window 'cd ~/workspace/camera_module/scripts' C-m
tmux send-keys -t $session:$window 'python3 camera_rest_client.py --port=3003 --host=strange.cels.anl.gov --alias overall_camera --camera_url="rtsp://admin:123@rplcam1.cels.anl.gov:8554/profile0"' C-m


window=4
tmux new-window -t $session:$window -n 'biometra_camera'
tmux send-keys -t $session:$window 'cd ~/workspace/camera_module/scripts' C-m
tmux send-keys -t $session:$window 'python3 camera_rest_client.py --port=3004 --host=strange.cels.anl.gov --alias biometra_camera --camera_url="rtsp://admin:123@rplcam5.cels.anl.gov:8554/profile0"' C-m


tmux attach-session -t nodes
