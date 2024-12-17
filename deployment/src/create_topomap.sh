#!/bin/bash

# Create a new tmux session
session_name="nomad_px4_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into three panes
tmux selectp -t 0    # Select the first (0) pane
tmux splitw -v -p 50 # Split it into two halves vertically
tmux selectp -t 0    # Go back to the first pane
tmux splitw -h -p 50 # Split the top pane horizontally

# First pane: Optional ROS 2 daemon start (if needed)
tmux select-pane -t 0
tmux send-keys "ros2 daemon start" Enter

# Second pane: Run the create_topomap.py script
tmux select-pane -t 1
tmux send-keys "conda activate nomad_deployment" Enter
tmux send-keys "python create_topomap.py --dt 1 --dir $1" Enter

# Third pane: Run ros2 bag play
tmux select-pane -t 2
tmux send-keys "mkdir -p ../topomaps/bags" Enter
tmux send-keys "cd ../topomaps/bags" Enter
tmux send-keys "ros2 bag play --rate 1.5 $2" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name
