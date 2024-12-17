#!/bin/bash

# Create a new tmux session
session_name="px4_x500_cam_2dlidar_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into four panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves

tmux selectp -t 2    # select the new, second (2) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane

# Pane 0: Run PX4 SITL with logging (Gazebo Sim)
tmux select-pane -t "${session_name}:0.0"
tmux send-keys "cd ~/PX4-Autopilot" Enter
tmux send-keys "__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia make px4_sitl gz_x500_cam_2dlidar_baylands 2>&1 | tee px4_output.log" Enter

# Wait for "Ready for takeoff!" in the log file of PX4-Autopilot
wait_for_ready() {
    echo "Waiting for PX4 to be ready for takeoff..."
    while true; do 
        if tmux capture-pane -p -J -t "${session_name}:0.0" | grep -Fq "Ready for takeoff!"; then
            break
        fi 
            sleep 2
    done
}

# Pane 1: launch the drone offboard control
tmux select-pane -t "${session_name}:0.1"
tmux send-keys "cd ~/ws_offboard_control" Enter
tmux send-keys "source install/local_setup.bash" Enter
tmux send-keys "ros2 launch px4_ros_com NoMaD_control.launch.xml" Enter

# Run the teleop.py script in the third pane
tmux select-pane -t "${session_name}:0.2"
tmux send-keys "conda activate nomad_deployment" Enter
tmux send-keys "python navigate.py" Enter

# Pane 3: Run the PD controller script 
tmux select-pane -t "${session_name}:0.3"
tmux send-keys "conda activate nomad_deployment" Enter
tmux send-keys "python pd_controller.py" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name
