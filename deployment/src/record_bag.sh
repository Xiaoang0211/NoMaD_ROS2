#!/bin/bash

# Create a new tmux session
session_name="record_bag_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into three panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane
tmux splitw -h -p 50 # split it into two halves

# Pane 0: Run PX4 SITL with logging (Gazebo Sim)
tmux select-pane -t "${session_name}:0.0"
tmux send-keys "cd ~/PX4-Autopilot" Enter
tmux send-keys "__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia make px4_sitl gz_x500_cam_2dlidar_baylands 2>&1 | tee px4_output.log" Enter

# Wait for "Ready for takeoff!" in the log file
wait_for_ready() {
    echo "Waiting for PX4 to be ready for takeoff..."
    while true; do 
        if tmux capture-pane -p -J -t "${session_name}:0.0" | grep -Fq "Ready for takeoff!"; then
            break
        fi 
            sleep 2
    done
}

# Start monitoring the log
wait_for_ready

# Pane 1: launch the drone offboard control
tmux select-pane -t "${session_name}:0.1"
tmux send-keys "cd ~/ws_offboard_control" Enter
tmux send-keys "source install/local_setup.bash" Enter
tmux send-keys "ros2 launch px4_ros_com NoMaD_control.launch.xml" Enter

# Change the directory to ../topomaps/bags and run the rosbag record command in the third pane
tmux select-pane -t "${session_name}:0.2"
tmux send-keys "cd ../topomaps/bags" Enter
tmux send-keys "ros2 bag record /usb_cam/image_raw -o $1" # change topic if necessary

# Attach to the tmux session
tmux -2 attach-session -t $session_name