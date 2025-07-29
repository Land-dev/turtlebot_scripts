#!/bin/bash

# --- CONFIGURATION ---

# Dynamically get local IP
# HOST_IP=$(hostname -I | tr ' ' '\n' | grep '^192\.168\.' | head -n1)
# ROS_MASTER_URI="http://$HOST_IP:11311"

ROS_MASTER_URI="http://192.168.0.105:11311"

ROS_IP="192.168.0.105"



export ROS_IP=$ROS_IP
export ROS_MASTER_URI=$ROS_MASTER_URI

echo "ROS_IP=$ROS_IP"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"

# --- FUNCTIONS ---

start_roscore() {
    echo "Starting roscore in background..."
    tmux new-session -d -s roscore_session "bash -i -c 'export ROS_IP=$ROS_IP; export ROS_MASTER_URI=$ROS_MASTER_URI; source /opt/ros/noetic/setup.bash && roscore'"
}

stop_roscore() {
    echo "Stopping roscore tmux session..."
    tmux kill-session -t roscore_session 2>/dev/null || echo "No roscore session found."
}

start_rosbridge() {
    echo "Starting rosbridge in background..."
    tmux new-session -d -s rosbridge_session "bash -i -c 'export ROS_IP=$ROS_IP; export ROS_MASTER_URI=$ROS_MASTER_URI; source /opt/ros/noetic/setup.bash; source /opt/ros/foxy/setup.bash; ros2 run ros1_bridge dynamic_bridge --bridge-all-topics'"
}

stop_rosbridge() {
    echo "Stopping rosbridge tmux session..."
    tmux kill-session -t rosbridge_session 2>/dev/null || echo "No rosbridge session found."
}

# --- PARSE ARGUMENTS ---
case "$1" in
    start-core)
        start_roscore
        ;;
    stop-core)
        stop_roscore
        ;;
    start-bridge)
        start_rosbridge
        ;;
    stop-bridge)
        stop_rosbridge
        ;;
    *)
        echo "Usage:"
        echo "  $0 start-core     # Start roscore in tmux"
        echo "  $0 stop-core      # Stop roscore session"
        echo "  $0 start-bridge   # Start rosbridge in tmux"
        echo "  $0 stop-bridge    # Stop rosbridge session"
        exit 1
        ;;
esac
