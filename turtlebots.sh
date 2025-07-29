#!/bin/bash

#NEED TO FIX DOES NOT WORK

SESSION_NAME="turtlebots_session"
PASSWORD="${PASSWORD:-pw}"  # Use env var PASSWORD or default 'pw'

# Default TurtleBot IPs - override by passing IPs as arguments
if [ "$#" -gt 0 ]; then
    TURTLEBOT_IPS=("$@")
else
    TURTLEBOT_IPS=(
        "192.168.0.141"
        "192.168.0.142"
        "192.168.0.143"
        "192.168.0.144"
        "192.168.0.145"
        "192.168.0.146"
    )
fi

ROS_MASTER_URI="http://192.168.0.105:11311"

echo "[INFO] Using ROS_MASTER_URI=$ROS_MASTER_URI"
echo "[INFO] Starting tmux session: $SESSION_NAME"

# Kill existing tmux session if it exists
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "[INFO] Killing existing tmux session $SESSION_NAME"
    tmux kill-session -t "$SESSION_NAME"
fi

# Start a new detached tmux session
tmux new-session -d -s "$SESSION_NAME" -n "turtlebots"

for idx in "${!TURTLEBOT_IPS[@]}"; do
    ip=${TURTLEBOT_IPS[$idx]}
    turtlebot_nbr=$((idx + 1))
    pane="${idx}"

    if [ "$idx" -gt 0 ]; then
        tmux split-window -v -t "${SESSION_NAME}:0"
        tmux select-layout -t "${SESSION_NAME}:0" tiled
        sleep 0.2
    fi

    # Properly escape quotes and export ROS_MASTER_URI
    # Also export TURTLEBOT_NBR and source ROS environments
    cmd="sshpass -p '$PASSWORD' ssh -o StrictHostKeyChecking=no -t ubuntu@$ip bash -i -c 'export ROS_MASTER_URI=\"$ROS_MASTER_URI\" && source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && export TURTLEBOT_NBR=$turtlebot_nbr && exec roslaunch turtlebot_bringup minimal.launch'"

    echo "[INFO] Sending command to tmux pane $pane for TurtleBot $turtlebot_nbr at $ip"
    tmux send-keys -t "${SESSION_NAME}:0.$pane" "$cmd" C-m
done

echo "[INFO] Attach to the tmux session with: tmux attach -t $SESSION_NAME"
