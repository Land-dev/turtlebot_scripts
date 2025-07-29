#!/bin/bash

SESSION_NAME="turtlebots_session"
PASSWORD="${PASSWORD:-pw}"  # Use env var PASSWORD or default 'pw'

# Default TurtleBot IPs
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

# Start detached tmux session
tmux new-session -d -s "$SESSION_NAME" -n "turtlebots"

for idx in "${!TURTLEBOT_IPS[@]}"; do
    ip="${TURTLEBOT_IPS[$idx]}"
    turtlebot_nbr=$((idx + 1))
    pane="$idx"

    # Create split windows
    if [ "$idx" -gt 0 ]; then
        tmux split-window -v -t "${SESSION_NAME}:0"
        tmux select-layout -t "${SESSION_NAME}:0" tiled
        sleep 0.2
    fi

    # 1. SSH in
    ssh_cmd="sshpass -p '$PASSWORD' ssh -o StrictHostKeyChecking=no -t ubuntu@$ip bash -i"
    tmux send-keys -t "${SESSION_NAME}:0.$pane" "$ssh_cmd" C-m

    # 2. Wait for login shell
    sleep 1

    # 3. Send export and roslaunch commands
    tmux send-keys -t "${SESSION_NAME}:0.$pane" "export ROS_MASTER_URI=\"$ROS_MASTER_URI\"" C-m
    tmux send-keys -t "${SESSION_NAME}:0.$pane" "export TURTLEBOT_NBR=$turtlebot_nbr" C-m
    tmux send-keys -t "${SESSION_NAME}:0.$pane" "source /opt/ros/noetic/setup.bash" C-m
    tmux send-keys -t "${SESSION_NAME}:0.$pane" "source ~/catkin_ws/devel/setup.bash" C-m
    tmux send-keys -t "${SESSION_NAME}:0.$pane" "roslaunch turtlebot_bringup minimal.launch" C-m

    echo "[INFO] Set up TurtleBot $turtlebot_nbr at $ip"
done

echo "[INFO] Attach to tmux with: tmux attach -t $SESSION_NAME"
