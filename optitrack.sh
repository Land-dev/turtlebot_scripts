#!/bin/bash
set -e

SESSION_NAME="optitrack_session"

start_optitrack() {
    echo "[INFO] Starting OptiTrack..."

    # Kill existing tmux session if running
    tmux kill-session -t $SESSION_NAME 2>/dev/null || true

    # Start a new tmux session
    tmux new-session -d -s $SESSION_NAME -n optitrack

    # Pane 0: ROS 2 launch
    tmux send-keys -t "${SESSION_NAME}:0.0" "source /opt/ros/foxy/setup.bash && source $HOME/ros2_ws/install/setup.bash && cd $HOME/ros2_ws/src/ros2-mocap_optitrack/launch && ros2 launch launch_z_up.py" C-m

    # Pane 1: process_mocap.py
    tmux split-window -v -t "${SESSION_NAME}:0"
    tmux select-layout -t "${SESSION_NAME}:0" tiled
    tmux send-keys -t "${SESSION_NAME}:0.1" "source /opt/ros/foxy/setup.bash && source $HOME/ros2_ws/install/setup.bash && cd $HOME/ros2_ws/src/tb_multi_robot_control/scripts && python3 process_mocap.py" C-m

    echo "[INFO] OptiTrack tmux session started in background (session name: $SESSION_NAME)."
    echo "You can attach with: tmux attach -t $SESSION_NAME"
}

stop_optitrack() {
    echo "[INFO] Stopping OptiTrack..."
    tmux kill-session -t $SESSION_NAME 2>/dev/null
    if [ $? -eq 0 ]; then
        echo "[INFO] OptiTrack session terminated successfully."
    else
        echo "[ERROR] No active OptiTrack session found."
    fi
}

# --- Entry point ---
case "$1" in
    start)
        start_optitrack
        ;;
    stop)
        stop_optitrack
        ;;
    *)
        echo "Usage: $0 {start|stop}"
        exit 1
        ;;
esac
