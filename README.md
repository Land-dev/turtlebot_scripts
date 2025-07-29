# TurtleBot Scripts

A collection of automation scripts for managing ROS (Robot Operating System) environments, TurtleBot fleets, and OptiTrack motion capture systems.

## 📋 Overview

This repository contains bash scripts designed to streamline the management of:
- **ROS Core and Bridge**: Start/stop ROS 1 core and ROS 1/2 bridge services
- **TurtleBot Fleet**: Manage multiple TurtleBot robots simultaneously
- **OptiTrack System**: Launch motion capture processing and ROS 2 nodes

## 🚀 Quick Start

### Prerequisites

- **ROS Noetic** (ROS 1) installed
- **ROS 2 Foxy** installed
- **tmux** for session management
- **sshpass** for automated SSH connections
- Network access to TurtleBot robots

### Installation

1. Clone this repository:
```bash
git clone https://github.com/Land-dev/turtlebot_scripts.git
cd turtlebot_scripts
```

2. Make scripts executable:
```bash
chmod +x *.sh
```

## �� Scripts Overview

### 1. `ros.sh` - ROS Core and Bridge Management

Manages ROS 1 core and ROS 1/2 bridge services using tmux sessions.

**Usage:**
```bash
./ros.sh start-core     # Start roscore in tmux
./ros.sh stop-core      # Stop roscore session
./ros.sh start-bridge   # Start rosbridge in tmux
./ros.sh stop-bridge    # Stop rosbridge session
```

**Features:**
- Automatic IP configuration (currently set to `192.168.0.105`)
- Background tmux session management
- ROS 1/2 bridge with dynamic topic bridging

### 2. `turtlebots.sh` - TurtleBot Fleet Management

Launches multiple TurtleBot robots simultaneously in a tmux session.

**Usage:**
```bash
# Use default IP addresses (192.168.0.141-146)
./turtlebots.sh

# Specify custom IP addresses
./turtlebots.sh 192.168.0.141 192.168.0.142 192.168.0.143
```

**Features:**
- Multi-robot management in single tmux session
- Automatic SSH connection with password authentication
- Environment variable setup for each robot
- Configurable robot numbering (`TURTLEBOT_NBR`)

**Configuration:**
- Set `PASSWORD` environment variable or use default `pw`
- Default ROS master: `http://192.168.0.105:11311`
- Default TurtleBot IPs: `192.168.0.141-146`

### 3. `optitrack.sh` - OptiTrack Motion Capture

Manages OptiTrack motion capture system with ROS 2 integration.

**Usage:**
```bash
./optitrack.sh start    # Start OptiTrack system
./optitrack.sh stop     # Stop OptiTrack system
```

**Features:**
- Dual-pane tmux session for motion capture processing
- ROS 2 launch file execution
- Python script for motion capture data processing

## �� Configuration

### Network Setup

Update IP addresses in the scripts according to your network configuration:

- **ROS Master IP**: Edit `ROS_MASTER_URI` in `ros.sh` and `turtlebots.sh`
- **TurtleBot IPs**: Modify the `TURTLEBOT_IPS` array in `turtlebots.sh`
- **Local IP**: Update `ROS_IP` in `ros.sh`

### Environment Variables

- `PASSWORD`: SSH password for TurtleBot robots (default: `pw`)
- `ROS_MASTER_URI`: ROS master URI (default: `http://192.168.0.105:11311`)

## 📖 Usage Examples

### Complete Setup Workflow

1. **Start ROS Core:**
```bash
./ros.sh start-core
```

2. **Start ROS Bridge:**
```bash
./ros.sh start-bridge
```

3. **Launch TurtleBot Fleet:**
```bash
./turtlebots.sh
```

4. **Start OptiTrack (if needed):**
```bash
./optitrack.sh start
```

### Session Management

**Attach to tmux sessions:**
```bash
# TurtleBot session
tmux attach -t turtlebots_session

# OptiTrack session
tmux attach -t optitrack_session

# ROS core session
tmux attach -t roscore_session

# ROS bridge session
tmux attach -t rosbridge_session
```

**Detach from tmux sessions:**
- Press `Ctrl+B` then `D`

## ��️ Troubleshooting

### Common Issues

1. **SSH Connection Failed:**
   - Verify TurtleBot IP addresses are correct
   - Check network connectivity
   - Ensure SSH password is correct

2. **ROS Master Not Found:**
   - Verify ROS core is running: `./ros.sh start-core`
   - Check ROS_MASTER_URI configuration
   - Ensure network connectivity to ROS master

3. **Tmux Session Already Exists:**
   - Scripts automatically kill existing sessions
   - Manually kill: `tmux kill-session -t <session_name>`

### Debug Commands

```bash
# Check ROS master status
rostopic list

# Verify TurtleBot connections
rosnode list

# Check tmux sessions
tmux list-sessions
```


**Note:** These scripts are designed for a specific network configuration. Please update IP addresses and credentials according to your setup before use.